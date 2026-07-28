#!/usr/bin/env bash
# =============================================================
# Section 4: PTP Sync from PC to LiDARs
#
# Configures up to 3 Seyond Robin W LiDARs as PTP slaves
# synchronized to the grandmaster built in Section 3, and
# installs the Seyond ROS 2 driver.
#
# Position in the sequence:
#   ./1_install_packages.sh
#   ./2_configure_host_network.sh
#   ./3_setup_ins_to_pc_sync.sh
#   ./4_setup_lidar_ptp.sh           ← you are here
#   ./5_setup_camera_ptp.sh
#
# One-time per-LiDAR provisioning (IP renumber + per-unit UDP
# port) is handled by a separate script:
#   ./provision_robin_w_multiunit.sh
# Run that ONCE for each new Robin W (or once for the dome
# when all three are first assembled). This script (Section 4)
# is the per-host setup you re-run on every fresh OS install
# and assumes provisioning has already happened.
#
# What this script does:
#   1. Verify PTP grandmaster is running (from Section 3)
#   2. Ping each LiDAR to confirm network connectivity
#   3. Enable PTP on each Robin W via innovusion_lidar_util
#   4. Set standard PTP mode (not automotive gPTP)
#   5. Clone and build the Seyond ROS 2 driver
#   6. Verify PTP slave synchronization
#
# Prerequisites:
#   - Sections 1–3 completed
#   - Robin W LiDARs powered on and connected to sensor LAN
#   - Default LiDAR IPs: 192.168.1.10, .11, .12
#
# Defaults are pulled from ../config/network_config.yaml — edit
# that file once for your installation, and these flags become
# optional. CLI flags and env vars still override the YAML.
#
# Usage:
#   chmod +x 4_setup_lidar_ptp.sh
#   ./4_setup_lidar_ptp.sh                              # use YAML defaults
#   ./4_setup_lidar_ptp.sh [--eth IFACE] [--ips IP1,IP2,IP3]
#   ./4_setup_lidar_ptp.sh --ros-distro humble
#
# Examples:
#   ./4_setup_lidar_ptp.sh
#   ./4_setup_lidar_ptp.sh --eth enp0s31f6 --ips 192.168.1.10,192.168.1.11
#   ./4_setup_lidar_ptp.sh --ips 192.168.1.10           # single LiDAR
# =============================================================

set -euo pipefail

# ─── Load defaults from /config/network_config.yaml ──────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../config/load_network_config.sh
source "$SCRIPT_DIR/../config/load_network_config.sh"

# ─── Configuration (env > CLI > YAML defaults) ───────────────
ETH_IFACE="${ETH_IFACE:-$NETCFG_ETH}"
LIDAR_IPS_STR="${LIDAR_IPS_STR:-$NETCFG_LIDAR_IPS}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS_DIR="$HOME/ros2_ws"
# Pinned because the source-contract checks below target this reviewed layout.
SEYOND_DRIVER_COMMIT="${SEYOND_DRIVER_COMMIT:-18c5c9362d41cd0766ee1b430f4b431bb14b1ccf}"

# ─── Parse arguments ─────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --eth)    ETH_IFACE="$2"; shift 2 ;;
        --eth=*)  ETH_IFACE="${1#*=}"; shift ;;
        --ips)    LIDAR_IPS_STR="$2"; shift 2 ;;
        --ips=*)  LIDAR_IPS_STR="${1#*=}"; shift ;;
        --ros-distro)   ROS_DISTRO="$2"; shift 2 ;;
        --ros-distro=*) ROS_DISTRO="${1#*=}"; shift ;;
        *)        echo "Unknown arg: $1"; exit 1 ;;
    esac
done

IFS=',' read -ra LIDAR_IPS <<< "$LIDAR_IPS_STR"

# ─── Helpers ─────────────────────────────────────────────────
info()  { echo -e "\n\033[1;34m[INFO]\033[0m $*"; }
warn()  { echo -e "\n\033[1;33m[WARN]\033[0m $*"; }
ok()    { echo -e "\033[1;32m[ OK ]\033[0m $*"; }
fail()  { echo -e "\033[1;31m[FAIL]\033[0m $*"; exit 1; }
validate_ros_distro() {
    case "$ROS_DISTRO" in
        humble|jazzy) ;;
        *) fail "Unsupported ROS_DISTRO='$ROS_DISTRO'. Use 'humble' or 'jazzy'." ;;
    esac
}
source_ros_setup() {
    [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] || \
        fail "ROS 2 $ROS_DISTRO not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    local rc=$?
    set -u
    return "$rc"
}
source_optional_setup() {
    set +u
    set +e
    # shellcheck disable=SC1090
    source "$1" 2>/dev/null
    local rc=$?
    set -e
    set -u
    return "$rc"
}

validate_ros_distro

# =============================================================
info "Section 4: PTP sync — PC to LiDARs"
info "  ROS distro: $ROS_DISTRO"
info "  Ethernet: $ETH_IFACE"
info "  LiDAR IPs: ${LIDAR_IPS[*]}"
echo ""

# ─── 1. Verify PTP grandmaster is running ────────────────────
info "Checking PTP grandmaster from Section 3..."
if systemctl is-active --quiet ptp4l-grandmaster 2>/dev/null; then
    ok "ptp4l-grandmaster is running"
else
    warn "ptp4l-grandmaster is not running. Starting it..."
    sudo systemctl start gpsd chrony ptp4l-grandmaster phc2sys-grandmaster 2>/dev/null || \
        fail "Could not start PTP stack. Run Section 3 first: ./3_setup_ins_to_pc_sync.sh"
    sleep 2
    if systemctl is-active --quiet ptp4l-grandmaster 2>/dev/null; then
        ok "ptp4l-grandmaster started"
    else
        fail "ptp4l-grandmaster failed to start. Check: sudo journalctl -u ptp4l-grandmaster"
    fi
fi

# ─── 2. Check LiDAR network connectivity ────────────────────
info "Pinging Robin W LiDARs..."
REACHABLE=()
for ip in "${LIDAR_IPS[@]}"; do
    if ping -c1 -W2 "$ip" &>/dev/null; then
        ok "Robin W @ $ip reachable"
        REACHABLE+=("$ip")
    else
        warn "Robin W @ $ip not reachable — skipping"
    fi
done

if [ ${#REACHABLE[@]} -eq 0 ]; then
    fail "No Robin W LiDARs reachable. Check power, cables, and IPs."
fi

# ─── 3. Enable PTP on each Robin W ──────────────────────────
info "Enabling PTP on reachable Robin W LiDARs..."

# Find the Seyond utility
UTIL=""
for candidate in \
    "innovusion_lidar_util" \
    "./innovusion_lidar_util" \
    "$HOME/seyond_sdk/bin/innovusion_lidar_util" \
    "$WS_DIR/src/seyond_ros_driver/lib/innovusion_lidar_util"; do
    if command -v "$candidate" &>/dev/null || [ -x "$candidate" ]; then
        UTIL="$candidate"
        break
    fi
done

if [ -z "$UTIL" ]; then
    warn "innovusion_lidar_util not found. Enable PTP manually on each LiDAR:"
    for ip in "${REACHABLE[@]}"; do
        echo "  innovusion_lidar_util $ip set_config time ptp_en 1"
        echo "  innovusion_lidar_util $ip set_config time ptp_automotive 0"
    done
    echo ""
    warn "You can also enable PTP via each LiDAR's web UI at http://<lidar_ip>"
else
    ok "Using utility: $UTIL"
    for ip in "${REACHABLE[@]}"; do
        info "Configuring Robin W @ $ip..."
        $UTIL "$ip" set_config time ptp_en 1 && \
            ok "  PTP enabled" || warn "  Failed to enable PTP"
        $UTIL "$ip" set_config time ptp_automotive 0 && \
            ok "  Standard PTP mode set (not automotive gPTP)" || warn "  Failed to set PTP mode"
    done
fi

# ─── 4. Build Seyond ROS 2 driver ───────────────────────────
info "Installing Seyond Robin W ROS 2 driver..."
source_ros_setup
mkdir -p "$WS_DIR/src"
cd "$WS_DIR/src"

if [ ! -d "seyond_ros_driver" ]; then
    git clone --recurse-submodules https://github.com/Seyond-Inc/seyond_ros_driver.git
fi

cd seyond_ros_driver
git fetch origin main
if [ "$(git rev-parse HEAD)" != "$SEYOND_DRIVER_COMMIT" ]; then
    git checkout --detach "$SEYOND_DRIVER_COMMIT"
fi
git submodule update --init --recursive

LEGACY_SEYOND_TIME_PATCH="$SCRIPT_DIR/seyond_robin_w_legacy_relative_time.patch"
if git apply --reverse --check "$LEGACY_SEYOND_TIME_PATCH" 2>/dev/null; then
    info "Removing the obsolete Robin W relative-time PointCloud2 override..."
    git apply --reverse "$LEGACY_SEYOND_TIME_PATCH"
    ok "Restored upstream timestamp/FLOAT64 absolute-seconds output"
elif git apply --check "$LEGACY_SEYOND_TIME_PATCH" >/dev/null 2>&1; then
    ok "Seyond driver already uses upstream timestamp/FLOAT64 output"
else
    fail "Seyond timestamp source differs from the reviewed $SEYOND_DRIVER_COMMIT"
fi

# Fail closed if a future source or local edit changes the ROS wire contract.
# The raw packet stores a compact point offset (`ts_10us`); the driver hydrates
# it with the packet's absolute PTP/GPS start before PCL creates PointCloud2.
SEYOND_DRIVER_SOURCE="src/seyond_lidar_ros/src/driver/driver_lidar.cc"
SEYOND_POINT_TYPES="src/seyond_lidar_ros/src/driver/point_types.h"
SEYOND_ROS2_ADAPTER="src/seyond_lidar_ros/src/driver/ros2_driver_adapter.hpp"
grep -Fq "double timestamp;" "$SEYOND_POINT_TYPES" || \
    fail "Seyond PointXYZIT no longer declares timestamp as double/FLOAT64"
grep -Fq "(double, timestamp, timestamp)" "$SEYOND_POINT_TYPES" || \
    fail "Seyond PCL registration no longer exports timestamp/FLOAT64"
grep -Fq \
    "point.timestamp = point_ptr->ts_10us / ten_us_in_second_c + current_ts_start_;" \
    "$SEYOND_DRIVER_SOURCE" || \
    fail "Seyond point timestamp is no longer absolute packet-start + point-offset seconds"
grep -Fq "frame_data.timestamp = frame_start_ts_;" "$SEYOND_DRIVER_SOURCE" || \
    fail "Seyond completed frames are no longer tagged with frame_start_ts_"
grep -Fq "int64_t ts_ns = timestamp * 1000;" "$SEYOND_ROS2_ADAPTER" || \
    fail "Seyond ROS 2 header.stamp is no longer derived from frame-start microseconds"
ok "Verified Robin W ROS contract: frame-start header + timestamp/FLOAT64 absolute Unix seconds"

info "Building Seyond driver (this may take a few minutes)..."
source_ros_setup
./build.bash

# Add to .bashrc
grep -q "source ~/ros2_ws/src/seyond_ros_driver/install/setup.bash" "$HOME/.bashrc" 2>/dev/null || \
    echo "source ~/ros2_ws/src/seyond_ros_driver/install/setup.bash" >> "$HOME/.bashrc"

ok "Seyond ROS 2 driver installed."

# ─── 5. Verify ───────────────────────────────────────────────
info "Verifying installation..."
source_optional_setup "$WS_DIR/src/seyond_ros_driver/install/setup.bash" || true

if ros2 pkg list 2>/dev/null | grep -q "seyond"; then
    ok "seyond ROS 2 package found"
else
    warn "seyond package not found in ros2 pkg list"
fi

# Check PTP slave status
info "Checking PTP slave status (LiDARs should appear as slaves)..."
sudo pmc -u -b 1 'GET PORT_DATA_SET' 2>/dev/null || \
    warn "Could not query PTP peers — run: sudo pmc -u -b 1 'GET PORT_DATA_SET'"

# =============================================================
# 6. Self-Test: Verify Robin W PTP Sync Quality
# =============================================================
verify_lidar_sync() {
    local PASS=0 FAIL=0 WARN_COUNT=0
    echo ""
    echo "============================================================="
    echo " VERIFICATION: Robin W LiDAR PTP Sync"
    echo "============================================================="

    # ── Test 1: PTP grandmaster running ──
    info "Test 1/6: PTP grandmaster status"
    if systemctl is-active --quiet ptp4l-grandmaster 2>/dev/null; then
        ok "  ptp4l-grandmaster is running"
        PASS=$((PASS+1))
    else
        echo "  FAIL: ptp4l-grandmaster not running"
        FAIL=$((FAIL+1))
    fi

    # ── Test 2: LiDAR network reachability ──
    info "Test 2/6: LiDAR network reachability"
    local LIVE_COUNT=0
    for ip in "${LIDAR_IPS[@]}"; do
        if ping -c1 -W2 "$ip" &>/dev/null; then
            ok "  Robin W @ $ip reachable"
            LIVE_COUNT=$((LIVE_COUNT+1))
        else
            warn "  Robin W @ $ip not reachable"
        fi
    done
    if [ $LIVE_COUNT -gt 0 ]; then
        PASS=$((PASS+1))
    else
        echo "  FAIL: No LiDARs reachable"
        FAIL=$((FAIL+1))
    fi
    info "  $LIVE_COUNT / ${#LIDAR_IPS[@]} LiDARs online"

    # ── Test 3: PTP enabled on each LiDAR ──
    info "Test 3/6: PTP enabled on LiDARs"
    if [ -n "${UTIL:-}" ] && [ -x "${UTIL:-}" ] 2>/dev/null || command -v "${UTIL:-}" &>/dev/null 2>/dev/null; then
        for ip in "${LIDAR_IPS[@]}"; do
            if ! ping -c1 -W1 "$ip" &>/dev/null; then continue; fi
            PTP_STATUS=$($UTIL "$ip" get_config time ptp_en 2>/dev/null || echo "unknown")
            if echo "$PTP_STATUS" | grep -q "1"; then
                ok "  Robin W @ $ip: PTP enabled"
                PASS=$((PASS+1))
            else
                warn "  Robin W @ $ip: PTP status = $PTP_STATUS"
                WARN_COUNT=$((WARN_COUNT+1))
            fi
        done
    else
        warn "  innovusion_lidar_util not available — cannot query PTP status directly"
        warn "  Check via web UI: http://<lidar_ip>"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    # ── Test 4: PTP slave detection ──
    info "Test 4/6: PTP slave devices on network"
    PTP_PORTS=$(sudo pmc -u -b 1 'GET PORT_DATA_SET' 2>/dev/null || true)
    if [ -n "$PTP_PORTS" ]; then
        SLAVE_COUNT=$(echo "$PTP_PORTS" | grep -c "SLAVE" || true)
        if [ "$SLAVE_COUNT" -gt 0 ]; then
            ok "  $SLAVE_COUNT PTP slave(s) detected"
            PASS=$((PASS+1))
        else
            warn "  No PTP slaves detected — LiDARs may still be syncing (wait 30–60 s)"
            WARN_COUNT=$((WARN_COUNT+1))
        fi
        echo "$PTP_PORTS" | grep -E "portIdentity|portState" | sed 's/^/    /'
    else
        warn "  Could not query PTP peers — slaves unverified"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    # ── Test 5: ptp4l master offset (sync quality) ──
    info "Test 5/6: ptp4l master offset quality"
    PTP_LOG=$(sudo journalctl -u ptp4l-grandmaster --no-pager -n 100 2>/dev/null || true)
    if [ -n "$PTP_LOG" ]; then
        # Extract last 10 master offset values
        OFFSETS=$(echo "$PTP_LOG" | grep -oP 'master offset\s+\K[-0-9]+' | tail -10)
        if [ -n "$OFFSETS" ]; then
            # Compute statistics
            MAX_ABS=0
            SUM=0
            COUNT=0
            for o in $OFFSETS; do
                ABS_O=${o#-}
                SUM=$((SUM + ABS_O))
                COUNT=$((COUNT + 1))
                if [ "$ABS_O" -gt "$MAX_ABS" ]; then MAX_ABS=$ABS_O; fi
            done
            if [ $COUNT -gt 0 ]; then
                AVG=$((SUM / COUNT))
                info "  Last $COUNT offsets: avg=${AVG} ns, max=${MAX_ABS} ns"
                if [ $MAX_ABS -lt 1000 ]; then
                    ok "  PTP offset < 1 µs — excellent (hardware timestamping)"
                    PASS=$((PASS+1))
                elif [ $MAX_ABS -lt 50000 ]; then
                    ok "  PTP offset < 50 µs — good (software timestamping)"
                    PASS=$((PASS+1))
                else
                    warn "  PTP offset > 50 µs — sync may be settling"
                    WARN_COUNT=$((WARN_COUNT+1))
                fi
            fi
        else
            warn "  No master offset values in ptp4l log yet"
            WARN_COUNT=$((WARN_COUNT+1))
        fi
    else
        warn "  Could not read ptp4l journal"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    # ── Test 6: Seyond ROS 2 driver ──
    info "Test 6/6: Seyond ROS 2 driver"
    source_ros_setup || true
    source_optional_setup "$WS_DIR/src/seyond_ros_driver/install/setup.bash" || true
    if ros2 pkg list 2>/dev/null | grep -q "seyond"; then
        ok "  seyond ROS 2 package found"
        PASS=$((PASS+1))
    else
        warn "  seyond package not found — rebuild: cd ~/ros2_ws/src/seyond_ros_driver && ./build.bash"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    # ── Results ──
    echo ""
    echo "============================================================="
    echo " RESULTS: $PASS passed, $WARN_COUNT warnings, $FAIL failed"
    echo "============================================================="
    if [ $FAIL -gt 0 ]; then
        echo " Some tests FAILED — review the output above."
    elif [ $WARN_COUNT -gt 0 ]; then
        echo " All critical tests passed. Warnings may resolve after"
        echo " LiDARs finish PTP sync (allow 30–60 seconds)."
    else
        echo " All tests passed — LiDARs are PTP-synchronized."
    fi
    echo ""
    echo " Quick test: launch a LiDAR in ROS 2:"
    echo "   source ~/.bashrc && ros2 launch seyond start.py"
    echo ""
    echo " Next: ./5_setup_camera_ptp.sh --eth $ETH_IFACE"
    echo ""
}

verify_lidar_sync
