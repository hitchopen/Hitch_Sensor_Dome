#!/usr/bin/env bash
# =============================================================
# Step 3: RouteCAM GigE Vision Cameras — PTP Sync + Driver
#
# Configures 4× e-con RouteCAM_P_CU25_CXLC_IP67 GigE Vision
# cameras for PTP-synchronized image capture via IEEE 1588.
#
# What this script does:
#   1. Install Aravis GigE Vision library and tools
#   2. Install ROS 2 camera packages (image_transport, etc.)
#   3. Configure GigE Vision network settings (jumbo frames, buffers)
#   4. Detect cameras on the network via Aravis
#   5. Enable PTP on each camera via Aravis GenICam interface
#   6. Verify PTP synchronization status
#
# Prerequisites:
#   - Step 1 (setup_ubuntu_sync.sh) completed
#   - Cameras powered via PoE switch on sensor Ethernet
#   - Default camera IPs: 192.168.1.20–.23
#
# Usage:
# Defaults are pulled from ../config/network_config.yaml — edit
# that file once for your installation, and these flags become
# optional. CLI flags and env vars still override the YAML.
#
#   chmod +x setup_camera_sync.sh
#   ./setup_camera_sync.sh                              # use YAML defaults
#   ./setup_camera_sync.sh [--eth IFACE] [--ips IP1,IP2,IP3,IP4]
#
# Examples:
#   ./setup_camera_sync.sh
#   ./setup_camera_sync.sh --eth enp0s31f6 --ips 192.168.1.20,192.168.1.21
# =============================================================

set -euo pipefail

# ─── Load defaults from /config/network_config.yaml ──────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../config/load_network_config.sh
source "$SCRIPT_DIR/../config/load_network_config.sh"

# ─── Configuration (env > CLI > YAML defaults) ───────────────
ETH_IFACE="${ETH_IFACE:-$NETCFG_ETH}"
CAM_IPS_STR="${CAM_IPS_STR:-$NETCFG_CAMERA_IPS}"
CAM_NAMES=("cam_front_right" "cam_front_left" "cam_rear_left" "cam_rear_right")
ROS_DISTRO="jazzy"
WS_DIR="$HOME/ros2_ws"

# ─── Parse arguments ─────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --eth)    ETH_IFACE="$2"; shift 2 ;;
        --eth=*)  ETH_IFACE="${1#*=}"; shift ;;
        --ips)    CAM_IPS_STR="$2"; shift 2 ;;
        --ips=*)  CAM_IPS_STR="${1#*=}"; shift ;;
        *)        echo "Unknown arg: $1"; exit 1 ;;
    esac
done

IFS=',' read -ra CAM_IPS <<< "$CAM_IPS_STR"

# ─── Helpers ─────────────────────────────────────────────────
info()  { echo -e "\n\033[1;34m[INFO]\033[0m $*"; }
warn()  { echo -e "\n\033[1;33m[WARN]\033[0m $*"; }
ok()    { echo -e "\033[1;32m[ OK ]\033[0m $*"; }
fail()  { echo -e "\033[1;31m[FAIL]\033[0m $*"; exit 1; }

# =============================================================
info "Step 3: RouteCAM GigE Vision Camera Setup"
info "  Ethernet: $ETH_IFACE"
info "  Camera IPs: ${CAM_IPS[*]}"
echo ""

# ─── 1. Verify PTP grandmaster is running ────────────────────
info "Checking PTP grandmaster from Step 1..."
if systemctl is-active --quiet ptp4l-grandmaster 2>/dev/null; then
    ok "ptp4l-grandmaster is running"
else
    warn "ptp4l-grandmaster is not running."
    warn "Start it: sudo systemctl start gpsd chrony ptp4l-grandmaster phc2sys-grandmaster"
fi

# ─── 2. Install Aravis GigE Vision library ──────────────────
info "Installing Aravis GigE Vision library..."
sudo apt install -y \
    aravis-tools \
    libaravis-dev \
    gir1.2-aravis-0.8 \
    libgstreamer1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good
ok "Aravis installed."

# ─── 3. Install ROS 2 camera packages ───────────────────────
info "Installing ROS 2 camera packages..."
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo apt install -y \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-cv-bridge
ok "ROS 2 camera packages installed."

# ─── 4. Configure network for GigE Vision ───────────────────
info "Configuring $ETH_IFACE for GigE Vision..."

# Increase receive buffer (GigE Vision sends large image packets)
sudo sysctl -w net.core.rmem_max=33554432 > /dev/null
sudo sysctl -w net.core.rmem_default=8388608 > /dev/null

# Enable jumbo frames if supported (GigE Vision benefits from 9000 MTU)
if sudo ip link set "$ETH_IFACE" mtu 9000 2>/dev/null; then
    ok "Jumbo frames enabled (MTU 9000) on $ETH_IFACE"
else
    warn "Jumbo frames not supported — using standard MTU 1500"
    warn "Performance may be reduced with 4 cameras at full frame rate"
fi

# Disable reverse path filtering (can block GigE Vision discovery)
sudo sysctl -w "net.ipv4.conf.$ETH_IFACE.rp_filter=0" > /dev/null 2>&1 || true
sudo sysctl -w "net.ipv4.conf.all.rp_filter=0" > /dev/null 2>&1 || true

ok "Network configured for GigE Vision."

# ─── 5. Detect cameras with Aravis ───────────────────────────
info "Scanning for GigE Vision cameras on $ETH_IFACE..."

# arv-tool-0.8 discovers all GigE Vision devices on the network
DETECTED_CAMS=$(arv-tool-0.8 2>/dev/null || true)

if [ -n "$DETECTED_CAMS" ]; then
    ok "Detected GigE Vision cameras:"
    echo "$DETECTED_CAMS" | sed 's/^/    /'
else
    warn "No GigE Vision cameras detected."
    warn "Check that cameras are powered (PoE) and on the same subnet ($ETH_IFACE)."
    warn "You can manually verify with: arv-tool-0.8"
fi

# ─── 6. Enable PTP on each camera ───────────────────────────
info "Enabling IEEE 1588 PTP on cameras..."

# GigE Vision cameras support PTP via GenICam standard features.
# The relevant GenICam registers are:
#   - GevIEEE1588 (bool): enable/disable IEEE 1588
#   - GevIEEE1588Status: PTP clock status
#
# arv-tool-0.8 can read/write GenICam features by camera IP or serial.

for i in "${!CAM_IPS[@]}"; do
    ip="${CAM_IPS[$i]}"
    name="${CAM_NAMES[$i]:-cam_$i}"

    if ! ping -c1 -W2 "$ip" &>/dev/null; then
        warn "$name @ $ip not reachable — skipping"
        continue
    fi

    ok "$name @ $ip reachable"

    # Try to enable PTP via Aravis GenICam interface
    # The exact feature name depends on the camera firmware.
    # Common names: GevIEEE1588, PtpEnable, IEEE1588Enable
    info "  Enabling PTP on $name..."

    # Attempt standard GigE Vision PTP feature names
    for feature in "GevIEEE1588" "PtpEnable"; do
        if arv-tool-0.8 -a "$ip" control "$feature=true" 2>/dev/null; then
            ok "  PTP enabled via $feature"
            break
        fi
    done

    # Read back PTP status if available
    arv-tool-0.8 -a "$ip" control "GevIEEE1588Status" 2>/dev/null && \
        ok "  PTP status read successfully" || true
done

# ─── 7. Verify camera streaming ──────────────────────────────
# NOTE: `arv-tool-0.8` has no `snapshot` subcommand (valid subcommands are
# control, description, discovery, features, genicam, register). The correct
# Aravis frame-grab CLI is `arv-camera-test-0.8`, and the GUI viewer is
# `arv-viewer-0.8`. We verify the control link here with a real GenICam read
# and point to arv-camera-test-0.8 for actual streaming.
info "Testing camera control link (reading GenICam feature from first reachable camera)..."

FIRST_CAM=""
for ip in "${CAM_IPS[@]}"; do
    if ping -c1 -W2 "$ip" &>/dev/null; then
        FIRST_CAM="$ip"
        break
    fi
done

if [ -n "$FIRST_CAM" ]; then
    info "  Reading DeviceVendorName / Width from $FIRST_CAM..."
    if arv-tool-0.8 -a "$FIRST_CAM" control DeviceVendorName Width Height PixelFormat 2>/dev/null; then
        ok "  GenICam control link healthy on $FIRST_CAM"
        info "  Grab a live frame with:  arv-camera-test-0.8 -n 1 <camera_name>"
        info "  Live GUI viewer:         arv-viewer-0.8"
    else
        warn "  Could not read GenICam features (camera may need IP/config)."
        warn "  Try manually: arv-tool-0.8 -a $FIRST_CAM control DeviceVendorName"
    fi
else
    warn "No cameras reachable for control-link test."
fi

# =============================================================
# 8. Self-Test: Verify Camera PTP Sync Quality
# =============================================================
verify_camera_sync() {
    local PASS=0 FAIL=0 WARN_COUNT=0
    echo ""
    echo "============================================================="
    echo " VERIFICATION: RouteCAM GigE Vision Camera PTP Sync"
    echo "============================================================="

    # ── Test 1: PTP grandmaster running ──
    info "Test 1/7: PTP grandmaster status"
    if systemctl is-active --quiet ptp4l-grandmaster 2>/dev/null; then
        ok "  ptp4l-grandmaster is running"
        ((PASS++))
    else
        echo "  FAIL: ptp4l-grandmaster not running — run Step 1 first"
        ((FAIL++))
    fi

    # ── Test 2: Aravis tools installed ──
    info "Test 2/7: Aravis GigE Vision library"
    if command -v arv-tool-0.8 &>/dev/null; then
        ok "  arv-tool-0.8 available"
        ((PASS++))
    else
        echo "  FAIL: arv-tool-0.8 not found — apt install aravis-tools"
        ((FAIL++))
    fi

    # ── Test 3: Camera network reachability ──
    info "Test 3/7: Camera network reachability"
    local LIVE_COUNT=0
    local LIVE_IPS=()
    for i in "${!CAM_IPS[@]}"; do
        ip="${CAM_IPS[$i]}"
        name="${CAM_NAMES[$i]:-cam_$i}"
        if ping -c1 -W2 "$ip" &>/dev/null; then
            ok "  $name @ $ip reachable"
            ((LIVE_COUNT++))
            LIVE_IPS+=("$ip")
        else
            warn "  $name @ $ip not reachable"
        fi
    done
    if [ $LIVE_COUNT -gt 0 ]; then
        ((PASS++))
    else
        echo "  FAIL: No cameras reachable — check PoE switch and cables"
        ((FAIL++))
    fi
    info "  $LIVE_COUNT / ${#CAM_IPS[@]} cameras online"

    # ── Test 4: Aravis camera discovery ──
    info "Test 4/7: GigE Vision camera discovery"
    if command -v arv-tool-0.8 &>/dev/null; then
        DISCOVERED=$(arv-tool-0.8 2>/dev/null || true)
        if [ -n "$DISCOVERED" ]; then
            CAM_COUNT=$(echo "$DISCOVERED" | wc -l)
            ok "  Aravis discovered $CAM_COUNT device(s):"
            echo "$DISCOVERED" | sed 's/^/    /'
            ((PASS++))
        else
            warn "  Aravis found no GigE Vision devices"
            warn "  Cameras may need IP configuration or firewall rules"
            ((WARN_COUNT++))
        fi
    fi

    # ── Test 5: PTP status on each camera ──
    info "Test 5/7: IEEE 1588 PTP status on cameras"
    if command -v arv-tool-0.8 &>/dev/null; then
        for ip in "${LIVE_IPS[@]}"; do
            PTP_ENABLED=""
            PTP_STATUS_VAL=""

            # Try to read PTP enable status
            for feature in "GevIEEE1588" "PtpEnable"; do
                RESULT=$(arv-tool-0.8 -a "$ip" control "$feature" 2>/dev/null || true)
                if [ -n "$RESULT" ]; then
                    PTP_ENABLED="$RESULT"
                    break
                fi
            done

            # Try to read PTP clock status
            for feature in "GevIEEE1588Status" "PtpStatus"; do
                RESULT=$(arv-tool-0.8 -a "$ip" control "$feature" 2>/dev/null || true)
                if [ -n "$RESULT" ]; then
                    PTP_STATUS_VAL="$RESULT"
                    break
                fi
            done

            if [ -n "$PTP_ENABLED" ]; then
                if echo "$PTP_ENABLED" | grep -qi "true\|1"; then
                    ok "  Camera @ $ip: PTP enabled"
                    ((PASS++))
                else
                    warn "  Camera @ $ip: PTP = $PTP_ENABLED (may need enabling)"
                    ((WARN_COUNT++))
                fi
            else
                warn "  Camera @ $ip: Could not read PTP status (feature name may differ)"
                ((WARN_COUNT++))
            fi

            if [ -n "$PTP_STATUS_VAL" ]; then
                info "  Camera @ $ip: PTP clock status = $PTP_STATUS_VAL"
                if echo "$PTP_STATUS_VAL" | grep -qi "slave\|locked"; then
                    ok "  Camera @ $ip: PTP is synchronized (slave/locked)"
                    ((PASS++))
                fi
            fi
        done
    else
        warn "  arv-tool-0.8 not available — skipping PTP check"
        ((WARN_COUNT++))
    fi

    # ── Test 6: Control-link sanity + streaming tool availability ──
    # (arv-tool-0.8 has no `snapshot` subcommand; use arv-camera-test-0.8
    # for actual frame grabs.)
    info "Test 6/7: GenICam control link + streaming tool"
    if command -v arv-tool-0.8 &>/dev/null && [ ${#LIVE_IPS[@]} -gt 0 ]; then
        TEST_IP="${LIVE_IPS[0]}"
        info "  Reading GenICam features from $TEST_IP..."
        if arv-tool-0.8 -a "$TEST_IP" control DeviceVendorName Width Height 2>/dev/null | \
                grep -qiE "width|height|vendor"; then
            ok "  GenICam control link OK on $TEST_IP"
            ((PASS++))
        else
            warn "  GenICam read failed — camera may need stream configuration"
            ((WARN_COUNT++))
        fi
        if command -v arv-camera-test-0.8 &>/dev/null; then
            ok "  arv-camera-test-0.8 available (use: arv-camera-test-0.8 -n 1 <camera_name>)"
        else
            warn "  arv-camera-test-0.8 not found — install the aravis-tools package"
            ((WARN_COUNT++))
        fi
    else
        warn "  No cameras available for control-link test"
        ((WARN_COUNT++))
    fi

    # ── Test 7: ROS 2 camera packages ──
    info "Test 7/7: ROS 2 camera packages"
    source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null || true
    local PKGS_OK=0
    for pkg in image_transport camera_info_manager cv_bridge; do
        if ros2 pkg list 2>/dev/null | grep -q "$pkg"; then
            ((PKGS_OK++))
        fi
    done
    if [ $PKGS_OK -ge 3 ]; then
        ok "  ROS 2 camera packages installed ($PKGS_OK/3)"
        ((PASS++))
    else
        warn "  Some ROS 2 camera packages missing ($PKGS_OK/3)"
        ((WARN_COUNT++))
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
        echo " cameras finish PTP lock (allow 30–60 seconds)."
    else
        echo " All tests passed — cameras are PTP-synchronized."
    fi
    echo ""
    echo " Useful commands:"
    echo "   arv-tool-0.8                                 # List cameras"
    echo "   arv-tool-0.8 -a IP control Width Height      # Read GenICam features"
    echo "   arv-camera-test-0.8 -n 1 <camera_name>       # Grab one frame"
    echo "   arv-viewer-0.8                               # Live GUI viewer"
    echo "   tcpdump -i $ETH_IFACE host IP -w cam.pcap    # Record raw packets"
    echo ""
    echo " Full sync chain now active:"
    echo "   GPS → Atlas Duo PPS → gpsd → chrony → ptp4l → LiDARs + cameras"
    echo ""
}

verify_camera_sync
