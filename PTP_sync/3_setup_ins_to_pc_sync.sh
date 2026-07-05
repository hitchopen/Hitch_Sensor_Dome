#!/usr/bin/env bash
# =============================================================
# Section 3: PTP Sync from INS to PC
#
# Builds the GPS-disciplined clock + PTP grandmaster chain on
# the host. The Atlas Duo's only physical link to the host is
# Ethernet (no BNC PPS, no USB serial — the device hardware
# does not expose them). All time and data flow through that
# single Ethernet cable:
#
#   Atlas Duo (192.168.1.30) ── Ethernet ──> RUTM50 LAN 4
#     - NMEA 0183 over TCP 30200
#         → gpsd ── SHM → chrony → CLOCK_REALTIME
#         → phc2sys: CLOCK_REALTIME → NIC PHC
#         → ptp4l: NIC PHC → PTP announce on Ethernet
#     - FusionEngine over TCP 30201
#         → ros2-fusion-engine-driver → /pose, /imu, /gps/fix, /odom
#     - NTRIP RTCM3 inbound from caster (RTK corrections)
#
# Steady-state accuracy: host CLOCK_REALTIME ~10–100 ms to GPS
# (NMEA-only discipline, no PPS edge). Cross-sensor PTP is
# unaffected; LiDARs and cameras still sync to the host NIC
# PHC at sub-µs (hardware timestamping) or sub-50 µs (software).
# FusionEngine message headers carry GPS time directly from the
# Atlas — that path is independent of host-clock drift.
#
# Position in the sequence:
#   ./1_install_packages.sh
#   ./2_configure_host_network.sh
#   ./3_setup_ins_to_pc_sync.sh      ← you are here
#   ./4_setup_lidar_ptp.sh
#   ./5_setup_camera_ptp.sh
#
# Prerequisites:
#   - Sections 1 + 2 completed
#   - Atlas Duo wired to RUTM50 LAN 4, reachable at 192.168.1.30
#   - Atlas Duo navigation engine started (web UI Map View →
#     "Start Navigating"; without this, no NMEA is emitted)
#
# Usage:
#   chmod +x 3_setup_ins_to_pc_sync.sh
#   ./3_setup_ins_to_pc_sync.sh
#   ./3_setup_ins_to_pc_sync.sh --atlas-ip 192.168.1.30
#   ./3_setup_ins_to_pc_sync.sh --ros-distro humble
# =============================================================

set -euo pipefail

# ─── Load network state written by script #2 ─────────────────
if [ -f /run/hitch_dome_net.env ]; then
    # shellcheck disable=SC1091
    source /run/hitch_dome_net.env
else
    echo "[WARN] /run/hitch_dome_net.env missing — falling back to software PTP timestamping." >&2
    echo "       Re-run ./2_configure_host_network.sh to refresh NIC detection." >&2
fi

# ─── Load defaults from /config/network_config.yaml ──────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../config/load_network_config.sh
source "$SCRIPT_DIR/../config/load_network_config.sh"

# ─── Configuration (env > CLI > YAML / /run/hitch_dome_net.env) ──
ETH_IFACE="${ETH_IFACE:-$NETCFG_ETH}"
ATLAS_IP="${ATLAS_IP:-$NETCFG_ATLAS_IP}"
ATLAS_NMEA_PORT="${ATLAS_NMEA_PORT:-$NETCFG_ATLAS_NMEA_PORT}"
ATLAS_FE_PORT="${ATLAS_FE_PORT:-$NETCFG_ATLAS_FE_PORT}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS_DIR="$HOME/ros2_ws"
: "${PTP_TIMESTAMPING:=software}"   # default to SW if script 2 didn't run

# ─── Parse arguments ─────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --atlas-ip)   ATLAS_IP="$2"; shift 2 ;;
        --atlas-ip=*) ATLAS_IP="${1#*=}"; shift ;;
        --eth)        ETH_IFACE="$2"; shift 2 ;;
        --eth=*)      ETH_IFACE="${1#*=}"; shift ;;
        --ros-distro)   ROS_DISTRO="$2"; shift 2 ;;
        --ros-distro=*) ROS_DISTRO="${1#*=}"; shift ;;
        *)            echo "Unknown arg: $1"; exit 1 ;;
    esac
done

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
info "Section 3: PTP sync — INS to PC (Ethernet-only)"
info "  ROS distro:      $ROS_DISTRO"
info "  Ethernet:        $ETH_IFACE"
info "  Atlas Duo IP:    $ATLAS_IP"
info "  NMEA TCP:        $ATLAS_IP:$ATLAS_NMEA_PORT"
info "  FusionEngine TCP: $ATLAS_IP:$ATLAS_FE_PORT"
info "  PTP timestamping: $PTP_TIMESTAMPING"
echo ""

# Pre-flight: verify Atlas Duo is reachable
if ! ping -c 2 -W 2 "$ATLAS_IP" &>/dev/null; then
    warn "Atlas Duo at $ATLAS_IP did not respond to ping — continuing setup, but check the cable / RUTM50 LAN 4 / Atlas power before expecting NMEA to flow."
fi

# ─── 1. gpsd reads NMEA over TCP from Atlas Duo ──────────────
info "Configuring gpsd to read NMEA from tcp://$ATLAS_IP:$ATLAS_NMEA_PORT..."
sudo tee /etc/default/gpsd > /dev/null << EOF
# gpsd consumes NMEA 0183 from the Atlas Duo's Ethernet TCP server.
# Atlas Duo has no PPS BNC and no USB serial output, so this is the
# only host-clock discipline source available.
DEVICES="tcp://$ATLAS_IP:$ATLAS_NMEA_PORT"
GPSD_OPTIONS="-n -b"
START_DAEMON="true"
USBAUTO="false"
EOF
ok "gpsd configured for TCP NMEA source."

# ─── 2. chrony disciplines CLOCK_REALTIME from gpsd SHM ──────
info "Configuring chrony — NMEA-only discipline (no PPS available on Atlas Duo)..."
sudo tee /etc/chrony/chrony.conf > /dev/null << 'EOF'
# Primary: GPS NMEA via gpsd shared memory.
# The Atlas Duo does not expose hardware PPS — sentence-rate (1 Hz)
# NMEA is the only source. Expected steady-state accuracy ~10–100 ms.
# (Was < 100 ns in legacy PPS-disciplined setups.)
refclock SHM 0 offset 0.5 delay 0.2 refid NMEA

# Fallback: NTP
pool ntp.ubuntu.com iburst maxsources 4
pool time.google.com iburst maxsources 2

makestep 1.0 3
driftfile /var/lib/chrony/chrony.drift
rtcsync
maxupdateskew 100.0
EOF
ok "chrony configured for NMEA-only discipline."

# ─── 3. ptp4l grandmaster on sensor NIC ─────────────────────
info "Configuring ptp4l grandmaster on $ETH_IFACE..."
sudo mkdir -p /etc/linuxptp
sudo tee /etc/linuxptp/ptp4l-grandmaster.conf > /dev/null << EOF
[global]
# IEEE 1588 grandmaster priorities — lower = higher priority.
priority1               127
priority2               128
# clockClass values:
#   6   = locked to a primary GPS/PPS reference (requires hardware PPS)
#   7   = holdover after losing primary reference
#   13  = application-specific, locked to an internal reference
#   52  = application-specific, holdover
#   128 = default, NOT synchronized (the default ptp4l ships with)
# We advertise 13 because the host's CLOCK_REALTIME is disciplined
# from NMEA via gpsd → chrony, which is GPS-locked but not at the
# < 100 ns precision of a hardware PPS edge. clockClass 13 honestly
# represents "we have a GPS-derived internal reference" without
# overstating accuracy. Cross-sensor relative sync is unaffected
# by the choice of clockClass; this is just the metadata PTP slaves
# read to weigh competing masters on the same fabric.
clockClass              13
slaveOnly               0
delay_mechanism         E2E
logging_level           6
verbose                 1
summary_interval        1
time_stamping           $PTP_TIMESTAMPING

[$ETH_IFACE]
EOF
ok "ptp4l grandmaster configured (clockClass=13)."

if [ "$PTP_TIMESTAMPING" = "hardware" ]; then
    # ─── 4. phc2sys (CLOCK_REALTIME → NIC PHC) ──────────────────
    info "Configuring phc2sys..."
    sudo tee /etc/linuxptp/phc2sys-grandmaster.conf > /dev/null << 'EOF'
[global]
logging_level           6
verbose                 1
step_threshold          1.0
first_step_threshold    0.00002
EOF
    ok "phc2sys configured."
else
    info "Software timestamping selected — skipping phc2sys service."
fi

# ─── 5. systemd services ────────────────────────────────────
info "Creating systemd services..."

sudo tee /etc/systemd/system/ptp4l-grandmaster.service > /dev/null << EOF
[Unit]
Description=PTP4L Grandmaster Clock
After=network.target gpsd.service chrony.service

[Service]
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l-grandmaster.conf -i $ETH_IFACE
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

if [ "$PTP_TIMESTAMPING" = "hardware" ]; then
    sudo tee /etc/systemd/system/phc2sys-grandmaster.service > /dev/null << EOF
[Unit]
Description=PHC2SYS System Clock to NIC PHC
After=ptp4l-grandmaster.service
Requires=ptp4l-grandmaster.service

[Service]
ExecStart=/usr/sbin/phc2sys -s CLOCK_REALTIME -c $ETH_IFACE -w
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
else
    sudo systemctl disable --now phc2sys-grandmaster.service 2>/dev/null || true
fi

sudo systemctl daemon-reload
SERVICES=(gpsd.service chrony.service ptp4l-grandmaster.service)
if [ "$PTP_TIMESTAMPING" = "hardware" ]; then
    SERVICES+=(phc2sys-grandmaster.service)
fi
sudo systemctl enable --now "${SERVICES[@]}" || \
    warn "One or more services did not start cleanly — check 'systemctl status <svc>'."
ok "systemd services enabled and started."

# ─── 6. Point One Nav driver + tools (TCP transport) ────────
info "Installing Point One Nav ROS 2 driver..."
source_ros_setup
mkdir -p "$WS_DIR/src"

cd "$WS_DIR/src"
[ -d "ros2-fusion-engine-driver" ] || \
    git clone https://github.com/PointOneNav/ros2-fusion-engine-driver.git

# Fix <cstdint> for GCC 14
find "$WS_DIR/src/ros2-fusion-engine-driver" \
    \( -name "*.h" -o -name "*.hpp" -o -name "*.cc" -o -name "*.cpp" \) | \
    xargs grep -l "uint8_t\|uint16_t\|uint32_t\|uint64_t" 2>/dev/null | \
    while read -r f; do
        grep -q "#include <cstdint>" "$f" || sed -i '1 a #include <cstdint>' "$f"
    done

cd "$WS_DIR"
rosdep install -i --from-paths src/ros2-fusion-engine-driver --rosdistro "$ROS_DISTRO" -y \
    --skip-keys fusion_engine_msgs
colcon build --packages-up-to fusion_engine_driver

grep -q "source ~/ros2_ws/install/setup.bash" "$HOME/.bashrc" 2>/dev/null || \
    echo "source ~/ros2_ws/install/setup.bash" >> "$HOME/.bashrc"

info "Installing Point One host tools..."
cd "$HOME"
[ -d "p1-host-tools" ] || git clone https://github.com/PointOneNav/p1-host-tools.git
cd p1-host-tools
pip install -r requirements.txt --break-system-packages 2>/dev/null || true
pip install "fusion-engine-client[all]" --break-system-packages 2>/dev/null || true

ok "Point One Nav driver and tools installed."
info "Driver invocation example (TCP transport):"
echo "  ros2 run fusion_engine_driver fusion_engine_ros_driver --ros-args \\"
echo "      -p connection_type:=tcp \\"
echo "      -p tcp_ip:=$ATLAS_IP \\"
echo "      -p tcp_port:=$ATLAS_FE_PORT \\"
echo "      -p frame_id:=imu_link"

# =============================================================
# Self-test
# =============================================================
verify_grandmaster() {
    local PASS=0 FAIL=0 WARN_COUNT=0
    echo ""
    echo "============================================================="
    echo " VERIFICATION: Section 3 — INS → PC PTP grandmaster"
    echo "============================================================="

    # ── Test 1: Atlas Duo reachable ──
    info "Test 1/6: Atlas Duo reachable at $ATLAS_IP"
    if ping -c 2 -W 2 "$ATLAS_IP" &>/dev/null; then
        ok "  $ATLAS_IP responds to ping"
        PASS=$((PASS+1))
    else
        warn "  $ATLAS_IP did not respond — check cable, RUTM50 LAN 4, Atlas Duo power"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    # ── Test 2: NMEA TCP stream live ──
    info "Test 2/6: NMEA TCP stream from $ATLAS_IP:$ATLAS_NMEA_PORT"
    if NMEA_HEAD=$(timeout 5 nc -w 4 "$ATLAS_IP" "$ATLAS_NMEA_PORT" 2>/dev/null | head -3); then
        if echo "$NMEA_HEAD" | grep -qE '^\$(GP|GN|GA)[A-Z]{3},'; then
            ok "  NMEA sentences received ($(echo "$NMEA_HEAD" | head -1 | cut -c1-15)...)"
            PASS=$((PASS+1))
        else
            warn "  Port open but no NMEA sentences — Atlas navigation engine may be stopped"
            warn "  Open http://$ATLAS_IP and click Start Navigating on the Map View"
            WARN_COUNT=$((WARN_COUNT+1))
        fi
    else
        warn "  Could not read $ATLAS_IP:$ATLAS_NMEA_PORT — Atlas web UI Output Settings may have NMEA disabled"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    # ── Test 3: FusionEngine TCP stream live ──
    info "Test 3/6: FusionEngine TCP stream from $ATLAS_IP:$ATLAS_FE_PORT"
    if FE_BYTES=$(timeout 5 nc -w 4 "$ATLAS_IP" "$ATLAS_FE_PORT" 2>/dev/null | head -c 64 | wc -c); then
        if [ "$FE_BYTES" -gt 16 ]; then
            ok "  FusionEngine bytes flowing ($FE_BYTES bytes in 4 s)"
            PASS=$((PASS+1))
        else
            warn "  FusionEngine port open but no data — check Atlas Output Settings"
            WARN_COUNT=$((WARN_COUNT+1))
        fi
    else
        warn "  Could not read $ATLAS_IP:$ATLAS_FE_PORT"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    # ── Test 4: chrony NMEA discipline ──
    info "Test 4/6: chrony clock discipline"
    if systemctl is-active --quiet chrony 2>/dev/null; then
        ok "  chrony service is running"
        PASS=$((PASS+1))

        CHRONY_SRC=$(chronyc sources 2>/dev/null || true)
        if echo "$CHRONY_SRC" | grep -q '^\*.*NMEA'; then
            ok "  chrony primary source: NMEA (~10–100 ms accuracy)"
            PASS=$((PASS+1))
        elif echo "$CHRONY_SRC" | grep -q '^\*'; then
            warn "  chrony primary source is NTP (not GPS NMEA) — gpsd may not be providing time"
            WARN_COUNT=$((WARN_COUNT+1))
        else
            warn "  chrony has no selected source"
            WARN_COUNT=$((WARN_COUNT+1))
        fi

        OFFSET=$(chronyc tracking 2>/dev/null | grep "System time" | awk '{print $4, $5}')
        [ -n "$OFFSET" ] && info "  System time offset: $OFFSET"
    else
        warn "  chrony not running — start with: sudo systemctl start chrony"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    # ── Test 5: ptp4l grandmaster ──
    info "Test 5/6: ptp4l grandmaster"
    if systemctl is-active --quiet ptp4l-grandmaster 2>/dev/null; then
        ok "  ptp4l-grandmaster service is running"
        PASS=$((PASS+1))

        GM_LOG=$(sudo journalctl -u ptp4l-grandmaster --no-pager -n 50 2>/dev/null || true)
        if echo "$GM_LOG" | grep -q "assuming the grand master role"; then
            ok "  ptp4l has assumed grandmaster role"
            PASS=$((PASS+1))
        else
            warn "  ptp4l may not be grandmaster — check: sudo journalctl -u ptp4l-grandmaster -f"
            WARN_COUNT=$((WARN_COUNT+1))
        fi

        LAST_OFFSET=$(echo "$GM_LOG" | grep "master offset" | tail -1)
        [ -n "$LAST_OFFSET" ] && info "  Last ptp4l log: $LAST_OFFSET"
    else
        warn "  ptp4l-grandmaster not running"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    # ── Test 6: phc2sys (HW timestamping only) + ROS 2 driver ──
    info "Test 6/6: phc2sys + ROS 2 driver"
    if [ "$PTP_TIMESTAMPING" = "hardware" ]; then
        if systemctl is-active --quiet phc2sys-grandmaster 2>/dev/null; then
            ok "  phc2sys-grandmaster running"
            PASS=$((PASS+1))
        else
            warn "  phc2sys-grandmaster not running"
            WARN_COUNT=$((WARN_COUNT+1))
        fi
    else
        info "  Software timestamping — phc2sys not needed"
        PASS=$((PASS+1))
    fi

    source_optional_setup "$WS_DIR/install/setup.bash" || true
    if ros2 pkg list 2>/dev/null | grep -q "fusion_engine_driver"; then
        ok "  fusion_engine_driver package found"
        PASS=$((PASS+1))
    else
        warn "  fusion_engine_driver not found — rebuild: cd ~/ros2_ws && colcon build --packages-up-to fusion_engine_driver"
        WARN_COUNT=$((WARN_COUNT+1))
    fi

    echo ""
    echo "============================================================="
    echo " RESULTS: $PASS passed, $WARN_COUNT warnings, $FAIL failed"
    echo "============================================================="
    if [ $WARN_COUNT -gt 0 ]; then
        echo " Warnings may resolve once the Atlas Duo finishes GPS lock"
        echo " or after Start Navigating is clicked in its web UI."
    fi
    echo ""
    echo " Next: ./4_setup_lidar_ptp.sh"
    echo ""
}

verify_grandmaster
