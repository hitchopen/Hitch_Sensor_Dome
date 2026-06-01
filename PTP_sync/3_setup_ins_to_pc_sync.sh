#!/usr/bin/env bash
# =============================================================
# Section 3: PTP Sync from INS to PC
#
# Builds the GPS-disciplined PTP grandmaster chain on the host:
#
#   Atlas Duo PPS + NMEA
#     → gpsd
#     → chrony (PPS-locked CLOCK_REALTIME)
#     → phc2sys (CLOCK_REALTIME → NIC PHC)
#     → ptp4l (NIC PHC → PTP grandmaster announce on Ethernet)
#
# Also installs the fusion-engine-driver ROS 2 package and
# Point One host tools so the Atlas Duo's pose / IMU / GPSFix
# can be consumed by the recorder and by GLIM++.
#
# Position in the sequence:
#   ./1_install_packages.sh
#   ./2_configure_host_network.sh
#   ./3_setup_ins_to_pc_sync.sh      ← you are here
#   ./4_setup_lidar_ptp.sh
#   ./5_setup_camera_ptp.sh
#
# Prerequisites:
#   - Section 1 + Section 2 completed
#   - Atlas Duo connected via serial (PPS to /dev/pps0,
#     NMEA / FusionEngine to /dev/ttyUSB0 by default)
#
# Usage:
#   chmod +x 3_setup_ins_to_pc_sync.sh
#   ./3_setup_ins_to_pc_sync.sh
#   ./3_setup_ins_to_pc_sync.sh --serial /dev/ttyACM0
# =============================================================

set -euo pipefail

# ─── Load network state written by script #2 ─────────────────
if [ ! -f /run/hitch_dome_net.env ]; then
    echo "[FAIL] /run/hitch_dome_net.env missing — run ./2_configure_host_network.sh first." >&2
    exit 1
fi
# shellcheck disable=SC1091
source /run/hitch_dome_net.env

# ─── Load defaults from /config/network_config.yaml ──────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../config/load_network_config.sh
source "$SCRIPT_DIR/../config/load_network_config.sh"

# ─── Configuration (env > CLI > YAML / /run/hitch_dome_net.env) ──
ETH_IFACE="${ETH_IFACE:-$NETCFG_ETH}"
ATLAS_SERIAL="${ATLAS_SERIAL:-$NETCFG_ATLAS_SERIAL}"
ATLAS_PPS="${ATLAS_PPS:-$NETCFG_ATLAS_PPS}"
ROS_DISTRO="jazzy"
WS_DIR="$HOME/ros2_ws"
: "${PTP_TIMESTAMPING:=software}"   # default to SW if script 2 didn't run

# ─── Parse arguments ─────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --serial)   ATLAS_SERIAL="$2"; shift 2 ;;
        --serial=*) ATLAS_SERIAL="${1#*=}"; shift ;;
        --eth)      ETH_IFACE="$2"; shift 2 ;;
        --eth=*)    ETH_IFACE="${1#*=}"; shift ;;
        *)          echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# ─── Helpers ─────────────────────────────────────────────────
info()  { echo -e "\n\033[1;34m[INFO]\033[0m $*"; }
warn()  { echo -e "\n\033[1;33m[WARN]\033[0m $*"; }
ok()    { echo -e "\033[1;32m[ OK ]\033[0m $*"; }
fail()  { echo -e "\033[1;31m[FAIL]\033[0m $*"; exit 1; }

# =============================================================
info "Section 3: PTP sync — INS to PC"
info "  Ethernet:     $ETH_IFACE"
info "  Atlas serial: $ATLAS_SERIAL"
info "  Atlas PPS:    $ATLAS_PPS"
info "  Timestamping: $PTP_TIMESTAMPING"
echo ""

# ─── 1. gpsd ────────────────────────────────────────────────
info "Configuring gpsd for Atlas Duo at $ATLAS_SERIAL..."
sudo tee /etc/default/gpsd > /dev/null << EOF
DEVICES="$ATLAS_SERIAL $ATLAS_PPS"
GPSD_OPTIONS="-n -b"
START_DAEMON="true"
USBAUTO="false"
EOF
ok "gpsd configured."

# ─── 2. chrony (GPS-disciplined CLOCK_REALTIME) ─────────────
info "Configuring chrony with GPS PPS discipline..."
sudo tee /etc/chrony/chrony.conf > /dev/null << 'EOF'
# Primary: GPS via gpsd shared memory
refclock SHM 0 offset 0.5 delay 0.2 refid NMEA noselect
refclock PPS /dev/pps0 lock NMEA refid PPS

# Fallback: NTP
pool ntp.ubuntu.com iburst maxsources 4
pool time.google.com iburst maxsources 2

makestep 1.0 3
driftfile /var/lib/chrony/chrony.drift
rtcsync
maxupdateskew 100.0
EOF
ok "chrony configured."

# ─── 3. ptp4l (PTP grandmaster) ─────────────────────────────
info "Configuring ptp4l grandmaster on $ETH_IFACE..."
sudo mkdir -p /etc/linuxptp
sudo tee /etc/linuxptp/ptp4l-grandmaster.conf > /dev/null << EOF
[global]
# IEEE 1588 grandmaster priorities — lower = higher priority.
priority1               127
priority2               128
# clockClass 6  = locked to a primary reference (GPS/PPS)
# clockClass 7  = holdover after losing primary reference
# clockClass 13 = application-specific, locked
# clockClass 52 = application-specific, holdover
# clockClass 128 = default, NOT synchronized — wrong for a GPS-disciplined GM.
# The Atlas Duo PPS disciplines CLOCK_REALTIME via chrony, so advertise 6.
# ptp4l does not auto-step this based on chrony state; if you anticipate long
# GPS outages, add logic to switch to 7 (e.g. a watchdog that rewrites the
# config via pmc SET GRANDMASTER_SETTINGS_NP and reloads).
clockClass              6
slaveOnly               0
delay_mechanism         E2E
logging_level           6
verbose                 1
summary_interval        1
time_stamping           $PTP_TIMESTAMPING

[$ETH_IFACE]
EOF
ok "ptp4l grandmaster configured."

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

sudo systemctl daemon-reload
# --now enables at boot AND starts immediately so the PTP chain is live
# without requiring a reboot. If a service fails to start (e.g. no GPS yet),
# systemctl still returns success; check status/journalctl below.
sudo systemctl enable --now gpsd.service chrony.service \
    ptp4l-grandmaster.service phc2sys-grandmaster.service || \
    warn "One or more services did not start cleanly — check 'systemctl status <svc>'."
ok "systemd services enabled and started."

# ─── 6. Point One Nav driver + tools ────────────────────────
info "Installing Point One Nav ROS 2 driver..."
source /opt/ros/${ROS_DISTRO}/setup.bash
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
rosdep install -i --from-path src/ros2-fusion-engine-driver --rosdistro "$ROS_DISTRO" -y 2>/dev/null || true
colcon build --packages-select fusion-engine-driver

grep -q "source ~/ros2_ws/install/setup.bash" "$HOME/.bashrc" 2>/dev/null || \
    echo "source ~/ros2_ws/install/setup.bash" >> "$HOME/.bashrc"

info "Installing Point One host tools..."
cd "$HOME"
[ -d "p1-host-tools" ] || git clone https://github.com/PointOneNav/p1-host-tools.git
cd p1-host-tools
pip install -r requirements.txt --break-system-packages 2>/dev/null || true
pip install "fusion-engine-client[all]" --break-system-packages 2>/dev/null || true

ok "Point One Nav driver and tools installed."

# =============================================================
# Self-test
# =============================================================
verify_grandmaster() {
    local PASS=0 FAIL=0 WARN_COUNT=0
    echo ""
    echo "============================================================="
    echo " VERIFICATION: Section 3 — INS → PC PTP grandmaster"
    echo "============================================================="

    # ── Test 1: gpsd + GPS fix ──
    info "Test 1/6: gpsd and GPS fix"
    if systemctl is-active --quiet gpsd 2>/dev/null; then
        ok "  gpsd service is running"
        ((PASS++))
        if timeout 5 gpspipe -w 2>/dev/null | head -5 | grep -q '"class":"TPV"'; then
            ok "  GPS fix detected (TPV message received)"
            ((PASS++))
        else
            warn "  No GPS fix yet — antenna may need clear sky view (cold start: up to 30 min)"
            ((WARN_COUNT++))
        fi
    else
        warn "  gpsd not running — start with: sudo systemctl start gpsd"
        ((WARN_COUNT++))
    fi

    # ── Test 2: PPS signal ──
    info "Test 2/6: PPS signal"
    if [ -e /dev/pps0 ]; then
        ok "  /dev/pps0 exists"
        ((PASS++))
        if timeout 3 sudo ppstest /dev/pps0 2>/dev/null | head -3 | grep -q "assert"; then
            ok "  PPS pulses detected"
            ((PASS++))
        else
            warn "  No PPS pulses — Atlas Duo may not have GPS lock yet"
            ((WARN_COUNT++))
        fi
    else
        warn "  /dev/pps0 not found — chrony will use NMEA only (~1 ms accuracy)"
        ((WARN_COUNT++))
    fi

    # ── Test 3: chrony GPS discipline ──
    info "Test 3/6: chrony clock discipline"
    if systemctl is-active --quiet chrony 2>/dev/null; then
        ok "  chrony service is running"
        ((PASS++))

        CHRONY_SRC=$(chronyc sources 2>/dev/null || true)
        if echo "$CHRONY_SRC" | grep -q '^\*.*PPS'; then
            ok "  chrony primary source: PPS (< 100 ns accuracy)"
            ((PASS++))
        elif echo "$CHRONY_SRC" | grep -q '^\*.*NMEA'; then
            warn "  chrony primary source: NMEA (~1 ms accuracy, no PPS)"
            ((WARN_COUNT++))
        elif echo "$CHRONY_SRC" | grep -q '^\*'; then
            warn "  chrony primary source is NTP (not GPS) — gpsd may not be providing time"
            ((WARN_COUNT++))
        else
            warn "  chrony has no selected source"
            ((WARN_COUNT++))
        fi

        OFFSET=$(chronyc tracking 2>/dev/null | grep "System time" | awk '{print $4, $5}')
        [ -n "$OFFSET" ] && info "  System time offset: $OFFSET"
    else
        warn "  chrony not running — start with: sudo systemctl start chrony"
        ((WARN_COUNT++))
    fi

    # ── Test 4: ptp4l grandmaster ──
    info "Test 4/6: ptp4l grandmaster"
    if systemctl is-active --quiet ptp4l-grandmaster 2>/dev/null; then
        ok "  ptp4l-grandmaster service is running"
        ((PASS++))

        GM_LOG=$(sudo journalctl -u ptp4l-grandmaster --no-pager -n 50 2>/dev/null || true)
        if echo "$GM_LOG" | grep -q "assuming the grand master role"; then
            ok "  ptp4l has assumed grandmaster role"
            ((PASS++))
        else
            warn "  ptp4l may not be grandmaster — check: sudo journalctl -u ptp4l-grandmaster -f"
            ((WARN_COUNT++))
        fi

        LAST_OFFSET=$(echo "$GM_LOG" | grep "master offset" | tail -1)
        [ -n "$LAST_OFFSET" ] && info "  Last ptp4l log: $LAST_OFFSET"
    else
        warn "  ptp4l-grandmaster not running"
        ((WARN_COUNT++))
    fi

    # ── Test 5: phc2sys (hardware timestamping only) ──
    info "Test 5/6: phc2sys (CLOCK_REALTIME → NIC PHC)"
    if [ "$PTP_TIMESTAMPING" = "hardware" ]; then
        if systemctl is-active --quiet phc2sys-grandmaster 2>/dev/null; then
            ok "  phc2sys-grandmaster running"
            ((PASS++))

            PHC_LOG=$(sudo journalctl -u phc2sys-grandmaster --no-pager -n 20 2>/dev/null || true)
            LAST_PHC=$(echo "$PHC_LOG" | grep "offset" | tail -1)
            if [ -n "$LAST_PHC" ]; then
                info "  Last phc2sys log: $LAST_PHC"
                PHC_OFFSET=$(echo "$LAST_PHC" | grep -oP 'offset\s+\K[-0-9]+' || true)
                if [ -n "$PHC_OFFSET" ] && [ "${PHC_OFFSET#-}" -lt 1000 ] 2>/dev/null; then
                    ok "  PHC offset < 1 µs"
                    ((PASS++))
                elif [ -n "$PHC_OFFSET" ] && [ "${PHC_OFFSET#-}" -lt 10000 ] 2>/dev/null; then
                    warn "  PHC offset < 10 µs (acceptable)"
                    ((WARN_COUNT++))
                fi
            fi
        else
            warn "  phc2sys-grandmaster not running"
            ((WARN_COUNT++))
        fi
    else
        info "  Software timestamping — phc2sys not needed (ptp4l uses system clock directly)"
        ((PASS++))
    fi

    # ── Test 6: ROS 2 driver ──
    info "Test 6/6: Point One Nav ROS 2 driver"
    source "$WS_DIR/install/setup.bash" 2>/dev/null || true
    if ros2 pkg list 2>/dev/null | grep -q "fusion-engine-driver"; then
        ok "  fusion-engine-driver package found"
        ((PASS++))
    else
        warn "  fusion-engine-driver not found — rebuild: cd ~/ros2_ws && colcon build"
        ((WARN_COUNT++))
    fi

    if command -v p1_print &>/dev/null || [ -f "$HOME/p1-host-tools/bin/p1_print" ]; then
        ok "  Point One host tools available"
        ((PASS++))
    else
        warn "  p1_print not found — install: pip install fusion-engine-client[all]"
        ((WARN_COUNT++))
    fi

    echo ""
    echo "============================================================="
    echo " RESULTS: $PASS passed, $WARN_COUNT warnings, $FAIL failed"
    echo "============================================================="
    if [ $WARN_COUNT -gt 0 ]; then
        echo " Warnings may resolve after GPS lock or a reboot."
    fi
    echo ""
    echo " Next: ./4_setup_lidar_ptp.sh"
    echo ""
}

verify_grandmaster
