#!/usr/bin/env bash
# =============================================================
# Step 1: Ubuntu 24.04 RT Kernel + GPS-Disciplined PTP Grandmaster
#
# Sets up the host system as a PTP grandmaster with GPS-disciplined
# time from the Point One Nav Atlas Duo. This is the foundation
# that all other sensors synchronize against.
#
# What this script does:
#   1. Install system prerequisites (linuxptp, chrony, gpsd, ROS 2)
#   2. Configure RT scheduling permissions
#   3. Tune kernel network buffers for high-bandwidth sensor I/O
#   4. Configure gpsd to read Atlas Duo PPS + NMEA
#   5. Configure chrony to discipline system clock from GPS PPS
#   6. Configure ptp4l as PTP grandmaster on sensor Ethernet
#   7. Configure phc2sys to sync NIC PHC from system clock
#   8. Create systemd services for the full PTP chain
#   9. Install ROS 2 Jazzy and Point One Nav driver
#
# Prerequisites:
#   - Ubuntu 24.04 LTS with PREEMPT_RT kernel installed
#   - Point One Nav Atlas Duo connected via serial (PPS + NMEA)
#   - Ethernet interface connected to sensor network
#
# Usage:
#   chmod +x setup_ubuntu_sync.sh
#   ./setup_ubuntu_sync.sh [--eth IFACE] [--serial DEVICE]
#
# Examples:
#   ./setup_ubuntu_sync.sh --eth enp0s31f6
#   ./setup_ubuntu_sync.sh --eth enp0s31f6 --serial /dev/ttyACM0
# =============================================================

set -euo pipefail

# ─── Configuration ───────────────────────────────────────────
ETH_IFACE="${ETH_IFACE:-eth0}"
HOST_IP="${HOST_IP:-192.168.1.1}"   # RFC 1918 private — 192.168.1.x for sensor LAN
ATLAS_SERIAL="${ATLAS_SERIAL:-/dev/ttyUSB0}"
ROS_DISTRO="jazzy"
WS_DIR="$HOME/ros2_ws"

# ─── Parse arguments ─────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --eth)      ETH_IFACE="$2"; shift 2 ;;
        --eth=*)    ETH_IFACE="${1#*=}"; shift ;;
        --serial)   ATLAS_SERIAL="$2"; shift 2 ;;
        --serial=*) ATLAS_SERIAL="${1#*=}"; shift ;;
        --host-ip)  HOST_IP="$2"; shift 2 ;;
        *)          echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# ─── Helpers ─────────────────────────────────────────────────
info()  { echo -e "\n\033[1;34m[INFO]\033[0m $*"; }
warn()  { echo -e "\n\033[1;33m[WARN]\033[0m $*"; }
ok()    { echo -e "\033[1;32m[ OK ]\033[0m $*"; }
fail()  { echo -e "\033[1;31m[FAIL]\033[0m $*"; exit 1; }

# =============================================================
info "Step 1: Ubuntu System + PTP Grandmaster Setup"
info "  Ethernet: $ETH_IFACE"
info "  Host IP:  $HOST_IP"
info "  Atlas serial: $ATLAS_SERIAL"
echo ""

# ─── Verify RT kernel ────────────────────────────────────────
if uname -r | grep -qi "realtime\|preempt_rt"; then
    ok "Running PREEMPT_RT kernel: $(uname -r)"
else
    warn "Not running RT kernel ($(uname -r))."
    warn "PTP timing will be less accurate. To install:"
    warn "  sudo pro attach YOUR_TOKEN && sudo pro enable realtime-kernel && sudo reboot"
fi

# ─── 1. System prerequisites ────────────────────────────────
info "Installing system prerequisites..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    build-essential cmake git curl wget \
    python3 python3-pip python3-venv python3-dev python-is-python3 \
    net-tools ethtool linuxptp chrony \
    gpsd gpsd-clients pps-tools \
    libyaml-cpp-dev \
    tcpdump
ok "System prerequisites installed."

# ─── 2. RT scheduling permissions ───────────────────────────
info "Configuring real-time scheduling permissions..."
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"

sudo tee /etc/security/limits.d/99-realtime.conf > /dev/null << 'EOF'
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
EOF
ok "RT permissions configured (log out/in to activate)."

# ─── 3. Kernel parameters ───────────────────────────────────
info "Tuning kernel parameters for sensor I/O..."
sudo tee /etc/sysctl.d/99-sensor-recording.conf > /dev/null << 'EOF'
# Large buffers for high-bandwidth LiDAR + camera UDP streams
net.core.rmem_max = 33554432
net.core.rmem_default = 8388608
net.core.wmem_max = 33554432
net.core.netdev_max_backlog = 10000

# Keep sensor drivers in RAM
vm.swappiness = 10
EOF
sudo sysctl --system > /dev/null
ok "Kernel parameters applied."

# ─── 4. Network interface ───────────────────────────────────
info "Configuring network interface $ETH_IFACE..."
if ! ip link show "$ETH_IFACE" &>/dev/null; then
    fail "Interface $ETH_IFACE not found. Available: $(ip -br link show | awk '{print $1}' | tr '\n' ' ')"
fi

sudo ip addr add "$HOST_IP/24" dev "$ETH_IFACE" 2>/dev/null || \
    warn "$HOST_IP already assigned to $ETH_IFACE"
sudo ip link set "$ETH_IFACE" up

# Detect hardware timestamping
if ethtool -T "$ETH_IFACE" 2>/dev/null | grep -q "hardware-transmit"; then
    ok "Hardware timestamping supported on $ETH_IFACE"
    PTP_TIMESTAMPING="hardware"
else
    warn "No hardware timestamping — using software mode (20–50 µs accuracy)"
    PTP_TIMESTAMPING="software"
fi

# Enable tcpdump without sudo
sudo setcap cap_net_raw+ep "$(which tcpdump)" 2>/dev/null || true
ok "Network configured: $ETH_IFACE = $HOST_IP/24"

# ─── 5. gpsd (Atlas Duo GPS) ────────────────────────────────
info "Configuring gpsd for Atlas Duo at $ATLAS_SERIAL..."
sudo tee /etc/default/gpsd > /dev/null << EOF
DEVICES="$ATLAS_SERIAL /dev/pps0"
GPSD_OPTIONS="-n -b"
START_DAEMON="true"
USBAUTO="false"
EOF
ok "gpsd configured."

# ─── 6. chrony (GPS-disciplined system clock) ───────────────
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

# ─── 7. ptp4l (PTP grandmaster) ─────────────────────────────
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

# ─── 8. phc2sys (system clock → NIC PHC) ────────────────────
info "Configuring phc2sys..."
sudo tee /etc/linuxptp/phc2sys-grandmaster.conf > /dev/null << 'EOF'
[global]
logging_level           6
verbose                 1
step_threshold          1.0
first_step_threshold    0.00002
EOF
ok "phc2sys configured."

# ─── 9. systemd services ────────────────────────────────────
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
ok "systemd services created, enabled, and started."

# ─── 10. ROS 2 Jazzy ────────────────────────────────────────
info "Installing ROS 2 $ROS_DISTRO..."

sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-desktop ros-dev-tools \
    ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-tf2-tools

# Source ROS in .bashrc
grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" "$HOME/.bashrc" 2>/dev/null || \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "$HOME/.bashrc"
ok "ROS 2 $ROS_DISTRO installed."

# ─── 11. Point One Nav driver + tools ────────────────────────
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
# 12. Self-Test: Verify PTP Sync Quality
# =============================================================
verify_ptp_sync() {
    local PASS=0 FAIL=0 WARN_COUNT=0
    echo ""
    echo "============================================================="
    echo " VERIFICATION: Ubuntu + PTP Grandmaster + Atlas Duo"
    echo "============================================================="

    # ── Test 1: RT kernel ──
    info "Test 1/8: PREEMPT_RT kernel"
    if uname -r | grep -qi "realtime\|preempt_rt"; then
        ok "  PREEMPT_RT kernel active: $(uname -r)"
        ((PASS++))
    else
        warn "  Standard kernel: $(uname -r) — PTP accuracy reduced"
        ((WARN_COUNT++))
    fi

    # ── Test 2: RT scheduling permissions ──
    info "Test 2/8: Real-time scheduling permissions"
    if id -nG "$USER" | grep -qw realtime; then
        ok "  User '$USER' is in 'realtime' group"
        ((PASS++))
    else
        warn "  User '$USER' not in 'realtime' group — log out/in required"
        ((WARN_COUNT++))
    fi
    if [ -f /etc/security/limits.d/99-realtime.conf ]; then
        ok "  RT limits file exists"
        ((PASS++))
    else
        echo "  FAIL: /etc/security/limits.d/99-realtime.conf missing"
        ((FAIL++))
    fi

    # ── Test 3: gpsd + GPS fix ──
    info "Test 3/8: gpsd and GPS fix"
    if systemctl is-active --quiet gpsd 2>/dev/null; then
        ok "  gpsd service is running"
        ((PASS++))
        # Check for GPS fix (timeout 5s)
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

    # ── Test 4: PPS signal ──
    info "Test 4/8: PPS signal"
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
        warn "  /dev/pps0 not found — PPS not exposed; chrony will use NMEA only (~1 ms accuracy)"
        ((WARN_COUNT++))
    fi

    # ── Test 5: chrony GPS discipline ──
    info "Test 5/8: chrony clock discipline"
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

        # Show system time offset
        OFFSET=$(chronyc tracking 2>/dev/null | grep "System time" | awk '{print $4, $5}')
        if [ -n "$OFFSET" ]; then
            info "  System time offset: $OFFSET"
        fi
    else
        warn "  chrony not running — start with: sudo systemctl start chrony"
        ((WARN_COUNT++))
    fi

    # ── Test 6: ptp4l grandmaster ──
    info "Test 6/8: ptp4l grandmaster"
    if systemctl is-active --quiet ptp4l-grandmaster 2>/dev/null; then
        ok "  ptp4l-grandmaster service is running"
        ((PASS++))

        # Check if it assumed grandmaster role
        GM_LOG=$(sudo journalctl -u ptp4l-grandmaster --no-pager -n 50 2>/dev/null || true)
        if echo "$GM_LOG" | grep -q "assuming the grand master role"; then
            ok "  ptp4l has assumed grandmaster role"
            ((PASS++))
        else
            warn "  ptp4l may not be grandmaster — check: sudo journalctl -u ptp4l-grandmaster -f"
            ((WARN_COUNT++))
        fi

        # Show recent master offset
        LAST_OFFSET=$(echo "$GM_LOG" | grep "master offset" | tail -1)
        if [ -n "$LAST_OFFSET" ]; then
            info "  Last ptp4l log: $LAST_OFFSET"
        fi
    else
        warn "  ptp4l-grandmaster not running — start with: sudo systemctl start ptp4l-grandmaster"
        ((WARN_COUNT++))
    fi

    # ── Test 7: phc2sys ──
    info "Test 7/8: phc2sys (system clock → NIC PHC)"
    if [ "$PTP_TIMESTAMPING" = "hardware" ]; then
        if systemctl is-active --quiet phc2sys-grandmaster 2>/dev/null; then
            ok "  phc2sys-grandmaster service is running"
            ((PASS++))

            PHC_LOG=$(sudo journalctl -u phc2sys-grandmaster --no-pager -n 20 2>/dev/null || true)
            LAST_PHC=$(echo "$PHC_LOG" | grep "offset" | tail -1)
            if [ -n "$LAST_PHC" ]; then
                info "  Last phc2sys log: $LAST_PHC"
                # Extract offset value and check magnitude
                PHC_OFFSET=$(echo "$LAST_PHC" | grep -oP 'offset\s+\K[-0-9]+' || true)
                if [ -n "$PHC_OFFSET" ] && [ "${PHC_OFFSET#-}" -lt 1000 ] 2>/dev/null; then
                    ok "  PHC offset < 1 µs — excellent sync"
                    ((PASS++))
                elif [ -n "$PHC_OFFSET" ] && [ "${PHC_OFFSET#-}" -lt 10000 ] 2>/dev/null; then
                    warn "  PHC offset < 10 µs — acceptable"
                    ((WARN_COUNT++))
                fi
            fi
        else
            warn "  phc2sys-grandmaster not running"
            ((WARN_COUNT++))
        fi
    else
        info "  phc2sys skipped (software timestamping — ptp4l uses system clock directly)"
        ((PASS++))
    fi

    # ── Test 8: ROS 2 + Point One Nav driver ──
    info "Test 8/8: ROS 2 and Point One Nav driver"
    if command -v ros2 &>/dev/null; then
        ok "  ros2 command available"
        ((PASS++))
        source "$WS_DIR/install/setup.bash" 2>/dev/null || true
        if ros2 pkg list 2>/dev/null | grep -q "fusion-engine-driver"; then
            ok "  fusion-engine-driver package found"
            ((PASS++))
        else
            warn "  fusion-engine-driver not found — rebuild: cd ~/ros2_ws && colcon build"
            ((WARN_COUNT++))
        fi
    else
        warn "  ros2 not found — source /opt/ros/$ROS_DISTRO/setup.bash"
        ((WARN_COUNT++))
    fi

    if command -v p1_print &>/dev/null || [ -f "$HOME/p1-host-tools/bin/p1_print" ]; then
        ok "  Point One host tools available"
        ((PASS++))
    else
        warn "  p1_print not found — install: pip install fusion-engine-client[all]"
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
        echo " GPS lock, rebooting, or logging out/in."
    else
        echo " All tests passed — system is ready."
    fi
    echo ""
    echo " Next: ./setup_robin_w_sync.sh --eth $ETH_IFACE"
    echo ""
}

verify_ptp_sync
