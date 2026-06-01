#!/usr/bin/env bash
# =============================================================
# Section 1: Install Ubuntu RT Kernel + System Packages
#
# Installs everything the host needs that is NOT network or
# PTP configuration. After this script, the host has:
#
#   - apt prerequisites (linuxptp, chrony, gpsd, build tooling)
#   - real-time scheduling group + limits
#   - kernel sysctl tuning for high-bandwidth sensor UDP
#   - ROS 2 Jazzy desktop + dev tools
#   - GCC 14 <cstdint> patch helper (sourced by §3 and §4 scripts)
#
# Network configuration and the PTP grandmaster chain are
# handled by the next scripts in the sequence:
#
#   ./1_install_packages.sh          ← you are here
#   ./2_configure_host_network.sh    ← static IP + HW timestamping
#   ./3_setup_ins_to_pc_sync.sh      ← gpsd + chrony + ptp4l + p1
#   ./4_setup_lidar_ptp.sh           ← Robin W PTP enable + driver
#   ./5_setup_camera_ptp.sh          ← RouteCAM PTP enable + Aravis
#
# The PREEMPT_RT kernel itself is installed via Ubuntu Pro (free
# for personal use on up to 5 machines) and is OPTIONAL for
# data recording — see the README §1 callout. This script
# detects the running kernel and warns if RT is not active, but
# does not refuse to proceed.
#
# Usage:
#   chmod +x 1_install_packages.sh
#   ./1_install_packages.sh
# =============================================================

set -euo pipefail

# ─── Helpers ─────────────────────────────────────────────────
info()  { echo -e "\n\033[1;34m[INFO]\033[0m $*"; }
warn()  { echo -e "\n\033[1;33m[WARN]\033[0m $*"; }
ok()    { echo -e "\033[1;32m[ OK ]\033[0m $*"; }
fail()  { echo -e "\033[1;31m[FAIL]\033[0m $*"; exit 1; }

ROS_DISTRO="jazzy"

# =============================================================
info "Section 1: Install RT kernel + system packages"
echo ""

# ─── Detect RT kernel ───────────────────────────────────────
if uname -r | grep -qi "realtime\|preempt_rt"; then
    ok "Running PREEMPT_RT kernel: $(uname -r)"
else
    warn "Not running PREEMPT_RT kernel ($(uname -r))."
    warn "This is fine for RECORDING — generic kernel is sufficient."
    warn "If you need RT (hard real-time control), install via Ubuntu Pro:"
    warn "  sudo pro attach YOUR_TOKEN"
    warn "  sudo pro enable realtime-kernel"
    warn "  sudo reboot"
    warn "Continuing on the generic kernel..."
fi

# ─── 1. System prerequisites ────────────────────────────────
info "Installing apt prerequisites..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    build-essential cmake git curl wget \
    python3 python3-pip python3-venv python3-dev python-is-python3 \
    net-tools ethtool linuxptp chrony \
    gpsd gpsd-clients pps-tools \
    libyaml-cpp-dev \
    tcpdump
ok "Apt prerequisites installed."

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
ok "RT permissions configured (log out/in to activate group membership)."

# ─── 3. Kernel parameters for sensor I/O ────────────────────
info "Tuning kernel parameters for sensor UDP / LiDAR traffic..."
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

# ─── 4. ROS 2 Jazzy ─────────────────────────────────────────
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

# Enable tcpdump without sudo (used by recording stack)
sudo setcap cap_net_raw+ep "$(which tcpdump)" 2>/dev/null || true

# =============================================================
# Self-test
# =============================================================
verify_install() {
    local PASS=0 FAIL=0 WARN_COUNT=0
    echo ""
    echo "============================================================="
    echo " VERIFICATION: Section 1 — RT kernel + packages"
    echo "============================================================="

    info "Test 1/4: PREEMPT_RT kernel"
    if uname -r | grep -qi "realtime\|preempt_rt"; then
        ok "  PREEMPT_RT kernel active: $(uname -r)"
        ((PASS++))
    else
        warn "  Standard kernel: $(uname -r) — fine for recording"
        ((WARN_COUNT++))
    fi

    info "Test 2/4: RT scheduling permissions"
    if id -nG "$USER" | grep -qw realtime; then
        ok "  User '$USER' is in 'realtime' group"
        ((PASS++))
    else
        warn "  User '$USER' not in 'realtime' group yet — log out / log in to activate"
        ((WARN_COUNT++))
    fi
    if [ -f /etc/security/limits.d/99-realtime.conf ]; then
        ok "  RT limits file present"
        ((PASS++))
    else
        echo "  FAIL: /etc/security/limits.d/99-realtime.conf missing"
        ((FAIL++))
    fi

    info "Test 3/4: Kernel sysctl tuning"
    if [ "$(sysctl -n net.core.rmem_max 2>/dev/null)" = "33554432" ]; then
        ok "  net.core.rmem_max = 33554432"
        ((PASS++))
    else
        warn "  net.core.rmem_max not at expected value — reapply with: sudo sysctl --system"
        ((WARN_COUNT++))
    fi

    info "Test 4/4: ROS 2 $ROS_DISTRO"
    if command -v ros2 &>/dev/null || [ -d "/opt/ros/$ROS_DISTRO" ]; then
        ok "  ROS 2 $ROS_DISTRO present at /opt/ros/$ROS_DISTRO"
        ((PASS++))
    else
        warn "  ros2 not found — open a new shell or source /opt/ros/$ROS_DISTRO/setup.bash"
        ((WARN_COUNT++))
    fi

    echo ""
    echo "============================================================="
    echo " RESULTS: $PASS passed, $WARN_COUNT warnings, $FAIL failed"
    echo "============================================================="
    if [ $FAIL -gt 0 ]; then
        echo " Some tests FAILED — review the output above."
    elif [ $WARN_COUNT -gt 0 ]; then
        echo " All critical tests passed. Warnings should resolve after"
        echo " logging out/in or sourcing the new ROS shell rc."
    else
        echo " All tests passed."
    fi
    echo ""
    echo " Next: ./2_configure_host_network.sh"
    echo ""
}

verify_install
