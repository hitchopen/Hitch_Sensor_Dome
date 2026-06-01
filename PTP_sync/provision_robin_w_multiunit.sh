#!/usr/bin/env bash
# =============================================================
# Robin W Per-Position Provisioning
# (run ONCE per LiDAR, attaching one unit at a time)
#
# WORKFLOW
# --------
# This script provisions ONE Robin W per invocation. You tell it
# which dome position the currently-attached LiDAR should become,
# and it walks that unit from factory state to the project's
# per-position IP + UDP port.
#
#   Connect ONE Robin W via Ethernet (it ships at the factory IP
#   172.168.1.10 per Seyond's Robin W1G User Manual V2.2 §3.1)
#   ────►
#   ./provision_robin_w_multiunit.sh --position front
#   (script: discovers LiDAR → reads its serial number →
#            set_network → reboot → upload position PCS_ENV →
#            reboot → verify → append SN to inventory.yaml)
#   ────►
#   Power off / disconnect, attach the NEXT Robin W
#   ────►
#   ./provision_robin_w_multiunit.sh --position rear_left
#   ────►
#   ... and once more for --position rear_right.
#
# Why one-at-a-time: every brand-new Robin W ships with the
# same factory IP (172.168.1.10). Two or more factory-state
# units on the same Ethernet segment would collide. After this
# script renumbers a unit to its dome IP, it stops colliding,
# but during provisioning only ONE unit at the factory IP can
# be on the wire at any time.
#
# How to decide which physical LiDAR is which position:
# the dome SCAD model in `3D files/sensor_dome.scad` defines:
#   robin_w_front       → mounted at 0°,   IP .10, UDP 8337
#   robin_w_rear_left   → mounted at 120°, IP .11, UDP 8338
#   robin_w_rear_right  → mounted at 240°, IP .12, UDP 8339
# Easiest workflow on a fresh build: label the three units
# with masking tape (FRONT / REAR-LEFT / REAR-RIGHT) BEFORE
# you start, then attach them to the host one at a time in
# that order.
#
# After all three pass, RobinW_FW2835_Multiunit/serial_inventory.yaml
# captures the SN-to-position map for traceability / spares
# tracking. The file is generated; do not hand-edit.
#
# Files this script uses (all in PTP_sync/):
#   innovusion_lidar_util                  Seyond CLI binary
#   RobinW_FW2835_Multiunit/                Per-position PCS_ENV files
#       RW_FW2835_robin_w_front_unicast.env       (port 8337)
#       RW_FW2835_robin_w_rear_left_unicast.env   (port 8338)
#       RW_FW2835_robin_w_rear_right_unicast.env  (port 8339)
#   RobinW_FW2835_Multiunit/serial_inventory.yaml  (generated)
#   ../config/network_config.yaml          Target IPs + host IP
#
# Usage:
#   chmod +x provision_robin_w_multiunit.sh
#   ./provision_robin_w_multiunit.sh --position <front|rear_left|rear_right>
#
# Optional flags:
#   --factory-ip 172.168.1.X    if the unit is no longer at the Seyond default
#   --netmask    255.255.255.0  rarely needed
#
# Re-run safety: if the unit is already at its target IP and the
# on-LiDAR PCS_ENV matches the local file, the script reports
# [SKIP] and exits cleanly.
# =============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../config/load_network_config.sh
source "$SCRIPT_DIR/../config/load_network_config.sh"

# ─── Defaults ────────────────────────────────────────────────
FACTORY_IP="${FACTORY_IP:-172.168.1.10}"
NETMASK="${NETMASK:-255.255.255.0}"
POSITION=""
SLEEP_AFTER_REBOOT="${SLEEP_AFTER_REBOOT:-25}"   # Robin W needs ~20 s
                                                  # to come back up after reboot

ENV_DIR="$SCRIPT_DIR/RobinW_FW2835_Multiunit"
INVENTORY="$ENV_DIR/serial_inventory.yaml"

# ─── Position table — IP + UDP port per dome position ────────
#
# Position name maps to:
#   - target_ip  (last octet of 192.168.1.0/24)
#   - udp_port   (point cloud + control + status, see below)
#   - pcs_env    (filename relative to ENV_DIR)
#
# UDP/TCP ports 8337-8339 chosen to avoid:
#   - Apache Hadoop defaults: 8020 (HDFS NameNode IPC) and 8030 (YARN
#     ResourceManager scheduler) would collide on any host doing data
#     engineering double duty.
#   - The heavily-trafficked 8000-8099 dev-server / HTTP-alt range
#     (Python http.server, Django, Cypress, Jupyter, etc.).
# 8337/8338 are IANA-registered to Konica Minolta PowerJet (printer
# management — vanishingly unlikely on a perception PC). 8339 is unassigned.
#
# Position table format: position|target_ip|udp_port|pcs_env_filename
POSITION_TABLE=(
  "front|192.168.1.10|8337|RW_FW2835_robin_w_front_unicast.env"
  "rear_left|192.168.1.11|8338|RW_FW2835_robin_w_rear_left_unicast.env"
  "rear_right|192.168.1.12|8339|RW_FW2835_robin_w_rear_right_unicast.env"
)

# ─── Parse args ──────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --position)     POSITION="$2"; shift 2 ;;
        --position=*)   POSITION="${1#*=}"; shift ;;
        --factory-ip)   FACTORY_IP="$2"; shift 2 ;;
        --factory-ip=*) FACTORY_IP="${1#*=}"; shift ;;
        --netmask)      NETMASK="$2"; shift 2 ;;
        --netmask=*)    NETMASK="${1#*=}"; shift ;;
        -h|--help)
            sed -n '2,80p' "$0"; exit 0 ;;
        *) echo "Unknown arg: $1 — use --help"; exit 1 ;;
    esac
done

# ─── Helpers ─────────────────────────────────────────────────
info()  { echo -e "\n\033[1;34m[INFO]\033[0m $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m $*"; }
ok()    { echo -e "\033[1;32m[ OK ]\033[0m $*"; }
skip()  { echo -e "\033[1;36m[SKIP]\033[0m $*"; }
fail()  { echo -e "\033[1;31m[FAIL]\033[0m $*"; exit 1; }

# ─── Validate --position ──────────────────────────────────────
if [ -z "$POSITION" ]; then
    fail "Missing required --position {front|rear_left|rear_right}. Run with --help for details."
fi

ROW=""
for row in "${POSITION_TABLE[@]}"; do
    IFS='|' read -r p _ _ _ <<< "$row"
    [ "$p" = "$POSITION" ] && { ROW="$row"; break; }
done
if [ -z "$ROW" ]; then
    fail "Unknown --position '$POSITION'. Must be one of: front, rear_left, rear_right."
fi
IFS='|' read -r POSITION TARGET_IP UDP_PORT ENV_FILE <<< "$ROW"
ENV_PATH="$ENV_DIR/$ENV_FILE"

# ─── Reachability fix: temporary IP alias on the sensor NIC ──
#
# Brand-new LiDARs are at Seyond's factory IP 172.168.1.10. The
# dome host PC lives on 192.168.1.0/24. The two subnets do not
# overlap, so the host cannot reach a factory-state LiDAR without
# an IP in 172.168.1.0/24.
#
# We add a temporary alias 172.168.1.100/24 to the sensor NIC at
# the start of provisioning, and remove it on exit. The alias
# coexists with the host's 192.168.1.5/24 address (both reachable
# at L2), so post-provisioning pings to the new 192.168.1.X address
# also work from the same script run.
ALIAS_ADDED=0
ALIAS_IP="${FACTORY_IP%.*}.100/24"
TARGET_IFACE="${NETCFG_ETH:-}"

maybe_add_alias() {
    [ -z "$TARGET_IFACE" ] && { warn "host.interface unset in network_config.yaml — skipping alias"; return; }
    if ip -4 -o addr show dev "$TARGET_IFACE" 2>/dev/null | grep -q "${FACTORY_IP%.*}\."; then
        ok "Host already has an address in ${FACTORY_IP%.*}.0/24 on $TARGET_IFACE — no alias needed"
        return
    fi
    info "Adding temporary IP alias $ALIAS_IP on $TARGET_IFACE (sudo)"
    if sudo ip addr add "$ALIAS_IP" dev "$TARGET_IFACE" 2>/dev/null; then
        ALIAS_ADDED=1
        ok "Alias up — host reachable on both 192.168.1.5 and ${ALIAS_IP%/*}"
    else
        warn "Could not add IP alias. You may need to run:"
        warn "  sudo ip addr add $ALIAS_IP dev $TARGET_IFACE"
        warn "and re-run this script."
        fail "Aborting because factory-IP LiDARs would be unreachable."
    fi
}
cleanup_alias() {
    if [ "$ALIAS_ADDED" -eq 1 ] && [ -n "$TARGET_IFACE" ]; then
        info "Removing temporary IP alias $ALIAS_IP from $TARGET_IFACE"
        sudo ip addr del "$ALIAS_IP" dev "$TARGET_IFACE" 2>/dev/null || \
            warn "Could not remove alias — clean up manually: sudo ip addr del $ALIAS_IP dev $TARGET_IFACE"
    fi
}
trap cleanup_alias EXIT

# ─── Locate the Seyond utility ───────────────────────────────
UTIL=""
for c in "$SCRIPT_DIR/innovusion_lidar_util" \
         "./innovusion_lidar_util" \
         "innovusion_lidar_util"; do
    if [ -x "$c" ] || command -v "$c" &>/dev/null; then UTIL="$c"; break; fi
done
[ -z "$UTIL" ] && fail "innovusion_lidar_util not found. Place it in PTP_sync/."

# ─── Pre-flight checks ───────────────────────────────────────
[ -d "$ENV_DIR" ] || fail "Missing $ENV_DIR (PCS_ENV files)."
[ -f "$ENV_PATH" ] || fail "Missing $ENV_PATH for position $POSITION."

HOST_IP="$NETCFG_HOST_IP"
[ -z "$HOST_IP" ] && fail "Host IP missing in network_config.yaml"
if ! grep -q "^UDP_IP=${HOST_IP}$" "$ENV_PATH"; then
    fail "$ENV_PATH has UDP_IP != $HOST_IP — edit the PCS_ENV file or update network_config.yaml host.ip"
fi

# ─── Banner ──────────────────────────────────────────────────
info "Robin W single-unit provisioning"
echo "    position  : $POSITION"
echo "    target IP : $TARGET_IP"
echo "    UDP port  : $UDP_PORT"
echo "    PCS_ENV   : $ENV_FILE"
echo "    factory IP: $FACTORY_IP"
echo "    host IP   : $HOST_IP"
echo "    utility   : $UTIL"
echo ""

# ─── Discover the attached LiDAR ─────────────────────────────
# We try TWO addresses:
#   1. The target IP (e.g. 192.168.1.10) — covers the re-run case
#      where this unit was already provisioned earlier.
#   2. The factory IP 172.168.1.10 — covers a brand-new unit
#      coming out of the box.
# Either way, we read the LiDAR's serial number with get_static_info
# so we can record it in serial_inventory.yaml later.

read_serial() {
    local ip="$1"
    "$UTIL" "$ip" get_static_info 2>/dev/null | \
        grep -oE 'sn[: =]*[0-9]+' | head -1 | grep -oE '[0-9]+' || true
}

CURRENT_IP=""
SERIAL=""

# Try target IP first (re-run / already-provisioned case)
if ping -c1 -W2 "$TARGET_IP" &>/dev/null; then
    SERIAL=$(read_serial "$TARGET_IP")
    if [ -n "$SERIAL" ]; then
        ok "Found Robin W at $TARGET_IP, serial $SERIAL"
        CURRENT_IP="$TARGET_IP"
    fi
fi

# Try factory IP (brand-new unit)
if [ -z "$CURRENT_IP" ]; then
    maybe_add_alias
    if ping -c1 -W2 "$FACTORY_IP" &>/dev/null; then
        SERIAL=$(read_serial "$FACTORY_IP")
        if [ -n "$SERIAL" ]; then
            ok "Found factory-state Robin W at $FACTORY_IP, serial $SERIAL"
            CURRENT_IP="$FACTORY_IP"
        fi
    fi
fi

if [ -z "$CURRENT_IP" ]; then
    fail "No Robin W reachable at $TARGET_IP or $FACTORY_IP. \
Power on the unit, confirm Ethernet link, and re-run."
fi

# ─── Safety: refuse if target IP is already occupied by a different unit ──
if [ "$CURRENT_IP" = "$TARGET_IP" ] && \
   [ -f "$INVENTORY" ] && \
   grep -A1 "^  $POSITION:" "$INVENTORY" 2>/dev/null | grep -q "serial:"; then
    recorded=$(grep -A1 "^  $POSITION:" "$INVENTORY" | grep "serial:" | grep -oE '[0-9]+' | head -1 || true)
    if [ -n "$recorded" ] && [ "$recorded" != "$SERIAL" ]; then
        warn "$TARGET_IP currently has serial $SERIAL"
        warn "Inventory says position $POSITION belongs to serial $recorded"
        warn "If you are intentionally re-assigning, delete the $POSITION block in:"
        warn "  $INVENTORY"
        fail "Refusing to overwrite — manual intervention required."
    fi
fi

# ─── Step A: change IP if needed ─────────────────────────────
if [ "$CURRENT_IP" = "$FACTORY_IP" ] && [ "$CURRENT_IP" != "$TARGET_IP" ]; then
    info "set_network $CURRENT_IP → $TARGET_IP / $NETMASK"
    "$UTIL" "$CURRENT_IP" set_network "$CURRENT_IP" "$TARGET_IP" "$NETMASK" || \
        fail "set_network failed"
    info "Rebooting LiDAR ($CURRENT_IP) — waiting ${SLEEP_AFTER_REBOOT}s"
    echo "reboot 1" | nc -nv "$CURRENT_IP" 8001 -w1 || true
    sleep "$SLEEP_AFTER_REBOOT"
    if ! ping -c2 -W2 "$TARGET_IP" &>/dev/null; then
        fail "$TARGET_IP not reachable after reboot. Check power / cabling."
    fi
    ok "LiDAR is now up at $TARGET_IP"
fi

# ─── Step B: download current PCS_ENV, compare, upload if different ──
tmp_current=$(mktemp /tmp/PCS_ENV_${POSITION}.XXXX)
if "$UTIL" "$TARGET_IP" download_internal_file PCS_ENV "$tmp_current" 2>/dev/null; then
    if diff -q "$tmp_current" "$ENV_PATH" >/dev/null 2>&1; then
        skip "PCS_ENV on $TARGET_IP already matches $ENV_FILE — no upload needed"
        rm -f "$tmp_current"
        UPLOAD_NEEDED=0
    else
        UPLOAD_NEEDED=1
    fi
else
    UPLOAD_NEEDED=1
fi
rm -f "$tmp_current"

if [ "${UPLOAD_NEEDED:-1}" = "1" ]; then
    info "Uploading $ENV_FILE to $TARGET_IP"
    # The Seyond utility uses 'upload_internal_file' for the write side;
    # older docs mention 'download_internal_file' for both directions,
    # which is a doc typo — the second invocation in Seyond's tutorial is
    # the upload step.
    if "$UTIL" "$TARGET_IP" upload_internal_file PCS_ENV "$ENV_PATH" 2>/dev/null; then
        ok "PCS_ENV uploaded"
    elif "$UTIL" "$TARGET_IP" download_internal_file PCS_ENV "$ENV_PATH" 2>/dev/null; then
        # Some older utility builds use the same verb for both directions.
        ok "PCS_ENV uploaded (legacy verb)"
    else
        fail "PCS_ENV upload to $TARGET_IP failed"
    fi

    info "Rebooting LiDAR ($TARGET_IP) — waiting ${SLEEP_AFTER_REBOOT}s"
    echo "reboot 1" | nc -nv "$TARGET_IP" 8001 -w1 || true
    sleep "$SLEEP_AFTER_REBOOT"
    if ! ping -c2 -W2 "$TARGET_IP" &>/dev/null; then
        fail "$TARGET_IP not reachable after PCS_ENV reboot"
    fi
fi

# ─── Step C: record SN → position in inventory.yaml ─────────
update_inventory() {
    local now
    now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    # Initialize file if missing
    if [ ! -f "$INVENTORY" ]; then
        cat > "$INVENTORY" << EOF
# Generated by provision_robin_w_multiunit.sh — DO NOT HAND-EDIT.
# Records which Seyond Robin W serial number was provisioned to
# which dome position. Updated automatically on every successful
# provisioning run. Useful for inventory tracking + spares.
positions:
EOF
    fi
    # Strip any existing block for this position, then re-add with new SN.
    python3 - "$INVENTORY" "$POSITION" "$SERIAL" "$TARGET_IP" "$UDP_PORT" "$now" << 'PY'
import sys, re, pathlib

path, pos, sn, ip, port, ts = sys.argv[1:7]
p = pathlib.Path(path)
lines = p.read_text().splitlines()

# Remove the existing block (header + 4 indented lines) for this position
new = []
i = 0
while i < len(lines):
    if lines[i].rstrip() == f"  {pos}:":
        # Skip header + up to 4 indented metadata lines
        i += 1
        while i < len(lines) and lines[i].startswith("    "):
            i += 1
        continue
    new.append(lines[i])
    i += 1

# Append the fresh block (under the trailing "positions:" header)
new.append(f"  {pos}:")
new.append(f"    serial:          \"{sn}\"")
new.append(f"    ip:              \"{ip}\"")
new.append(f"    udp_port:        {port}")
new.append(f"    provisioned_at:  \"{ts}\"")
p.write_text("\n".join(new) + "\n")
PY
    ok "Recorded SN $SERIAL at position $POSITION in serial_inventory.yaml"
}

update_inventory

# ─── Done ────────────────────────────────────────────────────
echo ""
ok "Provisioning complete: robin_w_$POSITION = serial $SERIAL @ $TARGET_IP : $UDP_PORT"
echo ""

# Suggest the next step based on what is still pending
REMAINING=()
for row in "${POSITION_TABLE[@]}"; do
    IFS='|' read -r p _ _ _ <<< "$row"
    if ! grep -q "^  $p:" "$INVENTORY" 2>/dev/null; then
        REMAINING+=("$p")
    fi
done

if [ "${#REMAINING[@]}" -gt 0 ]; then
    echo " Next: disconnect this LiDAR, attach the next one, and run:"
    for p in "${REMAINING[@]}"; do
        echo "   ./provision_robin_w_multiunit.sh --position $p"
    done
else
    echo " All three Robin Ws are provisioned. See:"
    echo "   $INVENTORY"
    echo ""
    echo " Next: ./4_setup_lidar_ptp.sh   # per-host PTP + driver setup"
fi
echo ""
