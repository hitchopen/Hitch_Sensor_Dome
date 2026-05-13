#!/usr/bin/env bash
# =============================================================
# Robin W Multi-Unit Provisioning (run ONCE per LiDAR)
#
# Walks each Seyond Robin W from factory state to project state:
#
#   factory IP (172.168.1.10)  ─┐
#                               ├─→ set_network → reboot → upload
#                               │   PCS_ENV with unit-specific UDP
#                               │   port → verify
#   final IPs per network_config.yaml (192.168.1.0/24)
#       robin_w_front       .10
#       robin_w_rear_left   .11
#       robin_w_rear_right  .12
#
# The script auto-adds a temporary 172.168.1.100/24 IP alias to the
# sensor NIC so the host (on 192.168.1.40) can reach factory-state
# LiDARs. Alias is removed on exit.
#
# This is the same procedure Seyond publishes for running three
# Robin Ws on the same Ethernet segment (multi-unit unicast).
# Two changes are required per LiDAR:
#   1. Unique IP, so the host can address each unit individually.
#   2. Unique UDP destination port, so the three point-cloud
#      streams don't collide at the host receiver.
#
# Both are handled here. Run ONCE per LiDAR — re-running it on an
# already-configured LiDAR is harmless (it detects the new IP and
# skips), but the standard per-host setup (./setup_robin_w_sync.sh)
# is what you run on every fresh OS install of the host PC.
#
# Files this script uses (all in PTP_sync/):
#   innovusion_lidar_util                  Seyond CLI binary
#   RobinW_FW2835_Multiunit/                Per-serial PCS_ENV files
#       RW_FW2835_Allen_<serial>_unicast.env
#   ../config/network_config.yaml          Target IPs + host IP
#
# Usage:
#   chmod +x provision_robin_w_multiunit.sh
#   ./provision_robin_w_multiunit.sh             # provision all 3
#   ./provision_robin_w_multiunit.sh --only front
#   ./provision_robin_w_multiunit.sh --factory-ip 192.168.1.10
#                                                # if not factory default
#
# Re-run is safe: a LiDAR already at its final IP with the right
# PCS_ENV is skipped with [SKIP] and the script moves on.
# =============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../config/load_network_config.sh
source "$SCRIPT_DIR/../config/load_network_config.sh"

# ─── Defaults ────────────────────────────────────────────────
# Per Seyond's official "Robin W1G LiDAR User Manual" V2.2 (2025-01-03), §3.1:
#     The initial IP address of the LiDAR is 172.168.1.10.
#     The initial subnet mask is 255.255.255.0.
#     The initial gateway is 172.168.1.1.
# Note that 172.168.1.0/24 is publicly-routable IP space (it is NOT
# inside RFC 1918's 172.16.0.0/12 private block — that ends at 172.31.x.x).
# The dome network runs on 192.168.1.0/24 (per network_config.yaml), so
# this script's job is to move each LiDAR FROM the Seyond factory IP TO
# its assigned 192.168.1.x address.
FACTORY_IP="${FACTORY_IP:-172.168.1.10}"
NETMASK="${NETMASK:-255.255.255.0}"
ONLY=""                                    # filter: front / rear_left / rear_right
SLEEP_AFTER_REBOOT="${SLEEP_AFTER_REBOOT:-25}"   # seconds — Robin W needs ~20 s
                                                  # to come back up after reboot

# ─── Static unit table — keep in sync with
# ─── RobinW_FW2835_Multiunit/README.md
# Format: position|serial|target_ip|udp_port|pcs_env_filename
UNIT_TABLE=(
  "front|533192400101|192.168.1.10|8010|RW_FW2835_Allen_533192400101_unicast.env"
  "rear_left|533262400110|192.168.1.11|8020|RW_FW2835_Allen_533262400110_unicast.env"
  "rear_right|533192400103|192.168.1.12|8030|RW_FW2835_Allen_533192400103_unicast.env"
)

# ─── Parse args ──────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --only)         ONLY="$2"; shift 2 ;;
        --only=*)       ONLY="${1#*=}"; shift ;;
        --factory-ip)   FACTORY_IP="$2"; shift 2 ;;
        --factory-ip=*) FACTORY_IP="${1#*=}"; shift ;;
        --netmask)      NETMASK="$2"; shift 2 ;;
        --netmask=*)    NETMASK="${1#*=}"; shift ;;
        -h|--help)
            sed -n '2,50p' "$0"; exit 0 ;;
        *) echo "Unknown arg: $1 — use --help"; exit 1 ;;
    esac
done

# ─── Helpers ─────────────────────────────────────────────────
info()  { echo -e "\n\033[1;34m[INFO]\033[0m $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m $*"; }
ok()    { echo -e "\033[1;32m[ OK ]\033[0m $*"; }
skip()  { echo -e "\033[1;36m[SKIP]\033[0m $*"; }
fail()  { echo -e "\033[1;31m[FAIL]\033[0m $*"; exit 1; }

# ─── Reachability fix: temporary IP alias on the sensor NIC ──
#
# Brand-new LiDARs are at Seyond's factory IP 172.168.1.10 (per the
# Robin W1G User Manual §3.1). The dome host PC lives on the
# 192.168.1.0/24 subnet (per network_config.yaml). The two subnets do
# not overlap, so the host cannot reach a factory-state LiDAR without
# an IP in 172.168.1.0/24.
#
# We add a temporary alias 172.168.1.100/24 to the sensor NIC at the
# start of provisioning, and remove it on exit. The alias coexists
# with the host's 192.168.1.40/24 address (both are reachable at L2),
# so post-provisioning pings to the new 192.168.1.X address also work
# from the same script run.
#
# The alias is skipped if the host already has an address in
# 172.168.1.0/24 (rare; only happens if you've manually pre-configured
# this).
ALIAS_ADDED=0
ALIAS_IP="${FACTORY_IP%.*}.100/24"   # e.g. 172.168.1.100/24
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
        ok "Alias up — host reachable on both 192.168.1.40 and ${ALIAS_IP%/*}"
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

# Add the alias only if at least one LiDAR is expected to still be at
# the factory IP. If all three are already provisioned (per the table
# below), this is a no-op re-run and we skip the alias.
NEED_ALIAS=0
for row in "${UNIT_TABLE[@]}"; do
    IFS='|' read -r _ _ target_ip _ _ <<< "$row"
    if [ -n "$ONLY" ]; then
        IFS='|' read -r position _ _ _ _ <<< "$row"
        [ "$ONLY" != "$position" ] && continue
    fi
    # If the target IP doesn't respond, the LiDAR is either off or
    # still at the factory IP — alias is needed in case it's the latter.
    if ! ping -c1 -W1 "$target_ip" &>/dev/null; then NEED_ALIAS=1; fi
done
if [ "$NEED_ALIAS" -eq 1 ]; then maybe_add_alias; fi

# ─── Locate the Seyond utility ───────────────────────────────
UTIL=""
for c in "$SCRIPT_DIR/innovusion_lidar_util" \
         "./innovusion_lidar_util" \
         "innovusion_lidar_util"; do
    if [ -x "$c" ] || command -v "$c" &>/dev/null; then UTIL="$c"; break; fi
done
[ -z "$UTIL" ] && fail "innovusion_lidar_util not found. Place it in PTP_sync/."
ok "Using utility: $UTIL"

ENV_DIR="$SCRIPT_DIR/RobinW_FW2835_Multiunit"
[ -d "$ENV_DIR" ] || fail "Missing $ENV_DIR (PCS_ENV files)."

# ─── Sanity check: host IP must be in the PCS_ENV files ─────
HOST_IP="$NETCFG_HOST_IP"
[ -z "$HOST_IP" ] && fail "Host IP missing in network_config.yaml"
for f in "$ENV_DIR"/RW_FW2835_*_unicast.env; do
    if ! grep -q "^UDP_IP=${HOST_IP}$" "$f"; then
        fail "$f has UDP_IP != $HOST_IP — edit the PCS_ENV file or update network_config.yaml host.ip"
    fi
done
ok "PCS_ENV files agree with host IP $HOST_IP"

# ─── Provision one unit ──────────────────────────────────────
provision_unit() {
    local position="$1" serial="$2" target_ip="$3" udp_port="$4" env_file="$5"
    local env_path="$ENV_DIR/$env_file"

    info "─── Provisioning robin_w_$position (serial $serial → $target_ip, UDP $udp_port) ───"

    if [ ! -f "$env_path" ]; then
        warn "Missing $env_path — skipping"
        return 1
    fi

    # ── Step A: locate the LiDAR ──
    local current_ip=""
    if ping -c1 -W2 "$target_ip" &>/dev/null; then
        local sn
        sn=$("$UTIL" "$target_ip" get_static_info 2>/dev/null | \
             grep -oE 'sn[: =]*[0-9]+' | head -1 | grep -oE '[0-9]+' || true)
        if [ "$sn" = "$serial" ]; then
            ok "Already at $target_ip with matching serial — verifying PCS_ENV..."
            current_ip="$target_ip"
        elif [ -n "$sn" ]; then
            warn "$target_ip is occupied by serial $sn (not $serial) — manual cleanup needed"
            return 1
        else
            warn "$target_ip is reachable but did not answer get_static_info — assuming factory state"
            current_ip="$FACTORY_IP"
        fi
    elif ping -c1 -W2 "$FACTORY_IP" &>/dev/null; then
        ok "Found a Robin W at factory IP $FACTORY_IP — checking serial..."
        local sn
        sn=$("$UTIL" "$FACTORY_IP" get_static_info 2>/dev/null | \
             grep -oE 'sn[: =]*[0-9]+' | head -1 | grep -oE '[0-9]+' || true)
        if [ -n "$sn" ] && [ "$sn" != "$serial" ]; then
            warn "Factory-IP LiDAR has serial $sn (expected $serial). Unplug the other units"
            warn "and run with --only $position to provision only this one."
            return 1
        fi
        current_ip="$FACTORY_IP"
    else
        warn "Neither $target_ip nor $FACTORY_IP responds. Power on the LiDAR and re-run."
        return 1
    fi

    # ── Step B: change IP if needed ──
    if [ "$current_ip" = "$FACTORY_IP" ] && [ "$current_ip" != "$target_ip" ]; then
        info "set_network $current_ip → $target_ip / $NETMASK"
        "$UTIL" "$current_ip" set_network "$current_ip" "$target_ip" "$NETMASK" || \
            fail "set_network failed"
        info "Rebooting LiDAR ($current_ip) — waiting ${SLEEP_AFTER_REBOOT}s"
        echo "reboot 1" | nc -nv "$current_ip" 8001 -w1 || true
        sleep "$SLEEP_AFTER_REBOOT"
        if ! ping -c2 -W2 "$target_ip" &>/dev/null; then
            fail "$target_ip not reachable after reboot. Check power / cabling."
        fi
        ok "LiDAR up at $target_ip"
    fi

    # ── Step C: download current PCS_ENV, compare, upload if different ──
    local tmp_current
    tmp_current=$(mktemp /tmp/PCS_ENV_${position}.XXXX)
    if "$UTIL" "$target_ip" download_internal_file PCS_ENV "$tmp_current" 2>/dev/null; then
        if diff -q "$tmp_current" "$env_path" >/dev/null 2>&1; then
            skip "PCS_ENV on $target_ip already matches $env_file — nothing to upload"
            rm -f "$tmp_current"
            return 0
        fi
    fi
    rm -f "$tmp_current"

    info "Uploading $env_file to $target_ip"
    # The Seyond utility uses 'upload_internal_file' for write-side; older docs
    # mention 'download_internal_file' for both directions, which is a doc typo
    # — the second invocation in Seyond's tutorial is the upload step.
    if "$UTIL" "$target_ip" upload_internal_file PCS_ENV "$env_path" 2>/dev/null; then
        ok "PCS_ENV uploaded"
    elif "$UTIL" "$target_ip" download_internal_file PCS_ENV "$env_path" 2>/dev/null; then
        # Some older utility builds use the same verb for both directions.
        ok "PCS_ENV uploaded (legacy verb)"
    else
        fail "PCS_ENV upload to $target_ip failed"
    fi

    info "Rebooting LiDAR ($target_ip) — waiting ${SLEEP_AFTER_REBOOT}s"
    echo "reboot 1" | nc -nv "$target_ip" 8001 -w1 || true
    sleep "$SLEEP_AFTER_REBOOT"
    if ! ping -c2 -W2 "$target_ip" &>/dev/null; then
        fail "$target_ip not reachable after PCS_ENV reboot"
    fi
    ok "Provisioning complete for robin_w_$position"
    return 0
}

# ─── Main ────────────────────────────────────────────────────
info "Robin W multi-unit provisioning — host UDP_IP=$HOST_IP, factory IP=$FACTORY_IP"
echo ""
ANY_FAIL=0
for row in "${UNIT_TABLE[@]}"; do
    IFS='|' read -r position serial target_ip udp_port env_file <<< "$row"
    if [ -n "$ONLY" ] && [ "$ONLY" != "$position" ]; then continue; fi
    if ! provision_unit "$position" "$serial" "$target_ip" "$udp_port" "$env_file"; then
        ANY_FAIL=1
    fi
done

echo ""
if [ "$ANY_FAIL" -eq 0 ]; then
    ok "All requested Robin Ws provisioned."
    echo ""
    echo " Next: ./setup_robin_w_sync.sh   # per-host PTP + driver setup"
else
    warn "One or more units could not be provisioned — see [WARN]/[FAIL] above."
    exit 2
fi
