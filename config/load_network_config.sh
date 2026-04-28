#!/usr/bin/env bash
# =============================================================
# load_network_config.sh — source from setup_*.sh
#
# Reads /config/network_config.yaml and exports its values as
# shell variables so the PTP_sync setup scripts (and any other
# bash tooling) can pull defaults from a single source of truth.
#
# Usage from a setup script:
#
#     SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#     source "$SCRIPT_DIR/../config/load_network_config.sh"
#
#     ETH_IFACE="${ETH_IFACE:-$NETCFG_ETH}"
#     LIDAR_IPS_STR="${LIDAR_IPS_STR:-$NETCFG_LIDAR_IPS}"
#     ...
#
# After sourcing, the following variables are exported:
#
#     NETCFG_FILE          absolute path to the YAML used
#     NETCFG_ETH           host.interface          (e.g. enp0s31f6)
#     NETCFG_HOST_IP       host.ip                 (e.g. 192.168.1.40)
#     NETCFG_GATEWAY       gateway                 (e.g. 192.168.1.1)
#     NETCFG_SUBNET        subnet                  (e.g. 192.168.1.0/24)
#     NETCFG_ATLAS_IP      atlas_duo.ethernet_ip   (e.g. 192.168.1.30)
#     NETCFG_ATLAS_SERIAL  atlas_duo.serial_port   (e.g. /dev/ttyUSB0)
#     NETCFG_ATLAS_PPS     atlas_duo.pps_device    (e.g. /dev/pps0)
#     NETCFG_LIDAR_IPS     comma-joined lidar IPs  (e.g. .10,.11,.12)
#     NETCFG_CAMERA_IPS    comma-joined camera IPs
#     NETCFG_ROUTER_IP     router.ip
#     NETCFG_SWITCH_IP     switch.ip
#
# Override the YAML path with NETCFG_FILE=/path/to/other.yaml
# before sourcing.
# =============================================================

# Resolve default YAML location (next to this file).
_netcfg_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NETCFG_FILE="${NETCFG_FILE:-$_netcfg_dir/network_config.yaml}"
unset _netcfg_dir

if [[ ! -f "$NETCFG_FILE" ]]; then
    echo "[FATAL] network config not found: $NETCFG_FILE" >&2
    return 1 2>/dev/null || exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "[FATAL] python3 required to parse $NETCFG_FILE" >&2
    return 1 2>/dev/null || exit 1
fi

# Render the YAML to `export KEY=VALUE` lines via a quoted heredoc
# (so $variables in the python source aren't pre-expanded by bash).
_netcfg_exports="$(NETCFG_FILE="$NETCFG_FILE" python3 - <<'PY'
import os, sys, shlex
try:
    import yaml
except ImportError:
    sys.stderr.write("[FATAL] python3 yaml module missing — `pip install pyyaml`\n")
    sys.exit(1)

path = os.environ["NETCFG_FILE"]
try:
    with open(path) as f:
        c = yaml.safe_load(f) or {}
except Exception as e:
    sys.stderr.write(f"[FATAL] cannot parse {path}: {e}\n")
    sys.exit(1)

def need(d, *keys):
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            sys.stderr.write(
                f"[FATAL] {path} missing key: {' -> '.join(keys)}\n")
            sys.exit(1)
        cur = cur[k]
    return cur

def emit(name, value):
    print(f"export {name}={shlex.quote(str(value))}")

emit("NETCFG_FILE",         path)
emit("NETCFG_ETH",          need(c, "host", "interface"))
emit("NETCFG_HOST_IP",      need(c, "host", "ip"))
emit("NETCFG_GATEWAY",      need(c, "gateway"))
emit("NETCFG_SUBNET",       need(c, "subnet"))
emit("NETCFG_ROUTER_IP",    need(c, "router", "ip"))
emit("NETCFG_SWITCH_IP",    need(c, "switch", "ip"))
emit("NETCFG_ATLAS_IP",     need(c, "atlas_duo", "ethernet_ip"))
emit("NETCFG_ATLAS_SERIAL", need(c, "atlas_duo", "serial_port"))
emit("NETCFG_ATLAS_PPS",    need(c, "atlas_duo", "pps_device"))

emit("NETCFG_LIDAR_IPS",
     ",".join(l["ip"] for l in c.get("lidars", []) if "ip" in l))
emit("NETCFG_CAMERA_IPS",
     ",".join(k["ip"] for k in c.get("cameras", []) if "ip" in k))
PY
)" || {
    echo "[FATAL] failed to load $NETCFG_FILE" >&2
    return 1 2>/dev/null || exit 1
}

eval "$_netcfg_exports"
unset _netcfg_exports
