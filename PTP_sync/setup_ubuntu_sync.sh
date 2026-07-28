#!/usr/bin/env bash
# =============================================================
# DEPRECATED — split into 1_/2_/3_ scripts.
#
# The monolithic setup_ubuntu_sync.sh has been retired in favour
# of three focused scripts that each handle one logical step of
# the host setup:
#
#   1_install_packages.sh         (was §1 + §2 + §3 + §10 of this)
#       apt prerequisites, RT scheduling permissions,
#       kernel sysctl tuning, ROS 2 Humble/Jazzy install.
#
#   2_configure_host_network.sh   (was §4 of this)
#       host NIC static IP, hardware-timestamping detection,
#       reachability ping of the RUTM50 router.
#
#   3_setup_ins_to_pc_sync.sh     (was §5–§9 + §11 of this)
#       gpsd, chrony with NMEA, ptp4l grandmaster, phc2sys,
#       Point One host tools (native Atlas ROS driver is deployment-provided).
#
# Run them in order:
#
#   ./1_install_packages.sh
#   ./2_configure_host_network.sh
#   ./3_setup_ins_to_pc_sync.sh
#
# Once you've run the new scripts, you can safely remove this
# tombstone:
#
#   git rm PTP_sync/setup_ubuntu_sync.sh
#   git commit -m "PTP_sync: remove deprecated setup_ubuntu_sync.sh"
#
# (This file is kept in the repo only because the script's
# automation sandbox cannot unlink it. If you arrived here from
# a stale bookmark or shell alias, jump to script 1 above.)
# =============================================================

cat << 'BANNER' >&2
[DEPRECATED] setup_ubuntu_sync.sh has been split into:
    1_install_packages.sh
    2_configure_host_network.sh
    3_setup_ins_to_pc_sync.sh

Please run those three in order instead. This script no longer
does anything — it just prints this banner and exits.

See PTP_sync/README.md §1–§3 for the new procedure.
BANNER
exit 1
