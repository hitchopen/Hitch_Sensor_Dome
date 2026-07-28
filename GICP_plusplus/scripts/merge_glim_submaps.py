#!/usr/bin/env python3
"""Compatibility entry point for the canonical GLIM-to-GICP map exporter.

The former implementation guessed the compact point layout and wrote a map in
GLIM's WORLD frame. Both behaviors are unsafe for GICP, whose adapter odometry
is in a surveyed local-ENU frame. This wrapper delegates to the maintained
exporter, which decodes the exact compact format, applies
``inverse(T_world_utm)``, and writes a datum manifest.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump_dir", type=Path)
    parser.add_argument("output_pcd", type=Path)
    parser.add_argument(
        "--voxel-res",
        "--voxel-size",
        dest="voxel_size",
        type=float,
        default=0.0,
        help="Output voxel size in metres.",
    )
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument(
        "--enu-origin",
        default="",
        help="Output/live adapter datum: 'lat_deg,lon_deg,alt_m'.",
    )
    parser.add_argument(
        "--gnss-enu-origin",
        default="",
        help="Mapping GNSS datum when re-anchoring to a different output datum.",
    )
    parser.add_argument("--transform-file", type=Path)
    args = parser.parse_args()

    exporter = (
        Path(__file__).resolve().parents[2]
        / "GLIM_plusplus"
        / "scripts"
        / "export_glim_dump_to_pcd.py"
    )
    if not exporter.is_file():
        parser.error(
            f"canonical exporter not found at {exporter}; run "
            "GLIM_plusplus/scripts/export_glim_dump_to_pcd.py directly"
        )

    command = [
        sys.executable,
        str(exporter),
        str(args.dump_dir),
        str(args.output_pcd),
        "--frame",
        "enu",
        "--voxel-size",
        str(args.voxel_size),
        "--stride",
        str(args.stride),
    ]
    if args.enu_origin:
        command.extend(["--enu-origin", args.enu_origin])
    if args.gnss_enu_origin:
        command.extend(["--gnss-enu-origin", args.gnss_enu_origin])
    if args.transform_file:
        command.extend(["--transform-file", str(args.transform_file)])

    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
