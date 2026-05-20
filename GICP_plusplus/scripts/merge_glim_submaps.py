#!/usr/bin/env python3
"""
merge_glim_submaps.py — Walk a GLIM dump directory and produce a
single PCD map for gicp_localization.

This closes the operator-side gap between GLIM_plusplus (which dumps
per-submap directories) and gicp_localization (which loads a single
PCD as its scan-to-map target). Replaces the manual workflow of
`ros2 run glim_ros offline_viewer` → File → Export PLY → convert_ply_to_pcd.py.

What it does
------------
1. Walks <dump_dir>/NNNNNN/ submap directories in numeric order.
2. For each submap:
     - Reads data.txt to extract T_world_origin (4×4 SE(3) double).
     - Reads points_compact.bin (gtsam_points::PointCloudCPU::save_compact format).
       The format is auto-detected: tries 32-byte (Eigen::Vector4d) and
       16-byte (Eigen::Vector4f) per-point layouts; picks the one whose
       size divides cleanly into the file. Falls back with a clear error
       if neither matches.
     - Applies T_world_origin to bring points into the map's world frame.
3. Concatenates all submaps into a single Open3D point cloud.
4. Applies (in order, each optional via CLI flag):
     - Z-clip (--z-min / --z-max)
     - Crop box around an XY centerline (--track-csv + --mask-width)
     - Statistical outlier removal (--outlier-k / --outlier-std)
     - Voxel downsample (--voxel-res)
5. Writes the result to a binary PCD file.
6. If <dump_dir>/T_world_utm.txt exists, copies it next to the output
   PCD so the operator can pass --ros-args
   -p localization/utm_transform_path:=<dump_dir>/T_world_utm.txt
   without re-finding the file.

Race-mode example
-----------------
    python3 merge_glim_submaps.py /tmp/dump /tmp/race_map.pcd \\
        --voxel-res 0.4 \\
        --outlier-k 12 --outlier-std 2.0 \\
        --z-min -2.0 --z-max 5.0

That produces a binary PCD with ~5–15 k points per visible voxel,
ready for gicp_localization to consume as localization/map_path.

Dependencies
------------
    pip install open3d numpy
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# open3d is imported lazily inside main() so --help renders even when
# the package isn't installed (useful on a fresh checkout).


# Supported per-point binary layouts in gtsam_points::PointCloudCPU::save_compact.
# (label, bytes per point, numpy dtype that decodes xyzw padded as Eigen::Vector*d/f).
_POINT_LAYOUTS = [
    ("Vector4d (float64 × 4)", 32, np.dtype([("x", "<f8"), ("y", "<f8"), ("z", "<f8"), ("w", "<f8")])),
    ("Vector4f (float32 × 4)", 16, np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("w", "<f4")])),
    ("Vector3d (float64 × 3)", 24, np.dtype([("x", "<f8"), ("y", "<f8"), ("z", "<f8")])),
    ("Vector3f (float32 × 3)", 12, np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])),
]


@dataclass
class Submap:
    idx: int
    T_world_origin: np.ndarray   # 4×4 float64
    points: np.ndarray           # N×3 float64 in submap-origin frame


def parse_data_txt(path: Path) -> np.ndarray:
    """Pull the T_world_origin 4×4 matrix out of GLIM's data.txt.

    The file has Eigen-formatted matrices preceded by their key on a
    separate line, e.g.:

        T_world_origin:
        1 0 0 0
        0 1 0 0
        0 0 1 0
        0 0 0 1

    We tokenize and pick the 16 floats following the literal
    "T_world_origin:" token, in row-major order.
    """
    text = path.read_text()
    # Split on whitespace (preserves Eigen's space-separated rows).
    tokens = text.split()
    try:
        i = tokens.index("T_world_origin:")
    except ValueError as e:
        raise RuntimeError(f"{path}: missing 'T_world_origin:' key") from e

    floats = tokens[i + 1: i + 1 + 16]
    if len(floats) != 16:
        raise RuntimeError(f"{path}: T_world_origin block truncated (got {len(floats)} entries)")
    try:
        vals = [float(v) for v in floats]
    except ValueError as e:
        raise RuntimeError(f"{path}: T_world_origin contains non-numeric token: {e}") from e

    return np.array(vals, dtype=np.float64).reshape((4, 4))


def parse_points_compact(path: Path) -> np.ndarray:
    """Parse a gtsam_points::PointCloudCPU::save_compact points_compact.bin.

    Tries each layout in _POINT_LAYOUTS, picks the one whose size
    divides cleanly into the file. Returns N×3 float64.
    """
    size = path.stat().st_size
    if size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    for label, bpp, dtype in _POINT_LAYOUTS:
        if size % bpp != 0:
            continue
        n = size // bpp
        if n == 0:
            continue
        with open(path, "rb") as f:
            buf = f.read()
        try:
            arr = np.frombuffer(buf, dtype=dtype, count=n)
        except ValueError:
            continue
        xyz = np.stack([arr["x"].astype(np.float64),
                        arr["y"].astype(np.float64),
                        arr["z"].astype(np.float64)], axis=1)
        # Sanity: the w-component should be ~1.0 for homogeneous-padded
        # Vector4 formats. If we got Vector4 but w is nowhere near 1,
        # the layout is wrong even though sizes matched — skip.
        if "w" in arr.dtype.names:
            mean_w = float(np.mean(arr["w"][:min(1000, n)]))
            if abs(mean_w - 1.0) > 0.5:
                continue
        # Bound check: typical map points are within ±1000 m of submap origin.
        finite_mask = np.isfinite(xyz).all(axis=1)
        if finite_mask.sum() < 1:
            continue
        bounded = (np.abs(xyz[finite_mask]).max() < 1.0e6)
        if not bounded:
            continue
        # Drop non-finite stragglers (rare).
        if not finite_mask.all():
            xyz = xyz[finite_mask]
        return xyz

    raise RuntimeError(
        f"{path}: could not parse {size} bytes as any known points_compact "
        f"layout. If your gtsam_points uses a different format, please "
        f"convert to PLY first via `ros2 run glim_ros offline_viewer` and "
        f"then use convert_ply_to_pcd.py."
    )


def load_submap(submap_dir: Path) -> Optional[Submap]:
    data_txt = submap_dir / "data.txt"
    points_bin = submap_dir / "points_compact.bin"
    if not data_txt.exists():
        print(f"  [skip] {submap_dir.name}: no data.txt", file=sys.stderr)
        return None
    if not points_bin.exists():
        print(f"  [skip] {submap_dir.name}: no points_compact.bin", file=sys.stderr)
        return None

    try:
        T = parse_data_txt(data_txt)
    except RuntimeError as e:
        print(f"  [skip] {submap_dir.name}: {e}", file=sys.stderr)
        return None

    try:
        pts = parse_points_compact(points_bin)
    except RuntimeError as e:
        print(f"  [skip] {submap_dir.name}: {e}", file=sys.stderr)
        return None

    return Submap(
        idx=int(re.match(r"^(\d+)$", submap_dir.name).group(1)) if re.match(r"^(\d+)$", submap_dir.name) else -1,
        T_world_origin=T,
        points=pts,
    )


def apply_T(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4×4 SE(3) to N×3 points; returns N×3."""
    if points.size == 0:
        return points
    n = points.shape[0]
    ones = np.ones((n, 1), dtype=np.float64)
    homog = np.concatenate([points, ones], axis=1)        # N×4
    transformed = homog @ T.T                              # N×4
    return transformed[:, :3]


def load_track_centerline(csv_path: Optional[Path]) -> Optional[np.ndarray]:
    """Read a 2-column (x,y) CSV of centerline waypoints, return N×2 float64."""
    if csv_path is None:
        return None
    rows = []
    with open(csv_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = re.split(r"[,\s]+", line)
            if len(parts) < 2:
                continue
            try:
                rows.append([float(parts[0]), float(parts[1])])
            except ValueError:
                continue
    if not rows:
        raise RuntimeError(f"{csv_path}: no usable rows (expecting x,y per line)")
    return np.array(rows, dtype=np.float64)


def filter_by_centerline(points_xyz: np.ndarray, centerline_xy: np.ndarray,
                         width_m: float) -> np.ndarray:
    """Keep points whose XY distance to the nearest centerline waypoint is ≤ width_m."""
    if points_xyz.shape[0] == 0 or centerline_xy.shape[0] == 0:
        return points_xyz
    # Brute-force NN — fine for a few thousand waypoints. For huge centerlines
    # swap in scipy.spatial.cKDTree if available.
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(centerline_xy)
        d, _ = tree.query(points_xyz[:, :2], k=1, workers=-1)
        mask = d <= width_m
    except ImportError:
        keep = np.zeros(points_xyz.shape[0], dtype=bool)
        for w in centerline_xy:
            d2 = (points_xyz[:, 0] - w[0]) ** 2 + (points_xyz[:, 1] - w[1]) ** 2
            keep |= d2 <= width_m * width_m
        mask = keep
    return points_xyz[mask]


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Merge GLIM submaps into a single PCD for gicp_localization.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("dump_dir", type=Path,
                    help="GLIM dump directory (the one passed to glim_rosbag -p dump_path:=...)")
    ap.add_argument("out_pcd", type=Path,
                    help="Output PCD file path.")
    ap.add_argument("--ascii", action="store_true",
                    help="Write ASCII PCD instead of binary (default: binary, ~3× faster loads).")
    # Z-clip
    ap.add_argument("--z-min", type=float, default=None,
                    help="Discard points below this Z (m, world frame). E.g., -2.0 for a roof-mounted dome.")
    ap.add_argument("--z-max", type=float, default=None,
                    help="Discard points above this Z (m, world frame). E.g., 5.0 to drop sky reflections.")
    # XY centerline mask
    ap.add_argument("--track-csv", type=Path, default=None,
                    help="Optional CSV of (x,y) centerline waypoints; only points within --mask-width m are kept.")
    ap.add_argument("--mask-width", type=float, default=30.0,
                    help="Half-width (m) of the centerline corridor when --track-csv is set. Default: 30.0.")
    # Outlier removal
    ap.add_argument("--outlier-k", type=int, default=0,
                    help="Statistical outlier removal: number of neighbors. 0 (default) disables.")
    ap.add_argument("--outlier-std", type=float, default=2.0,
                    help="Statistical outlier removal: standard-deviation multiplier (default 2.0).")
    # Voxel downsample
    ap.add_argument("--voxel-res", type=float, default=0.5,
                    help="Voxel downsample resolution in metres. Set to 0 to disable. Default 0.5.")
    # T_world_utm passthrough
    ap.add_argument("--copy-utm", action="store_true",
                    help="If <dump_dir>/T_world_utm.txt exists, copy it next to the output PCD.")

    args = ap.parse_args()

    try:
        import open3d as o3d
    except ImportError:
        sys.exit("ERROR: open3d is required. Install with: pip install open3d")

    if not args.dump_dir.is_dir():
        sys.exit(f"ERROR: {args.dump_dir} is not a directory")

    submap_dirs = sorted(
        d for d in args.dump_dir.iterdir()
        if d.is_dir() and re.match(r"^\d{6}$", d.name)
    )
    if not submap_dirs:
        sys.exit(f"ERROR: no submap subdirectories (e.g. 000000/) under {args.dump_dir}")

    print(f"Walking {len(submap_dirs)} submap directories under {args.dump_dir} ...")
    all_points: List[np.ndarray] = []
    layout_seen: Optional[str] = None
    for d in submap_dirs:
        sm = load_submap(d)
        if sm is None or sm.points.size == 0:
            continue
        if layout_seen is None:
            # Re-probe to log which layout fired (informational).
            for label, bpp, _ in _POINT_LAYOUTS:
                if (d / "points_compact.bin").stat().st_size % bpp == 0:
                    layout_seen = label
                    break
        world_pts = apply_T(sm.points, sm.T_world_origin)
        all_points.append(world_pts)
        print(f"  + {d.name}: {sm.points.shape[0]:>10,d} pts  (layout: {layout_seen})")

    if not all_points:
        sys.exit("ERROR: every submap was empty or unreadable")

    merged = np.concatenate(all_points, axis=0)
    print(f"Merged total: {merged.shape[0]:,d} points before filters")

    # --- Z-clip ---
    if args.z_min is not None:
        merged = merged[merged[:, 2] >= args.z_min]
        print(f"After --z-min={args.z_min:.2f}: {merged.shape[0]:,d} points")
    if args.z_max is not None:
        merged = merged[merged[:, 2] <= args.z_max]
        print(f"After --z-max={args.z_max:.2f}: {merged.shape[0]:,d} points")

    # --- Centerline mask ---
    if args.track_csv is not None:
        centerline = load_track_centerline(args.track_csv)
        merged = filter_by_centerline(merged, centerline, args.mask_width)
        print(f"After centerline corridor (--mask-width={args.mask_width:.1f}m, "
              f"{centerline.shape[0]} waypoints): {merged.shape[0]:,d} points")

    if merged.shape[0] == 0:
        sys.exit("ERROR: filters left zero points. Loosen the constraints.")

    # --- Hand off to Open3D for outlier removal + voxel downsample + PCD write ---
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(merged)

    if args.outlier_k > 0:
        pcd, _ = pcd.remove_statistical_outlier(args.outlier_k, args.outlier_std)
        print(f"After statistical outlier removal "
              f"(k={args.outlier_k}, std={args.outlier_std}): {len(pcd.points):,d} points")

    if args.voxel_res > 0:
        pcd = pcd.voxel_down_sample(args.voxel_res)
        print(f"After voxel downsample ({args.voxel_res:.3f} m): {len(pcd.points):,d} points")

    args.out_pcd.parent.mkdir(parents=True, exist_ok=True)
    write_ascii = bool(args.ascii)
    if not o3d.io.write_point_cloud(str(args.out_pcd), pcd, write_ascii=write_ascii):
        sys.exit(f"ERROR: failed to write {args.out_pcd}")
    print(f"Wrote {args.out_pcd}  ({len(pcd.points):,d} points, "
          f"{'ASCII' if write_ascii else 'binary'} PCD)")

    if args.copy_utm:
        src = args.dump_dir / "T_world_utm.txt"
        if src.exists():
            dst = args.out_pcd.with_name(args.out_pcd.stem + "_T_world_utm.txt")
            shutil.copyfile(src, dst)
            print(f"Copied {src} → {dst}")
            print(f"  use with: --ros-args -p localization/utm_transform_path:={dst}")
        else:
            print(f"  --copy-utm requested but {src} does not exist (no GNSS factor was active?)")

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
