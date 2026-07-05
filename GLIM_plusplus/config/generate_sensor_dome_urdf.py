#!/usr/bin/env python3
# =============================================================================
# generate_sensor_dome_urdf.py
#
# Convert ../../config/sensor_dome_tf.yaml into a URDF that GLIM's
# sensor-config loader can consume (the same URDF code path that the
# original AV-24 deployment used). This keeps sensor_dome_tf.yaml as the
# single source of truth for every static sensor → IMU transform.
#
# Output: GLIM/config/sensor_dome.urdf
#
# The URDF tree is a star rooted at imu_link, with one fixed joint per
# sensor frame in the YAML:
#
#       ├── cam_front_left_link
#       │   └── cam_front_left_optical_frame
#       ├── cam_front_right_link
#       │   └── cam_front_right_optical_frame
#       ├── cam_rear_left_link
#       │   └── cam_rear_left_optical_frame
#       ├── cam_rear_right_link
#       │   └── cam_rear_right_optical_frame
#       ├── base_link               (identity by default; vehicle body
#       │                            frame for downstream consumers)
#       ├── lidar_front_link        (yaw   0°)
#       ├── lidar_rear_left_link    (yaw 120°)
#       ├── lidar_rear_right_link   (yaw 240°)
#       ├── gnss_antenna_primary_link
#       └── gnss_antenna_secondary_link (optional)
#
# Note that base_link is a CHILD of imu_link here (not the URDF root
# convention). This is intentional: the map is anchored at imu_link
# at session start (GLIM_plusplus), and base_link is positioned
# relative to imu_link via the YAML so the operator can override
# the vehicle body frame without invalidating the map.
#
# GLIM consumes this file via the urdf_path entries in config_sensors.json
# (T_lidar_imu) and the lidar_concat block (multi-LiDAR primary + aux).
#
# Re-run this script whenever sensor_dome_tf.yaml changes. Typical use:
#
#     cd GLIM/config && python3 generate_sensor_dome_urdf.py
# =============================================================================

from __future__ import annotations

import math
import sys
import xml.dom.minidom as minidom
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("ERROR: pyyaml is required. Install with: sudo apt install python3-yaml")

HERE = Path(__file__).resolve().parent
# The TF YAML lives in /config/ (top-level), per the consolidation that
# happened after the recording/ module was added.
_REPO = HERE.parent.parent
TF_YAML = _REPO / "config" / "sensor_dome_tf.yaml"
OUT_URDF = HERE / "sensor_dome.urdf"


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Quaternion (x,y,z,w) → URDF roll-pitch-yaw (rad).

    Standard ROS conversion: yaw is rotation about Z, applied last in the
    R = Rz(y) * Ry(p) * Rx(r) chain. For the yaw-only transforms in
    sensor_dome_tf.yaml this collapses to roll = pitch = 0.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main() -> int:
    if not TF_YAML.exists():
        sys.exit(f"ERROR: TF YAML not found at {TF_YAML}")

    with open(TF_YAML) as f:
        cfg = yaml.safe_load(f)

    transforms = cfg.get("static_transforms", [])
    if not transforms:
        sys.exit(f"ERROR: no 'static_transforms' entries in {TF_YAML}")

    # Validate that every transform forms one tree rooted at imu_link.
    # Camera optical frames are parented to their camera links, while the
    # physical sensor frames remain direct imu_link children.
    children = [tf["child_frame_id"] for tf in transforms]
    duplicate_children = sorted({child for child in children if children.count(child) > 1})
    if duplicate_children:
        sys.exit(f"ERROR: duplicate child_frame_id entries: {duplicate_children}")

    child_set = set(children)
    parent_set = {tf["frame_id"] for tf in transforms}
    unknown_parents = sorted(parent_set - child_set - {"imu_link"})
    if unknown_parents:
        sys.exit(
            "ERROR: every non-root parent frame must also appear as a child "
            f"frame; unknown parents: {unknown_parents}"
        )

    children_by_parent = {}
    for tf in transforms:
        children_by_parent.setdefault(tf["frame_id"], []).append(tf["child_frame_id"])
    reachable = set()
    stack = ["imu_link"]
    while stack:
        parent = stack.pop()
        for child in children_by_parent.get(parent, []):
            if child in reachable:
                continue
            reachable.add(child)
            stack.append(child)
    disconnected = sorted(child_set - reachable)
    if disconnected:
        sys.exit(f"ERROR: transforms are not connected to imu_link: {disconnected}")

    # Build the URDF document.
    doc = minidom.Document()
    robot = doc.createElement("robot")
    robot.setAttribute("name", "sensor_dome")
    doc.appendChild(robot)

    # Header comment.
    rel = TF_YAML.relative_to(_REPO) if TF_YAML.is_relative_to(_REPO) else TF_YAML
    robot.appendChild(doc.createComment(
        f" Auto-generated from {rel} — DO NOT EDIT BY HAND. "
        "Re-run GLIM/config/generate_sensor_dome_urdf.py to regenerate."
    ))

    def add_link(name: str) -> None:
        link = doc.createElement("link")
        link.setAttribute("name", name)
        robot.appendChild(link)

    links_added = set()

    def add_link_once(name: str) -> None:
        if name in links_added:
            return
        add_link(name)
        links_added.add(name)

    add_link_once("imu_link")

    for tf in transforms:
        parent = tf["frame_id"]
        child = tf["child_frame_id"]
        t = tf["translation"]
        r = tf["rotation"]
        roll, pitch, yaw = quat_to_rpy(r["x"], r["y"], r["z"], r["w"])

        add_link_once(parent)
        add_link_once(child)

        joint = doc.createElement("joint")
        joint.setAttribute("name", f"{parent}_to_{child}")
        joint.setAttribute("type", "fixed")

        parent_el = doc.createElement("parent")
        parent_el.setAttribute("link", parent)
        joint.appendChild(parent_el)

        child_el = doc.createElement("child")
        child_el.setAttribute("link", child)
        joint.appendChild(child_el)

        origin = doc.createElement("origin")
        origin.setAttribute("xyz", f"{t['x']:.6f} {t['y']:.6f} {t['z']:.6f}")
        origin.setAttribute("rpy", f"{roll:.6f} {pitch:.6f} {yaw:.6f}")
        joint.appendChild(origin)

        robot.appendChild(joint)

    # Pretty-print and write.
    xml_str = doc.toprettyxml(indent="  ", encoding="utf-8").decode("utf-8")
    # Drop blank lines that minidom emits between siblings.
    xml_str = "\n".join(line for line in xml_str.splitlines() if line.strip())
    OUT_URDF.write_text(xml_str + "\n")
    print(f"Wrote {OUT_URDF}")
    print(f"  imu_link → {len(transforms)} child frames")

    # Dual-antenna detection summary — informational. The GLIM++ wrapper
    # does the same check at startup and auto-enables the dual-antenna
    # code paths when the secondary translation norm exceeds the
    # threshold (also DUAL_BASELINE_THRESH = 0.05 m).
    DUAL_BASELINE_THRESH = 0.05  # m
    primary = next(
        (tf for tf in transforms
         if tf["child_frame_id"] == "gnss_antenna_primary_link"),
        None)
    secondary = next(
        (tf for tf in transforms
         if tf["child_frame_id"] == "gnss_antenna_secondary_link"),
        None)
    if primary is not None and secondary is not None:
        # Sentinel: secondary translation = (0, 0, 0) means "not
        # configured". Any actual installation will have the second
        # antenna at a non-origin location in the imu_link frame.
        st = secondary["translation"]
        sec_norm = (st["x"] ** 2 + st["y"] ** 2 + st["z"] ** 2) ** 0.5
        if sec_norm < DUAL_BASELINE_THRESH:
            print(f"  GNSS antenna mode: SINGLE  "
                  f"(secondary translation norm {sec_norm:.3f} m "
                  f"< {DUAL_BASELINE_THRESH:.2f} m threshold)")
            print("  ⇒ GLIM++ will run in single-antenna mode. To enable "
                  "dual-antenna heading, edit config/sensor_dome_tf.yaml "
                  "and set the gnss_antenna_secondary_link translation.")
        else:
            # Baseline vector from primary to secondary, in imu_link frame.
            pt = primary["translation"]
            bv = (st["x"] - pt["x"], st["y"] - pt["y"], st["z"] - pt["z"])
            bl = (sum(c * c for c in bv)) ** 0.5
            heading_sigma_rad = 0.01 / max(bl, 0.05)   # 1 cm RTK / baseline
            heading_sigma_deg = heading_sigma_rad * 180.0 / 3.141592653589793
            print(f"  GNSS antenna mode: DUAL    "
                  f"(baseline = {bl:.3f} m, "
                  f"vec = ({bv[0]:+.3f}, {bv[1]:+.3f}, {bv[2]:+.3f}) m)")
            print(f"  ⇒ GLIM++ will enable dual-antenna code paths. "
                  f"Expected heading σ ≈ {heading_sigma_deg:.2f}° "
                  "(at RTK-fixed, 1 cm position σ).")
    elif primary is not None:
        print("  GNSS antenna mode: PARTIAL — primary present, secondary missing.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
