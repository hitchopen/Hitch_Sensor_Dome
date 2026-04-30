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
#     imu_link
#       ├── lidar_front_link        (yaw   0°)
#       ├── lidar_rear_left_link    (yaw 120°)
#       ├── lidar_rear_right_link   (yaw 240°)
#       ├── cam_front_right_link
#       ├── cam_front_left_link
#       ├── cam_rear_left_link
#       └── cam_rear_right_link
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
    sys.exit("ERROR: pyyaml is required. Install with: pip install pyyaml")

HERE = Path(__file__).resolve().parent
# The TF YAML lives in /config/ (top-level), per the consolidation that
# happened after the recording/ module was added. Earlier versions of
# the repo had it under "ROS2 config/" — both paths are checked.
_REPO = HERE.parent.parent
_CANDIDATES = [
    _REPO / "config" / "sensor_dome_tf.yaml",
    _REPO / "ROS2 config" / "sensor_dome_tf.yaml",
]
TF_YAML = next((p for p in _CANDIDATES if p.exists()), _CANDIDATES[0])
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

    # Collect parents (should all be imu_link), children, and joints.
    parents = {tf["frame_id"] for tf in transforms}
    if parents != {"imu_link"}:
        sys.exit(
            f"ERROR: expected every static transform to be parented to "
            f"imu_link; got {sorted(parents)}"
        )

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

    add_link("imu_link")

    for tf in transforms:
        child = tf["child_frame_id"]
        t = tf["translation"]
        r = tf["rotation"]
        roll, pitch, yaw = quat_to_rpy(r["x"], r["y"], r["z"], r["w"])

        add_link(child)

        joint = doc.createElement("joint")
        joint.setAttribute("name", f"imu_to_{child}")
        joint.setAttribute("type", "fixed")

        parent_el = doc.createElement("parent")
        parent_el.setAttribute("link", "imu_link")
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
