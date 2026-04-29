#!/usr/bin/env python3
# =============================================================================
# hitch_sensor_dome.launch.py
#
# Launch GLIM against the Hitch Sensor Dome. Wires up:
#
#   1. Static TF publishers from config/sensor_dome_tf.yaml so every
#      "imu_link → lidar_*_link / cam_*_link" transform is on the wire.
#      GLIM uses these (via sensor_dome.urdf) to align the rear-left and
#      rear-right Robin W clouds into the front-lidar primary frame.
#
#   2. The glim_rosnode binary, configured via:
#        --ros-args -p config_path:=GLIM/config
#      so all the tuning in GLIM/glim/config/*.json and
#      GLIM/glim_ext/config/*.json applies.
#
#   3. Optional foxglove_bridge for live visualization
#      (set foxglove:=false to skip).
#
# The map is built body-relative to imu_link. Each vehicle that wants a
# base_link-anchored map publishes its own static "imu_link → base_link"
# transform downstream.
#
# Usage:
#   ros2 launch GLIM/launch/hitch_sensor_dome.launch.py
#   ros2 launch GLIM/launch/hitch_sensor_dome.launch.py foxglove:=false
#   ros2 launch GLIM/launch/hitch_sensor_dome.launch.py \
#                            tf_yaml:=/abs/path/to/sensor_dome_tf.yaml
# =============================================================================

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory  # noqa: F401

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Repository layout assumptions (this file lives at GLIM/launch/).
HERE = Path(__file__).resolve().parent
GLIM_DIR = HERE.parent                          # GLIM/
REPO_ROOT = GLIM_DIR.parent                     # Hitch_Sensor_Dome/
DEFAULT_TF_YAML = REPO_ROOT / "config" / "sensor_dome_tf.yaml"
DEFAULT_GLIM_CONFIG = GLIM_DIR / "glim" / "config"


def _build_static_tfs(context, *args, **kwargs):
    """Read sensor_dome_tf.yaml and emit one static_transform_publisher
    per entry. The launch-time YAML read keeps the URDF and the live TF
    tree in sync without a separate publisher process."""
    tf_yaml_path = LaunchConfiguration("tf_yaml").perform(context)
    with open(tf_yaml_path, "r") as f:
        config = yaml.safe_load(f)

    nodes = []
    for tf in config.get("static_transforms", []):
        t = tf["translation"]
        r = tf["rotation"]
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_tf_{tf['child_frame_id']}",
            arguments=[
                "--x", str(t["x"]),
                "--y", str(t["y"]),
                "--z", str(t["z"]),
                "--qx", str(r["x"]),
                "--qy", str(r["y"]),
                "--qz", str(r["z"]),
                "--qw", str(r["w"]),
                "--frame-id", tf["frame_id"],
                "--child-frame-id", tf["child_frame_id"],
            ],
        ))
    return nodes


def _build_glim_node(context, *args, **kwargs):
    """Spawn the glim_rosnode with the project's config directory."""
    config_path = LaunchConfiguration("config_path").perform(context)
    return [
        Node(
            package="glim_ros",
            executable="glim_rosnode",
            name="glim_rosnode",
            output="screen",
            parameters=[{"config_path": config_path}],
        )
    ]


def _maybe_foxglove(context, *args, **kwargs):
    if LaunchConfiguration("foxglove").perform(context).lower() != "true":
        return []
    return [
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{"port": 8765}],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "tf_yaml",
            default_value=str(DEFAULT_TF_YAML),
            description="Path to config/sensor_dome_tf.yaml",
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value=str(DEFAULT_GLIM_CONFIG),
            description="Path to GLIM/glim/config (folder containing config_*.json)",
        ),
        DeclareLaunchArgument(
            "foxglove",
            default_value="true",
            description="Start foxglove_bridge alongside GLIM (true/false)",
        ),
        OpaqueFunction(function=_build_static_tfs),
        OpaqueFunction(function=_build_glim_node),
        OpaqueFunction(function=_maybe_foxglove),
    ])
