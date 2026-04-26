#!/usr/bin/env python3
# =============================================================
# static_tf.launch.py — publish all sensor->IMU TFs from the
# project YAML so Foxglove can render every frame.
#
# Usage (invoked by sensor_recorder.py):
#   ros2 launch sensor_dome static_tf.launch.py \
#       tf_yaml:=/.../ROS2 config/sensor_dome_tf.yaml
#
# Standalone:
#   ros2 launch ./static_tf.launch.py \
#       tf_yaml:=$(realpath ../../ROS2\ config/sensor_dome_tf.yaml)
# =============================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def _build(context, *args, **kwargs):
    path = LaunchConfiguration("tf_yaml").perform(context)
    with open(path, "r") as f:
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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "tf_yaml",
            description="Path to sensor_dome_tf.yaml",
        ),
        OpaqueFunction(function=_build),
    ])
