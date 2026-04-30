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
#   2. (optional, opt-in) Pre-flight stationarity check on /imu/data —
#      bold RED CLI warning if the bag/stream starts with the vehicle
#      in motion. Pure diagnostic; the actual fix is in the C++ side
#      (see #4 below). Set skip_stationarity_check:=true to suppress.
#
#   3. The glim_rosnode binary, configured via:
#        --ros-args -p config_path:=GLIM_plusplus/glim/config
#                   -p ins_pose_topic:=/pose
#      so all the tuning in GLIM_plusplus/glim/config/*.json and
#      GLIM_plusplus/glim_ext/config/*.json applies and the C++ subscribes to the
#      Atlas Duo's INS pose for initialization.
#
#   4. C++-side INS initialization (Hitch fork). The glim_ros2 wrapper
#      subscribes to ins_pose_topic (default /pose) at startup. The first
#      valid PoseStamped is forwarded to OdometryEstimationIMU::set_init_state(),
#      which pins NaiveInitialStateEstimation's force_init pathway.
#      GLIM's gravity-from-accelerometer calibration is NEVER invoked in
#      this fork — it has been removed from initial_state_estimation.cpp.
#      See GLIM_plusplus/docs/moving_start_initialization.md for the full design.
#
#   5. Optional foxglove_bridge for live visualization
#      (set foxglove:=false to skip).
#
# The map is built body-relative to imu_link. Each vehicle that wants a
# base_link-anchored map publishes its own static "imu_link → base_link"
# transform downstream.
#
# Usage:
#   ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py
#   ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py foxglove:=false
#   ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py \
#                            ins_pose_topic:=/atlas/pose
# =============================================================================

import subprocess
import sys
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Repository layout assumptions (this file lives at GLIM_plusplus/launch/).
HERE = Path(__file__).resolve().parent
GLIM_DIR = HERE.parent                          # GLIM_plusplus/
REPO_ROOT = GLIM_DIR.parent                     # Hitch_Sensor_Dome/
DEFAULT_TF_YAML = REPO_ROOT / "config" / "sensor_dome_tf.yaml"
DEFAULT_GLIM_CONFIG = GLIM_DIR / "glim" / "config"
SCRIPTS_DIR = GLIM_DIR / "scripts"


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


def _maybe_pre_flight(context, *args, **kwargs):
    """Run the pre-flight stationarity check synchronously before GLIM
    starts. Prints a bold RED warning if the first 3 s of /imu/data is
    non-stationary — informational only in this fork (the C++ INS-init
    pathway handles moving starts correctly), but useful for debugging
    Atlas Duo lock issues.

    Disabled with skip_stationarity_check:=true."""
    if LaunchConfiguration("skip_stationarity_check").perform(context).lower() == "true":
        return []
    checker = SCRIPTS_DIR / "check_init_stationarity.py"
    if not checker.exists():
        sys.stderr.write(f"[launch] missing {checker}, skipping stationarity check\n")
        return []
    try:
        subprocess.run([sys.executable, str(checker), "--live", "--window", "3.0"],
                       check=False)
    except Exception as ex:
        sys.stderr.write(f"[launch] stationarity check failed to run: {ex}\n")
    return []


def _build_glim_node(context, *args, **kwargs):
    """Spawn glim_rosnode with the INS topic + RTK-gate parameters.

    The C++ wrapper subscribes to ins_pose_topic / ins_odom_topic and
    consults ins_fix_topic (NavSatFix) as the RTK gate. Only after the
    gate passes (RTK fix + cm-grade covariance + N consecutive stable
    poses) does it call set_init_state and unblock the optimizer."""
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    return [
        Node(
            package="glim_ros",
            executable="glim_rosnode",
            name="glim_rosnode",
            output="screen",
            parameters=[{
                "config_path":                  cfg("config_path"),
                # Topics
                "ins_pose_topic":               cfg("ins_pose_topic"),
                "ins_odom_topic":               cfg("ins_odom_topic"),
                "ins_fix_topic":                cfg("ins_fix_topic"),
                # Gate thresholds
                "ins_require_rtk_fixed":        cfg("ins_require_rtk_fixed").lower() == "true",
                "ins_max_position_stddev":      float(cfg("ins_max_position_stddev")),
                "ins_min_pose_window_samples":  int(cfg("ins_min_pose_window_samples")),
                "ins_max_pose_jitter_trans":    float(cfg("ins_max_pose_jitter_trans")),
                "ins_min_quat_dot":             float(cfg("ins_min_quat_dot")),
                "ins_init_timeout_s":           float(cfg("ins_init_timeout_s")),
                # GNSS factor bridge (RTK-gated, post-init)
                "gnss_factor_topic":               cfg("gnss_factor_topic"),
                "gnss_factor_require_rtk_fixed":   cfg("gnss_factor_require_rtk_fixed").lower() == "true",
                "gnss_factor_max_position_stddev": float(cfg("gnss_factor_max_position_stddev")),
            }],
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
            description="Path to GLIM_plusplus/glim/config (folder containing config_*.json).",
        ),
        DeclareLaunchArgument(
            "ins_pose_topic",
            default_value="/pose",
            description="INS PoseStamped topic for GLIM's external init "
                        "(Atlas Duo /pose by default). Set to '' to disable "
                        "and rely on ins_odom_topic instead.",
        ),
        DeclareLaunchArgument(
            "ins_odom_topic",
            default_value="",
            description="INS Odometry topic for GLIM's external init. "
                        "Provides linear velocity in addition to orientation. "
                        "Empty by default (use ins_pose_topic).",
        ),
        DeclareLaunchArgument(
            "ins_fix_topic",
            default_value="/gps/fix",
            description="NavSatFix topic used as the RTK gate signal. "
                        "GLIM rejects all INS poses until the most recent "
                        "fix here shows RTK-class status and "
                        "covariance below ins_max_position_stddev.",
        ),
        DeclareLaunchArgument(
            "ins_require_rtk_fixed",
            default_value="true",
            description="If true, require NavSatFix.status >= GBAS_FIX "
                        "(RTK-class) before accepting an INS pose. Set to "
                        "false to allow SBAS / single-point starts (degraded; "
                        "moving-start fix is no longer guaranteed).",
        ),
        DeclareLaunchArgument(
            "ins_max_position_stddev",
            default_value="0.10",
            description="Reject poses whose NavSatFix position covariance "
                        "diagonal stddev exceeds this (m). 0.10 m matches "
                        "RTK-fixed; raise to 0.5 for SBAS-class.",
        ),
        DeclareLaunchArgument(
            "ins_min_pose_window_samples",
            default_value="10",
            description="Require this many consecutive consistent INS pose "
                        "messages before accepting init.",
        ),
        DeclareLaunchArgument(
            "ins_max_pose_jitter_trans",
            default_value="0.05",
            description="Max translation drift (m) between consecutive INS "
                        "pose samples in the stability window.",
        ),
        DeclareLaunchArgument(
            "ins_min_quat_dot",
            default_value="0.999",
            description="Min |q1·q2| between consecutive INS pose samples in "
                        "the stability window. 0.999 ≈ <2.5° drift.",
        ),
        DeclareLaunchArgument(
            "ins_init_timeout_s",
            default_value="60.0",
            description="Seconds to wait for RTK fix before promoting the "
                        "bold RED warning to TIMEOUT messaging. GLIM never "
                        "auto-aborts; the operator decides whether to wait, "
                        "relax thresholds, or kill the process.",
        ),
        DeclareLaunchArgument(
            "gnss_factor_topic",
            default_value="/gnss/pose_rtk_only",
            description="Topic on which the wrapper republishes RTK-fixed "
                        "INS poses as PoseWithCovarianceStamped. "
                        "libgnss_global.so subscribes here. Set to '' to "
                        "disable the bridge entirely.",
        ),
        DeclareLaunchArgument(
            "gnss_factor_require_rtk_fixed",
            default_value="true",
            description="If true, only republish poses while the latest "
                        "NavSatFix shows RTK-class status (GBAS_FIX). "
                        "Set false to also include RTK-float (degraded; "
                        "z-anchor will be looser).",
        ),
        DeclareLaunchArgument(
            "gnss_factor_max_position_stddev",
            default_value="0.10",
            description="Reject poses for the factor stream if "
                        "NavSatFix.position_covariance σ exceeds this (m). "
                        "Defaults match the init gate; matches the "
                        "RTK-fixed regime.",
        ),
        DeclareLaunchArgument(
            "foxglove",
            default_value="true",
            description="Start foxglove_bridge alongside GLIM (true/false).",
        ),
        DeclareLaunchArgument(
            "skip_stationarity_check",
            default_value="false",
            description="If true, skip the bold RED pre-flight warning "
                        "when the bag starts in motion. Default false "
                        "(always run as a diagnostic).",
        ),
        # Pre-flight diagnostic (informational; the C++ init handles moving starts).
        OpaqueFunction(function=_maybe_pre_flight),
        # Live nodes.
        OpaqueFunction(function=_build_static_tfs),
        OpaqueFunction(function=_build_glim_node),
        OpaqueFunction(function=_maybe_foxglove),
    ])
