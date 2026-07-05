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
#   3. NO GLIM node. GLIM maps OFFLINE ONLY (enable_online_mapping=false
#      in glim/config/config_ros.json makes glim_rosnode refuse to run).
#      Record the session (recording/), then build the map with:
#        ros2 run glim_ros glim_rosbag <bag> --ros-args -p dump_path:=<out>
#      glim_rosbag feeds /imu/data, /robin_w_*/points, /pose, and /gps/fix
#      from the bag into the same callbacks the live path would use, so
#      INS init and the RTK-gated GNSS factor bridge behave identically.
#      A launch-time diagnostic still runs dual-antenna detection and the
#      orientation-prior consistency check (see _build_glim_node).
#
#   4. C++-side INS initialization (Hitch fork). In offline replay the
#      first PoseStamped on ins_pose_topic (config_ros.json, default
#      /pose) that passes the RTK gate is forwarded to
#      OdometryEstimationIMU::set_init_state(), which pins
#      NaiveInitialStateEstimation's force_init pathway. GLIM's
#      gravity-from-accelerometer calibration is NEVER invoked in this
#      fork — it has been removed from initial_state_estimation.cpp.
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

import json
import re
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
DEFAULT_GNSS_CONFIG = GLIM_DIR / "glim_ext" / "config" / "config_gnss_global.json"
SCRIPTS_DIR = GLIM_DIR / "scripts"

# Bold-yellow ANSI escape — used for consistency-mismatch warnings.
_YELLOW_BOLD = "\033[1;33m"
_ANSI_RESET = "\033[0m"


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


def _read_orientation_prior_flag(gnss_config_path: Path):
    """Parse GLIM_plusplus/glim_ext/config/config_gnss_global.json (JSONC
    format — has `// …` comments) and return the value of
    gnss.enable_orientation_prior. Returns None if the file cannot be
    read or the key is absent, so callers can distinguish "not present"
    from "present and false".

    GLIM's config_gnss_global.json carries inline comments, so we strip
    everything after `//` on each line before json.loads. Block /* */
    comments are not used in this file but are stripped for safety."""
    try:
        raw = gnss_config_path.read_text(encoding="utf-8")
        # Strip block comments first (rare in this file, but defensive).
        no_block = re.sub(r"/\*.*?\*/", "", raw, flags=re.DOTALL)
        # Strip single-line comments — careful not to eat // inside strings.
        # The config file does not contain // inside any string, so a
        # simple line-wise strip is safe.
        cleaned_lines = []
        for line in no_block.splitlines():
            # Find the first // not inside a string.
            in_str = False
            cut = None
            i = 0
            while i < len(line):
                c = line[i]
                if c == '"' and (i == 0 or line[i - 1] != "\\"):
                    in_str = not in_str
                elif not in_str and c == "/" and i + 1 < len(line) and line[i + 1] == "/":
                    cut = i
                    break
                i += 1
            cleaned_lines.append(line if cut is None else line[:cut])
        parsed = json.loads("\n".join(cleaned_lines))
        gnss_block = parsed.get("gnss", {})
        if "enable_orientation_prior" in gnss_block:
            return bool(gnss_block["enable_orientation_prior"])
        return None
    except Exception as ex:
        sys.stderr.write(
            f"[launch] could not parse {gnss_config_path}: {ex} — skipping "
            f"orientation-prior consistency check\n")
        return None


def _warn_orientation_prior_mismatch(dual_enabled: bool, prior_on,
                                     gnss_config_path: Path) -> None:
    """Print a bold-yellow warning when the operator-side configuration
    of dual-antenna mode disagrees with config_gnss_global.json's
    enable_orientation_prior. Two failure modes are flagged:

      (A) Dual-antenna detected but enable_orientation_prior = false
          — the operator is leaving an RTK-fixed yaw constraint on the
          floor. The system still works (it just runs as if it were
          single-antenna), but the dual-antenna heading is wasted.

      (B) Single-antenna detected but enable_orientation_prior = true
          — the bridge will publish a tight yaw covariance on a
          gyro-integrated heading, and the optimizer will be pulled
          toward a drifting reference. This is the failure mode the
          gating exists to prevent.

    This check cannot reach into the Atlas Duo firmware to verify that
    dual-antenna mode is actually programmed there; the runtime yaw σ
    sanity check in the C++ bridge catches that case post-launch."""
    if prior_on is None:
        return  # File missing / unparseable — already warned.
    if dual_enabled and not prior_on:
        sys.stderr.write(
            f"{_YELLOW_BOLD}[launch] WARNING — orientation-prior config "
            f"MISMATCH:\n"
            f"  sensor_dome_tf.yaml says DUAL-antenna mode is enabled,\n"
            f"  but {gnss_config_path.name} has "
            f"enable_orientation_prior: false.\n"
            f"  The dome will run as if single-antenna and waste the RTK "
            f"heading.\n"
            f"  Fix: flip enable_orientation_prior to true in\n"
            f"       {gnss_config_path}\n"
            f"{_ANSI_RESET}")
    elif not dual_enabled and prior_on:
        sys.stderr.write(
            f"{_YELLOW_BOLD}[launch] WARNING — orientation-prior config "
            f"MISMATCH:\n"
            f"  sensor_dome_tf.yaml says SINGLE-antenna mode "
            f"(secondary translation is sentinel),\n"
            f"  but {gnss_config_path.name} has "
            f"enable_orientation_prior: true.\n"
            f"  GLIM++ will inject a yaw factor sourced from "
            f"gyro-integrated INS heading,\n"
            f"  which drifts over the session and degrades map quality.\n"
            f"  Fix: either set the secondary antenna's translation in "
            f"sensor_dome_tf.yaml,\n"
            f"       or flip enable_orientation_prior to false in\n"
            f"       {gnss_config_path}\n"
            f"{_ANSI_RESET}")


def _detect_dual_antenna(tf_yaml_path: str) -> tuple:
    """Read sensor_dome_tf.yaml and decide whether dual-antenna heading
    is configured. Returns (enabled: bool, baseline_m: float).

    Sentinel: the secondary antenna's translation is (0, 0, 0) by default
    in the YAML; any norm >= DUAL_BASELINE_THRESH (5 cm) is treated as a
    real installation."""
    DUAL_BASELINE_THRESH = 0.05  # m
    try:
        with open(tf_yaml_path, "r") as f:
            cfg = yaml.safe_load(f)
        primary = next(
            (tf for tf in cfg.get("static_transforms", [])
             if tf["child_frame_id"] == "gnss_antenna_primary_link"), None)
        secondary = next(
            (tf for tf in cfg.get("static_transforms", [])
             if tf["child_frame_id"] == "gnss_antenna_secondary_link"), None)
        if primary is None or secondary is None:
            return (False, 0.0)
        st = secondary["translation"]
        sec_norm = (st["x"] ** 2 + st["y"] ** 2 + st["z"] ** 2) ** 0.5
        if sec_norm < DUAL_BASELINE_THRESH:
            return (False, 0.0)
        pt = primary["translation"]
        bv = (st["x"] - pt["x"], st["y"] - pt["y"], st["z"] - pt["z"])
        bl = (sum(c * c for c in bv)) ** 0.5
        return (True, bl)
    except Exception as ex:
        sys.stderr.write(
            f"[launch] dual-antenna detection failed: {ex} — defaulting to single-antenna mode\n")
        return (False, 0.0)


def _build_glim_node(context, *args, **kwargs):
    """GLIM maps OFFLINE ONLY — no glim_rosnode is spawned (B4 fix).

    glim_rosnode hard-exits when glim_ros/enable_online_mapping is false
    (the shipped default), so spawning it here only produced a dead node
    at launch. This launch now provides the live-session support pieces
    (static TFs, pre-flight check, foxglove); the map is built afterwards
    with:

        ros2 run glim_ros glim_rosbag <bag> --ros-args -p dump_path:=<out>

    glim_rosbag reads the INS init-gate / GNSS-factor-bridge settings from
    glim/config/config_ros.json ("glim_ros" section) and feeds /pose +
    /gps/fix from the bag itself. The ins_* / gnss_factor_* launch
    arguments are NOT forwarded to GLIM (they never were — the C++ reads
    config_ros.json, not ROS parameters); they are kept only as documented
    defaults. Edit config_ros.json to change gate thresholds or topics.

    What remains here is the dual-antenna detection + config consistency
    check, kept as a launch-time diagnostic."""
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    tf_yaml = cfg("tf_yaml")
    dual_enabled, baseline = _detect_dual_antenna(tf_yaml)

    if dual_enabled:
        # Heading σ ≈ 1 cm RTK noise / baseline, in radians.
        heading_sigma_rad = 0.01 / max(baseline, 0.05)
        sys.stderr.write(
            f"[launch] dual-antenna mode detected in sensor_dome_tf.yaml "
            f"(baseline = {baseline:.3f} m, heading σ ≈ {heading_sigma_rad:.4f} rad).\n"
            f"[launch]   Mirror this in glim/config/config_ros.json (glim_ros section):\n"
            f"[launch]     \"dual_antenna_enabled\": true,\n"
            f"[launch]     \"dual_antenna_baseline_m\": {baseline:.3f},\n"
            f"[launch]     \"dual_antenna_heading_sigma_rad\": {heading_sigma_rad:.4f}\n"
            f"[launch]   so offline mapping (glim_rosbag) picks it up.\n")
    else:
        sys.stderr.write(
            "[launch] single-antenna mode (no dual-antenna baseline configured)\n")

    # Consistency check between sensor_dome_tf.yaml (antenna geometry)
    # and config_gnss_global.json (enable_orientation_prior). Warns
    # loudly on mismatch in either direction; cannot detect Atlas-
    # firmware-side misconfiguration (covered by the runtime yaw σ
    # check in the C++ bridge).
    gnss_config_path = Path(cfg("gnss_config_path"))
    prior_on = _read_orientation_prior_flag(gnss_config_path)
    _warn_orientation_prior_mismatch(dual_enabled, prior_on, gnss_config_path)

    sys.stderr.write(
        "[launch] GLIM maps OFFLINE only — no glim_rosnode spawned. After the "
        "session, build the map with:\n"
        "[launch]   ros2 run glim_ros glim_rosbag <bag> --ros-args -p dump_path:=<out_dir>\n")

    return []


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
            "gnss_config_path",
            default_value=str(DEFAULT_GNSS_CONFIG),
            description="Path to config_gnss_global.json. Read at launch "
                        "time only to run the orientation-prior consistency "
                        "check against sensor_dome_tf.yaml; the C++ side "
                        "reads this file itself via GlobalConfigExt.",
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
