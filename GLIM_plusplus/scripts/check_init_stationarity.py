#!/usr/bin/env python3
# =============================================================================
# check_init_stationarity.py — pre-flight stationarity check for GLIM init
#
# Upstream GLIM's IMU initialization (LOOSE / NAIVE) assumes the vehicle is
# stationary for the first `initialization_window_size` seconds so it can
# extract the gravity vector from the accelerometer's mean reading. If the
# bag (or live recording) starts with the vehicle already moving, the
# integrated linear acceleration leaks into the gravity estimate, producing
# a tilted reference frame that biases everything downstream — including
# the "second-lap-tilts-to-the-sky" failure mode.
#
# In this fork the C++ wrapper (glim_ros2/src/glim_ros/glim_ros.cpp) bypasses
# that entirely by subscribing to the Atlas Duo INS pose with an RTK-fixed
# gate before init. So a non-stationary start no longer breaks GLIM. This
# tool is preserved as a *diagnostic*: it tells you whether the input data
# is stationary, which is useful when investigating slow Atlas Duo lock,
# bag-trim correctness, or unexpected init delays.
#
# It reads the first N seconds of /imu/data from a rosbag (or live topic),
# computes |‖a‖ − 9.81| and ‖ω‖ statistics, and prints a bold RED warning
# if the data is non-stationary.  See GLIM_plusplus/docs/moving_start_initialization.md
# for the full design of the C++-side init.
#
# Usage:
#   # Live mode (subscribe for 3 s)
#   python3 check_init_stationarity.py --live
#
#   # Bag mode (read first 3 s of /imu/data from an MCAP)
#   python3 check_init_stationarity.py --bag recording/data/session_<ts>/rosbag2
#
#   # Strict: exit 2 if non-stationary (CI / launch gating)
#   python3 check_init_stationarity.py --bag <path> --strict
# =============================================================================

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Iterable, Tuple

GRAVITY = 9.80665   # m/s², standard gravity


# ANSI: bold red on terminals, plain text elsewhere.
def _ansi(code: str) -> str:
    return code if sys.stderr.isatty() else ""


BOLD_RED = _ansi("\033[1;31m")
BOLD_YELLOW = _ansi("\033[1;33m")
BOLD_GREEN = _ansi("\033[1;32m")
RESET = _ansi("\033[0m")


# ---------------------------------------------------------------------------
# Statistics — what counts as "stationary"
# ---------------------------------------------------------------------------

# Default thresholds. Tuned for the Atlas Duo (Bosch BMI088 class IMU).
# `acc_dev`: |‖a‖ − g| stays this small while still on the ground at idle.
# `gyro`:    ‖ω‖ stays this small while parked, including engine vibration.
DEFAULT_ACC_DEV_THRESH = 0.5    # m/s²  (deviation from 9.81)
DEFAULT_GYRO_THRESH    = 0.10   # rad/s
DEFAULT_WINDOW_S       = 3.0


def _classify(samples: Iterable[Tuple[float, float, float, float, float, float]],
              acc_thresh: float,
              gyro_thresh: float) -> dict:
    """Aggregate stats over a window of (ax, ay, az, gx, gy, gz)."""
    acc_norms, gyro_norms = [], []
    for ax, ay, az, gx, gy, gz in samples:
        acc_norms.append(math.sqrt(ax * ax + ay * ay + az * az))
        gyro_norms.append(math.sqrt(gx * gx + gy * gy + gz * gz))
    if not acc_norms:
        return {"count": 0}
    acc_mean = statistics.fmean(acc_norms)
    acc_max = max(acc_norms)
    gyro_mean = statistics.fmean(gyro_norms)
    gyro_max = max(gyro_norms)
    acc_dev = abs(acc_mean - GRAVITY)
    moving = acc_dev > acc_thresh or gyro_max > gyro_thresh
    return {
        "count":       len(acc_norms),
        "acc_mean":    acc_mean,
        "acc_max":     acc_max,
        "acc_dev":     acc_dev,
        "gyro_mean":   gyro_mean,
        "gyro_max":    gyro_max,
        "moving":      moving,
        "acc_thresh":  acc_thresh,
        "gyro_thresh": gyro_thresh,
    }


# ---------------------------------------------------------------------------
# Sample-source backends
# ---------------------------------------------------------------------------

def _samples_from_bag(bag_path: Path,
                      topic: str,
                      window_s: float) -> Iterable[Tuple[float, ...]]:
    """Yield (ax, ay, az, gx, gy, gz) tuples from the first window_s seconds of
    `topic` in `bag_path`. Supports MCAP via rosbag2_py."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu
    except ImportError as ex:
        sys.exit(f"ERROR: rosbag2_py / rclpy / sensor_msgs missing: {ex}\n"
                 "       Source ROS 2 first: source /opt/ros/jazzy/setup.bash")

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Filter by topic
    storage_filter = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(storage_filter)

    t0 = None
    while reader.has_next():
        topic_name, data, t_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg: Imu = deserialize_message(data, Imu)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t0 is None:
            t0 = t
        if t - t0 > window_s:
            break
        yield (msg.linear_acceleration.x,
               msg.linear_acceleration.y,
               msg.linear_acceleration.z,
               msg.angular_velocity.x,
               msg.angular_velocity.y,
               msg.angular_velocity.z)


def _samples_from_live(topic: str, window_s: float) -> Iterable[Tuple[float, ...]]:
    """Subscribe to `topic` for window_s seconds and yield IMU samples."""
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Imu
    except ImportError as ex:
        sys.exit(f"ERROR: rclpy / sensor_msgs missing: {ex}\n"
                 "       Source ROS 2 first: source /opt/ros/jazzy/setup.bash")

    rclpy.init()
    node = rclpy.create_node("glim_init_stationarity_check")
    samples: list[Tuple[float, ...]] = []

    def on_imu(msg):
        samples.append((msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z,
                        msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z))

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=200)
    node.create_subscription(Imu, topic, on_imu, qos)
    end = time.monotonic() + window_s
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    yield from samples


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def _report(stats: dict) -> None:
    if stats["count"] == 0:
        print(f"{BOLD_RED}[ERR]{RESET}  no IMU samples received in window — "
              "is the topic correct? is the bag valid?", file=sys.stderr)
        return

    if not stats["moving"]:
        print(f"{BOLD_GREEN}[ OK]{RESET}  initialization window appears "
              f"stationary ({stats['count']} samples)", file=sys.stderr)
        print(f"        |‖a‖ − g| = {stats['acc_dev']:.3f} m/s²  "
              f"(< {stats['acc_thresh']} threshold)", file=sys.stderr)
        print(f"        ‖ω‖ max   = {stats['gyro_max']:.3f} rad/s  "
              f"(< {stats['gyro_thresh']} threshold)", file=sys.stderr)
        return

    # Non-stationary — bold RED warning, multi-line, with remediation.
    bar = "=" * 78
    print(f"{BOLD_RED}{bar}{RESET}", file=sys.stderr)
    print(f"{BOLD_RED}  ⚠  GLIM INIT WARNING: VEHICLE IS NOT STATIONARY  ⚠{RESET}",
          file=sys.stderr)
    print(f"{BOLD_RED}{bar}{RESET}", file=sys.stderr)
    print(file=sys.stderr)
    print(f"  Stationarity check over {stats['count']} samples:", file=sys.stderr)
    print(f"    |‖a‖ − g|  mean = {stats['acc_dev']:.3f} m/s²"
          f"   (threshold {stats['acc_thresh']})", file=sys.stderr)
    print(f"    ‖a‖        max  = {stats['acc_max']:.3f} m/s²", file=sys.stderr)
    print(f"    ‖ω‖        max  = {stats['gyro_max']:.3f} rad/s"
          f"  (threshold {stats['gyro_thresh']})", file=sys.stderr)
    print(file=sys.stderr)
    print(f"  {BOLD_YELLOW}Why this matters{RESET}", file=sys.stderr)
    print("    GLIM's default LOOSE / NAIVE initialization assumes the IMU is",
          file=sys.stderr)
    print("    stationary during the first few seconds so it can extract gravity",
          file=sys.stderr)
    print("    from the accelerometer mean. With the vehicle already moving, the",
          file=sys.stderr)
    print("    integrated linear acceleration leaks into the gravity estimate,",
          file=sys.stderr)
    print("    producing a tilted world frame. Symptoms downstream: the",
          file=sys.stderr)
    print("    trajectory drifts up into the sky, especially on lap 2+.",
          file=sys.stderr)
    print(file=sys.stderr)
    print(f"  {BOLD_YELLOW}Already mitigated in this fork{RESET}", file=sys.stderr)
    print("    The Hitch fork's C++ wrapper subscribes to the Atlas Duo's", file=sys.stderr)
    print("    INS pose (with RTK-fixed gating) and pins init_T_world_imu", file=sys.stderr)
    print("    BEFORE the optimizer starts, so a non-stationary start no", file=sys.stderr)
    print("    longer corrupts gravity. This warning is informational only.", file=sys.stderr)
    print("    Launch normally:", file=sys.stderr)
    print("       ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py", file=sys.stderr)
    print(file=sys.stderr)
    print(f"  {BOLD_YELLOW}If you'd rather avoid the dependency on RTK at start{RESET}",
          file=sys.stderr)
    print("    1. Trim the bag to start from a stationary segment", file=sys.stderr)
    print("       (use scripts/find_stationary_window.py if available, or",
          file=sys.stderr)
    print("       `ros2 bag play --start-offset` for live replay).",
          file=sys.stderr)
    print("    2. If acceptable, restart the data collection with a 3-second",
          file=sys.stderr)
    print("       stationary pause at the beginning before driving.",
          file=sys.stderr)
    print(f"{BOLD_RED}{bar}{RESET}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Pre-flight stationarity check for GLIM IMU initialization.")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag",  type=Path, help="Path to a rosbag2 directory (MCAP).")
    src.add_argument("--live", action="store_true",
                     help="Subscribe live to --topic for --window seconds.")
    p.add_argument("--topic", default="/imu/data",
                   help="IMU topic name (default: /imu/data)")
    p.add_argument("--window", type=float, default=DEFAULT_WINDOW_S,
                   help=f"Window in seconds (default: {DEFAULT_WINDOW_S})")
    p.add_argument("--acc-dev-thresh", type=float,
                   default=DEFAULT_ACC_DEV_THRESH,
                   help="Max |‖a‖−g| in m/s² (default: %(default)s)")
    p.add_argument("--gyro-thresh", type=float, default=DEFAULT_GYRO_THRESH,
                   help="Max ‖ω‖ in rad/s (default: %(default)s)")
    p.add_argument("--strict", action="store_true",
                   help="Exit non-zero if non-stationary (use in launch / CI).")
    args = p.parse_args(argv)

    if args.bag:
        if not args.bag.exists():
            sys.exit(f"ERROR: bag path not found: {args.bag}")
        samples = list(_samples_from_bag(args.bag, args.topic, args.window))
    else:
        samples = list(_samples_from_live(args.topic, args.window))

    stats = _classify(samples, args.acc_dev_thresh, args.gyro_thresh)
    _report(stats)

    if args.strict and stats.get("moving"):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
