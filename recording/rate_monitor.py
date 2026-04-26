#!/usr/bin/env python3
# =============================================================
# rate_monitor.py — publish per-topic Hz on diagnostic_msgs.
#
# Subscribes to every topic given on --topics with a generic
# rclpy "any-message" subscription, computes the moving-average
# rate over a sliding window, and republishes them on a single
# diagnostic_msgs/DiagnosticArray topic. Foxglove's Diagnostics
# panel then renders the live rates in the recording dashboard.
#
# Run standalone for debugging:
#   python3 rate_monitor.py --topics /imu/data /robin_w_front/points
# =============================================================

from __future__ import annotations

import argparse
import sys
import time
from collections import defaultdict, deque
from typing import Deque, Dict, List

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                           DurabilityPolicy)
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
except ImportError:  # pragma: no cover
    sys.exit("ERROR: rclpy / diagnostic_msgs not available. "
             "Source ROS 2 Jazzy first: source /opt/ros/jazzy/setup.bash")


class RateMonitor(Node):
    """Subscribe to arbitrary topics, publish per-topic Hz."""

    def __init__(self, topics: List[str], window_s: float,
                 rate_topic: str, pub_period_s: float = 0.5):
        super().__init__("sensor_dome_rate_monitor")
        self.window_s = window_s
        self.stamps: Dict[str, Deque[float]] = defaultdict(deque)

        # BEST_EFFORT to match LiDAR / camera publishers; depth=10
        # is enough — we only count arrivals, not buffer messages.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subs = []
        for t in topics:
            # rclpy can't subscribe with type="*" — we resolve the type
            # by looking up the publisher. Wait briefly for it to come
            # up; if no publisher exists yet, retry on each tick.
            self._try_subscribe(t, sensor_qos)

        self.pub = self.create_publisher(
            DiagnosticArray, rate_topic, 10)
        self.create_timer(pub_period_s, self._publish_rates)
        self.create_timer(2.0, self._retry_pending_subs)
        self._pending = list(topics)

    # -- subscription helpers --
    def _try_subscribe(self, topic: str, qos: QoSProfile) -> bool:
        infos = self.get_publishers_info_by_topic(topic)
        if not infos:
            return False
        type_name = infos[0].topic_type           # e.g. "sensor_msgs/msg/Imu"
        try:
            msg_class = _import_msg(type_name)
        except Exception as ex:
            self.get_logger().warn(f"can't import {type_name}: {ex}")
            return False
        self.subs.append(self.create_subscription(
            msg_class, topic,
            lambda _msg, t=topic: self._on_msg(t),
            qos))
        if topic in self._pending:
            self._pending.remove(topic)
        self.get_logger().info(f"subscribed to {topic} ({type_name})")
        return True

    def _retry_pending_subs(self) -> None:
        if not self._pending:
            return
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        for t in list(self._pending):
            self._try_subscribe(t, sensor_qos)

    # -- accounting --
    def _on_msg(self, topic: str) -> None:
        now = time.monotonic()
        dq = self.stamps[topic]
        dq.append(now)
        cutoff = now - self.window_s
        while dq and dq[0] < cutoff:
            dq.popleft()

    def _publish_rates(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        now = time.monotonic()

        for topic, dq in self.stamps.items():
            cutoff = now - self.window_s
            while dq and dq[0] < cutoff:
                dq.popleft()
            hz = len(dq) / self.window_s if self.window_s > 0 else 0.0

            status = DiagnosticStatus()
            status.name = f"sensor_dome/rate/{topic.lstrip('/')}"
            status.hardware_id = topic
            status.message = f"{hz:.2f} Hz"
            if hz <= 0.01:
                status.level = DiagnosticStatus.ERROR
            elif hz < self._expected_min(topic):
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.OK
            status.values = [
                KeyValue(key="rate_hz", value=f"{hz:.2f}"),
                KeyValue(key="window_s", value=f"{self.window_s:.1f}"),
                KeyValue(key="count_in_window", value=str(len(dq))),
            ]
            msg.status.append(status)

        self.pub.publish(msg)

    @staticmethod
    def _expected_min(topic: str) -> float:
        # Coarse expectations matching PTP_sync/README.md §5.2.
        if "/imu" in topic:        return 150.0    # IMU @ 200 Hz
        if "points" in topic:      return 8.0      # Robin W @ 10 Hz
        if "image_raw" in topic:   return 15.0     # RouteCAM @ 20 Hz
        if "gps" in topic or "fix" in topic: return 5.0
        if "pose" in topic:        return 5.0
        return 0.5


def _import_msg(type_name: str):
    """Import a ROS 2 message class from its 'pkg/msg/Name' string."""
    parts = type_name.split("/")
    if len(parts) == 3:
        pkg, _, name = parts
    elif len(parts) == 2:
        pkg, name = parts
    else:
        raise ValueError(f"unexpected type name {type_name!r}")
    mod = __import__(f"{pkg}.msg", fromlist=[name])
    return getattr(mod, name)


def main(argv=None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--topics", nargs="+", required=True,
                   help="Topics to monitor")
    p.add_argument("--window", type=float, default=2.0,
                   help="Sliding window in seconds")
    p.add_argument("--rate-topic", default="/sensor_dome/rates",
                   help="Topic to publish DiagnosticArray on")
    args = p.parse_args(argv)

    rclpy.init()
    node = RateMonitor(args.topics, args.window, args.rate_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
