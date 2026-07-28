#!/usr/bin/env python3
# =============================================================
# sensor_recorder.py — Hitch Sensor Dome recording orchestrator
#
# Records GNSS / IMU / LiDAR / camera data to a Foxglove-native
# ROS 2 MCAP bag, with a live foxglove_bridge dashboard for the
# operator (3D point clouds in IMU frame, camera views, sensor
# rates, GNSS map).
#
# Pipeline:
#   1. Load sensor_config.yaml and parse CLI overrides.
#   2. Auto-detect connected sensors (Atlas Duo, Robin W LiDARs,
#      RouteCAMs). Print a checklist and let the user confirm or
#      deselect any of them.
#   3. Spawn every confirmed ROS 2 LiDAR/camera driver once.
#   4. Read and verify chrony state, compare each sensor header
#      directly with host CLOCK_REALTIME, and check the live Atlas
#      adapter streams. Clock configuration remains exclusively in
#      PTP_sync/; this process only observes and fails closed.
#   5. Keep the verified drivers running and start static TF,
#      foxglove_bridge, the rate monitor, and
#      `ros2 bag record -s mcap` on the relevant topics.
#   6. Print live status. H/Q for health / quit.
#   7. On exit, write session_metadata.json and tear everything
#      down cleanly.
#
# Run:
#   python3 sensor_recorder.py                          # YAML defaults
#   python3 sensor_recorder.py --config my.yaml
#   python3 sensor_recorder.py --no-foxglove --headless
#   python3 sensor_recorder.py --dry-run                # full live preflight
#
# =============================================================

from __future__ import annotations

import argparse
import json
import math
import os
import re
import select
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from contextlib import suppress
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("ERROR: pyyaml is required. Install with: sudo apt install python3-yaml")


# ---------------------------------------------------------------------
# Pretty printing helpers (no external deps)
# ---------------------------------------------------------------------

ANSI = {
    "reset":  "\033[0m",
    "bold":   "\033[1m",
    "dim":    "\033[2m",
    "red":    "\033[31m",
    "green":  "\033[32m",
    "yellow": "\033[33m",
    "blue":   "\033[34m",
    "cyan":   "\033[36m",
}


def color(text: str, c: str) -> str:
    if not sys.stdout.isatty():
        return text
    return f"{ANSI.get(c,'')}{text}{ANSI['reset']}"


def banner(msg: str) -> None:
    bar = "=" * max(60, len(msg) + 4)
    print(color(bar, "cyan"))
    print(color(f"  {msg}", "cyan"))
    print(color(bar, "cyan"))


def info(msg: str) -> None:    print(color("[ ok ] ", "green") + msg)
def warn(msg: str) -> None:    print(color("[warn] ", "yellow") + msg)
def err(msg: str)  -> None:    print(color("[ err] ", "red") + msg)
def step(msg: str) -> None:    print(color("\n>>> ", "blue") + color(msg, "bold"))


# ---------------------------------------------------------------------
# Config + dataclasses
# ---------------------------------------------------------------------

@dataclass
class SyncResult:
    name: str
    ok: bool
    detail: str
    offset_ns: Optional[int] = None
    tolerance_ns: Optional[int] = None


@dataclass
class SensorEntry:
    kind: str           # "gnss_imu" | "lidar" | "camera"
    name: str
    cfg: Dict[str, Any]
    detected: bool = False
    detail: str = ""
    enabled: bool = True   # user may deselect


@dataclass
class SessionInfo:
    started_utc: str
    output_dir: str
    storage: str
    ended_utc: str = ""
    success: bool = False
    sensors: List[Dict[str, Any]] = field(default_factory=list)
    sync: List[Dict[str, Any]] = field(default_factory=list)
    requirements: List[Dict[str, Any]] = field(default_factory=list)
    profile: str = ""
    topics: List[str] = field(default_factory=list)
    processes: List[Dict[str, Any]] = field(default_factory=list)
    notes: str = ""


# ---------------------------------------------------------------------
# Sensor auto-detection
# ---------------------------------------------------------------------

class SensorDetector:
    """Probe each configured sensor and report whether it's reachable.

    These probes are intentionally cheap (TCP connect, file stat) so the
    operator can re-run detection quickly if a cable changes.
    """

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg

    # ----- Atlas Duo -----
    def detect_gnss_imu(self) -> SensorEntry:
        c = self.cfg.get("point_one_nav", {})
        entry = SensorEntry("gnss_imu", "point_one_nav_atlas_duo", c)
        if not c.get("enabled", True):
            entry.detail = "disabled in config"
            return entry
        # Atlas Duo is Ethernet-only — probe the FusionEngine TCP port.
        # (Hardware has no BNC PPS pin and no USB serial output.)
        host = c.get("tcp_ip", c.get("tcp_host", "192.168.1.30"))
        port = int(c.get("tcp_port", 30201))
        try:
            with socket.create_connection((host, port), timeout=2):
                entry.detected = True
                entry.detail = f"FusionEngine TCP {host}:{port} open"
        except Exception as e:
            entry.detail = f"{host}:{port} not reachable ({type(e).__name__}: Atlas off or nav engine not started?)"
        # Bonus: confirm gpsd is consuming the NMEA stream if it's running.
        if entry.detected and shutil.which("gpspipe"):
            try:
                r = subprocess.run(
                    ["gpspipe", "-w", "-n", "1"], capture_output=True,
                    text=True, timeout=3,
                )
                if r.returncode == 0 and r.stdout.strip():
                    entry.detail += "; gpsd responding"
            except Exception:
                pass
        return entry

    # ----- Robin W LiDARs -----
    @staticmethod
    def _tcp_probe(host: str, port: int, timeout: float = 1.0) -> bool:
        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except OSError:
            return False

    def detect_lidars(self) -> List[SensorEntry]:
        out: List[SensorEntry] = []
        for c in self.cfg.get("lidars", []):
            e = SensorEntry("lidar", c["name"], c)
            ok = self._tcp_probe(c["ip"], int(c.get("port", 8010)))
            e.detected = ok
            e.detail = (
                f"TCP {c['ip']}:{c.get('port',8010)} "
                + ("reachable" if ok else "no response")
            )
            out.append(e)
        return out

    # ----- RouteCAM cameras -----
    def detect_cameras(self) -> List[SensorEntry]:
        out: List[SensorEntry] = []
        # First try arv-tool-0.8 enumeration (cheap, exact).
        seen_ips: set = set()
        if shutil.which("arv-tool-0.8"):
            try:
                r = subprocess.run(
                    ["arv-tool-0.8"], capture_output=True,
                    text=True, timeout=5,
                )
                # arv-tool prints "Vendor-Model-SN (192.168.1.20)" lines.
                for m in re.finditer(r"\(([0-9.]+)\)", r.stdout):
                    seen_ips.add(m.group(1))
            except Exception:
                pass
        for c in self.cfg.get("cameras", []):
            e = SensorEntry("camera", c["name"], c)
            ip = c["ip"]
            if ip in seen_ips:
                e.detected = True
                e.detail = f"GigE Vision device at {ip} (arv-tool)"
            elif self._tcp_probe(ip, 3956, timeout=1.0):
                # GigE Vision control channel default port.
                e.detected = True
                e.detail = f"GVCP port 3956 reachable at {ip}"
            else:
                # Final fallback: ICMP via /bin/ping, 1 packet.
                e.detected = _ping(ip)
                e.detail = (
                    f"ping {ip} "
                    + ("ok (camera reachable, but PTP/Aravis unverified)"
                       if e.detected else "no response")
                )
            out.append(e)
        return out

    def detect_all(self) -> List[SensorEntry]:
        result: List[SensorEntry] = []
        gi = self.detect_gnss_imu()
        if gi.cfg.get("enabled", True) or gi.detected:
            result.append(gi)
        result.extend(self.detect_lidars())
        result.extend(self.detect_cameras())
        return result


def _ping(host: str, timeout_s: float = 1.0) -> bool:
    if not shutil.which("ping"):
        return False
    try:
        r = subprocess.run(
            ["ping", "-c", "1", "-W", str(int(max(1, timeout_s))), host],
            capture_output=True, text=True, timeout=timeout_s + 1.0,
        )
        return r.returncode == 0
    except Exception:
        return False


# ---------------------------------------------------------------------
# Interactive confirmation
# ---------------------------------------------------------------------

def confirm_sensors(entries: List[SensorEntry], assume_yes: bool) -> List[SensorEntry]:
    banner("Sensor checklist")
    for i, e in enumerate(entries, 1):
        flag = (
            color("[FOUND]   ", "green") if e.detected
            else color("[MISSING] ", "yellow")
        )
        print(f" {i:>2}. {flag}{e.kind:9s} {e.name:25s} {color(e.detail,'dim')}")
    if assume_yes:
        info("--yes given; auto-confirming all detected sensors")
        for e in entries:
            e.enabled = e.detected
        return entries

    print()
    print("Press ENTER to record from all FOUND sensors, or type a")
    print("space-separated list of indices to TOGGLE (e.g. '2 5'):")
    try:
        line = input("> ").strip()
    except EOFError:
        line = ""
    # Default: enable detected, disable missing.
    for e in entries:
        e.enabled = e.detected
    if line:
        for tok in line.split():
            if tok.isdigit():
                idx = int(tok) - 1
                if 0 <= idx < len(entries):
                    if entries[idx].detected:
                        entries[idx].enabled = not entries[idx].enabled
                    else:
                        warn(
                            f"{entries[idx].name} was not detected and cannot "
                            "be enabled; fix connectivity and rerun detection")

    print()
    print(color("Final selection:", "bold"))
    for e in entries:
        mark = color("REC", "green") if e.enabled else color("skip", "dim")
        print(f"  {mark}  {e.kind:9s} {e.name}")
    print()
    return entries


# ---------------------------------------------------------------------
# Clock-sync verification
# ---------------------------------------------------------------------

class SyncVerifier:
    """Verify that the host CLOCK_REALTIME remains GNSS-disciplined.

    SensorTimestampVerifier independently checks each selected sensor against
    this timebase using the ROS messages that will be written to the bag.
    """

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.tol = cfg.get("sync", {})

    # chrony
    def check_chrony(self) -> SyncResult:
        if not shutil.which("chronyc"):
            return SyncResult("chrony", False, "chronyc not installed")
        try:
            r = subprocess.run(["chronyc", "tracking"],
                               capture_output=True, text=True, timeout=3)
        except Exception as ex:
            return SyncResult("chrony", False, f"chronyc failed: {ex}")
        out = r.stdout
        # Parse "System time : 0.000000123 seconds slow of NTP time"
        m = re.search(r"System time\s*:\s*([\d.]+)\s+seconds", out)
        offset_ns: Optional[int] = None
        if m:
            offset_ns = int(float(m.group(1)) * 1e9)
        ref = re.search(r"Reference ID\s*:\s*(\S+)\s*\((\S*)\)", out)
        ref_name = ref.group(2) if ref and ref.group(2) else (
            ref.group(1) if ref else "?"
        )
        tol_ns = int(self.tol.get("chrony_offset_ns", 1000))
        max_age_s = float(self.tol.get("chrony_max_age_s", 30.0))
        ok = offset_ns is not None and offset_ns <= tol_ns
        # Reference must be GPS/PPS/NMEA, not an IP-style NTP server.
        gps_disciplined = bool(ref) and (
            ref_name.upper() in ("GPS", "PPS", "NMEA") or
            ref_name.startswith("PPS") or ref_name.startswith("NMEA")
        )
        if not gps_disciplined:
            ok = False
        ref_age_s: Optional[float] = None
        ref_time = re.search(r"Ref time \(UTC\)\s*:\s*(.+)", out)
        if ref_time:
            value = ref_time.group(1).strip()
            for fmt in ("%a %b %d %H:%M:%S %Y",
                        "%a %b %d %H:%M:%S.%f %Y"):
                try:
                    parsed = datetime.strptime(value, fmt).replace(
                        tzinfo=timezone.utc)
                    ref_age_s = (
                        datetime.now(timezone.utc) - parsed).total_seconds()
                    break
                except ValueError:
                    continue
        fresh = (
            ref_age_s is not None
            and -1.0 <= ref_age_s <= max_age_s
        )
        if not fresh:
            ok = False
        age_text = (
            f"{ref_age_s:.1f} s" if ref_age_s is not None else "unparseable"
        )
        return SyncResult(
            "chrony", ok,
            f"ref={ref_name} offset={offset_ns} ns (tol {tol_ns} ns), "
            f"reference age={age_text} (max {max_age_s:.1f} s)",
            offset_ns=offset_ns, tolerance_ns=tol_ns,
        )


class SensorTimestampVerifier:
    """Measure each selected sensor's ROS timestamp against CLOCK_REALTIME."""

    def __init__(self, cfg: Dict[str, Any], sensors: List[SensorEntry]):
        raw = cfg.get("sync", {}).get("sensor_timestamp", {}) or {}
        if not isinstance(raw, dict):
            raise ValueError("sync.sensor_timestamp must be a mapping")
        self.cfg = raw
        self.sensors = [
            sensor for sensor in sensors
            if sensor.enabled and sensor.kind in ("lidar", "camera")
        ]
        self.capture_window_s = float(raw.get("capture_window_s", 5.0))
        self.min_samples = int(raw.get("min_samples", 10))
        self.max_future_ms = float(raw.get("max_future_ms", 5.0))
        self.default_max_drift_ms = float(raw.get("max_drift_ms", 5.0))
        if self.capture_window_s <= 0.0:
            raise ValueError(
                "sync.sensor_timestamp.capture_window_s must be positive")
        if self.min_samples < 2:
            raise ValueError(
                "sync.sensor_timestamp.min_samples must be at least 2")
        if self.max_future_ms < 0.0 or self.default_max_drift_ms < 0.0:
            raise ValueError(
                "sensor timestamp future/drift tolerances cannot be negative")
        self.samples: Dict[str, List[Tuple[int, int]]] = {
            sensor.name: [] for sensor in self.sensors
        }
        self.collection_error: Optional[str] = None
        self._subscriptions: List[Any] = []

    def _kind_limit(
            self,
            sensor: SensorEntry,
            key: str,
            default: float,
    ) -> float:
        override = sensor.cfg.get(f"timestamp_{key}")
        kind_cfg = self.cfg.get(sensor.kind, {}) or {}
        value = override if override is not None else kind_cfg.get(key, default)
        result = float(value)
        if result < 0.0:
            raise ValueError(
                f"{sensor.name} timestamp {key} cannot be negative")
        return result

    @staticmethod
    def _percentile(sorted_values: List[int], fraction: float) -> int:
        position = (len(sorted_values) - 1) * fraction
        lower = math.floor(position)
        upper = math.ceil(position)
        if lower == upper:
            return sorted_values[lower]
        weight = position - lower
        return int(round(
            sorted_values[lower] * (1.0 - weight)
            + sorted_values[upper] * weight))

    def attach(self, node: Any, point_cloud_type: Any, image_type: Any,
               qos: Any) -> None:
        for sensor in self.sensors:
            msg_type = point_cloud_type if sensor.kind == "lidar" else image_type

            def on_message(msg, name=sensor.name):
                # time.time_ns() is CLOCK_REALTIME on supported Python/Linux.
                receive_ns = time.time_ns()
                stamp_ns = (
                    int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec)
                )
                self.samples[name].append((receive_ns, stamp_ns))

            self._subscriptions.append(node.create_subscription(
                msg_type, sensor.cfg["topic"], on_message, qos))

    def collect(self) -> None:
        """Collect sensor timestamps when no Atlas preflight node is available."""
        try:
            import rclpy
            from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                                   HistoryPolicy)
            from sensor_msgs.msg import Image, PointCloud2
        except ImportError as ex:
            self.collection_error = (
                f"rclpy/sensor_msgs not available: {ex} "
                "(source ROS 2 first)")
            return

        node = None
        initialized = False
        try:
            rclpy.init()
            initialized = True
            node = rclpy.create_node("sensor_timestamp_preflight")
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=50)
            self.attach(node, PointCloud2, Image, sensor_qos)
            end = time.monotonic() + self.capture_window_s
            while time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception as ex:
            self.collection_error = (
                f"sensor timestamp collection failed: {ex}")
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                if initialized and rclpy.ok():
                    rclpy.shutdown()

    def results(self) -> List[SyncResult]:
        return [
            self._sensor_result(sensor, self.samples[sensor.name])
            for sensor in self.sensors
        ]

    def _sensor_result(
            self,
            sensor: SensorEntry,
            samples: List[Tuple[int, int]],
    ) -> SyncResult:
        name = f"timestamp_{sensor.name}"
        if self.collection_error:
            return SyncResult(name, False, self.collection_error)
        if len(samples) < self.min_samples:
            return SyncResult(
                name, False,
                f"{len(samples)} samples on {sensor.cfg['topic']}; "
                f"need at least {self.min_samples} in "
                f"{self.capture_window_s:.1f} s")

        ordered = sorted(samples)
        valid = [(received, stamp) for received, stamp in ordered if stamp > 0]
        if len(valid) != len(ordered):
            return SyncResult(
                name, False,
                f"{len(ordered) - len(valid)} zero/invalid header stamp(s) "
                f"on {sensor.cfg['topic']}")

        stamps_monotonic = all(
            valid[index][1] > valid[index - 1][1]
            for index in range(1, len(valid))
        )
        ages = [received - stamp for received, stamp in valid]
        sorted_ages = sorted(ages)
        median_age = int(median(sorted_ages))
        p05_age = self._percentile(sorted_ages, 0.05)
        p95_age = self._percentile(sorted_ages, 0.95)
        jitter_ns = p95_age - p05_age
        edge_count = max(1, len(ages) // 5)
        drift_ns = abs(int(
            median(ages[-edge_count:]) - median(ages[:edge_count])))

        max_age_ms = self._kind_limit(
            sensor, "max_age_ms",
            250.0 if sensor.kind == "lidar" else 150.0)
        max_jitter_ms = self._kind_limit(
            sensor, "max_jitter_ms", 30.0)
        max_drift_ms = self._kind_limit(
            sensor, "max_drift_ms", self.default_max_drift_ms)
        max_age_ns = int(max_age_ms * 1_000_000)
        max_future_ns = int(self.max_future_ms * 1_000_000)
        max_jitter_ns = int(max_jitter_ms * 1_000_000)
        max_drift_ns = int(max_drift_ms * 1_000_000)

        failures: List[str] = []
        if not stamps_monotonic:
            failures.append("header stamps are not strictly increasing")
        if p05_age < -max_future_ns:
            failures.append(
                f"p05 age {p05_age / 1e6:.3f} ms is in the future")
        if p95_age > max_age_ns:
            failures.append(
                f"p95 age {p95_age / 1e6:.3f} ms exceeds "
                f"{max_age_ms:.1f} ms")
        if jitter_ns > max_jitter_ns:
            failures.append(
                f"p95-p05 jitter {jitter_ns / 1e6:.3f} ms exceeds "
                f"{max_jitter_ms:.1f} ms")
        if drift_ns > max_drift_ns:
            failures.append(
                f"edge-median drift {drift_ns / 1e6:.3f} ms exceeds "
                f"{max_drift_ms:.1f} ms")

        detail = (
            f"{len(valid)} samples on {sensor.cfg['topic']}; "
            f"age median={median_age / 1e6:.3f} ms, "
            f"p05={p05_age / 1e6:.3f} ms, "
            f"p95={p95_age / 1e6:.3f} ms; "
            f"jitter={jitter_ns / 1e6:.3f} ms; "
            f"drift={drift_ns / 1e6:.3f} ms"
        )
        if failures:
            detail += "; " + "; ".join(failures)
        return SyncResult(
            name, not failures, detail,
            offset_ns=median_age,
            tolerance_ns=max_age_ns,
        )


def show_sync_report(results: List[SyncResult]) -> bool:
    banner("Clock-sync verification")
    all_ok = True
    for r in results:
        tag = color("[ok]  ", "green") if r.ok else color("[FAIL]", "red")
        print(f"  {tag}  {r.name:14s}  {r.detail}")
        all_ok = all_ok and r.ok
    return all_ok


def handle_sync_outcome(all_ok: bool, mode: str) -> bool:
    """Return True to proceed, False to abort."""
    if all_ok:
        info("All clock-sync checks passed.")
        return True
    if mode == "hard":
        err("Clock-sync out of tolerance — aborting (mode=hard).")
        return False
    if mode == "log":
        warn("Clock-sync out of tolerance — continuing (mode=log).")
        return True
    # mode == "prompt"
    warn("One or more sync checks failed.")
    try:
        ans = input("Proceed with recording anyway? [y/N] ").strip().lower()
    except EOFError:
        ans = ""
    return ans in ("y", "yes")


# ---------------------------------------------------------------------
# RTK-fix pre-flight verification
# ---------------------------------------------------------------------
#
# GLIM++ requires the Atlas Duo to report RTK-fixed GNSS + cm-grade
# covariance to start a mapping session (see GLIM_plusplus/README.md §6).
# If the recording starts before RTK has converged, the captured bag
# either fails to init at replay time or inits part-way through (losing
# the early portion of the trajectory).
#
# This pre-flight check briefly subscribes to both /gps_p1/fix and the adapter's
# Fixed-only odometry. NavSatFix supplies the synchronized freshness/covariance
# cross-check, while arrival on the latter proves FusionEngine reported
# solution_type == kRtkFixed. REP-145 status alone cannot distinguish Float.

@dataclass
class RtkResult:
    name: str
    ok: bool
    detail: str
    samples: int = 0
    fix_status: Optional[int] = None
    pos_stddev: Optional[float] = None
    threshold: Optional[float] = None
    fixed_samples: int = 0
    imu_samples: int = 0
    fixed_quality: bool = False
    heading_stddev: Optional[float] = None   # Pose yaw quality, not source proof


_FIX_NAMES = {
    -1: "NO_FIX",
    0:  "FIX (single-point)",
    1:  "SBAS_FIX",
    2:  "GBAS_FIX (RTK-class)",
}


class RtkVerifier:
    """Verify both NavSatFix quality and the authoritative Fixed-only stream.

    Reuses rclpy. If ROS is not sourced or the adapter streams are absent,
    the returned failed result is enforced by the selected profile.
    """

    def __init__(self, cfg: Dict[str, Any]):
        rcfg = cfg.get("rtk", {})
        self.topic = rcfg.get("gnss_topic", "/gps_p1/fix")
        self.fixed_topic = rcfg.get(
            "fixed_odom_topic", "/gps_p1/filtered_odom_rtk_fixed")
        self.imu_topic = rcfg.get("imu_topic", "/gps_p1/imu")
        self.window_s = float(rcfg.get("capture_window_s", 3.0))
        self.max_stddev = float(rcfg.get("max_position_stddev", 0.10))
        self.require_rtk_fixed = bool(rcfg.get("require_rtk_fixed", True))

    def check(
            self,
            timestamp_verifier: Optional[SensorTimestampVerifier] = None,
    ) -> RtkResult:
        try:
            import rclpy
            from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                                   HistoryPolicy)
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
        except ImportError as ex:
            if timestamp_verifier is not None:
                timestamp_verifier.collection_error = (
                    f"rclpy/sensor_msgs not available: {ex} "
                    "(source ROS 2 first)")
            return RtkResult("rtk_fix", False,
                             f"rclpy/sensor_msgs not available: {ex} "
                             "(source ROS 2 first)")

        # Statuses to compare against (avoids importing NavSatStatus const).
        STATUS_GBAS_FIX = 2

        samples: list = []
        fixed_samples: list = []
        imu_samples: list = []
        heading_stddevs: list = []

        def on_fix(msg):
            cov = msg.position_covariance
            diag = (cov[0], cov[4], cov[8])
            cov_valid = (
                msg.position_covariance_type != 0
                and all(math.isfinite(v) and v >= 0.0 for v in diag)
            )
            sx, sy, sz = (
                tuple(math.sqrt(v) for v in diag)
                if cov_valid else (math.inf, math.inf, math.inf)
            )
            samples.append((msg.status.status, max(sx, sy, sz),
                            msg.latitude, msg.longitude, msg.altitude,
                            cov_valid))

        def on_fixed_odom(msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if all(math.isfinite(v) for v in values):
                fixed_samples.append(msg.header.stamp)
                # pose.covariance[35] is the yaw variance the adapter copies
                # from FusionEngine's rpy_covariance[8]. It verifies current
                # heading quality, but cannot identify whether that heading
                # came from two antennas. Commissioning is checked separately.
                yaw_var = msg.pose.covariance[35]
                if math.isfinite(yaw_var) and yaw_var > 0.0:
                    heading_stddevs.append(math.sqrt(yaw_var))

        def on_imu(msg):
            a = msg.linear_acceleration
            w = msg.angular_velocity
            values = (a.x, a.y, a.z, w.x, w.y, w.z)
            stamp_ns = (
                int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec)
            )
            if stamp_ns > 0 and all(math.isfinite(v) for v in values):
                imu_samples.append(msg.header.stamp)

        node = None
        initialized = False
        collection_error: Optional[str] = None
        try:
            rclpy.init()
            initialized = True
            node = rclpy.create_node("rtk_preflight_check")
            reliable_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, depth=10)
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=50)
            node.create_subscription(
                NavSatFix, self.topic, on_fix, reliable_qos)
            node.create_subscription(
                Odometry, self.fixed_topic, on_fixed_odom, reliable_qos)
            node.create_subscription(
                Imu, self.imu_topic, on_imu, sensor_qos)
            if timestamp_verifier is not None:
                timestamp_verifier.attach(
                    node, PointCloud2, Image, sensor_qos)
            capture_window_s = max(
                self.window_s,
                timestamp_verifier.capture_window_s
                if timestamp_verifier is not None else 0.0,
            )
            end = time.monotonic() + capture_window_s
            while time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception as ex:
            collection_error = f"ROS preflight collection failed: {ex}"
            if timestamp_verifier is not None:
                timestamp_verifier.collection_error = collection_error
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                if initialized and rclpy.ok():
                    rclpy.shutdown()

        if collection_error:
            return RtkResult(
                "rtk_fix", False, collection_error,
                imu_samples=len(imu_samples))

        if not samples:
            return RtkResult(
                "rtk_fix", False,
                f"no NavSatFix on {self.topic} during {self.window_s:.1f} s "
                "(adapter/driver not running or wrong topic?); "
                f"adapter IMU={len(imu_samples)} samples on {self.imu_topic}",
                imu_samples=len(imu_samples))

        # Worst sample drives the verdict.
        worst_status = min(s[0] for s in samples)
        worst_stddev = max(s[1] for s in samples)
        last_lat = samples[-1][2]
        last_lon = samples[-1][3]
        last_alt = samples[-1][4]

        status_ok = (not self.require_rtk_fixed) or (worst_status >= STATUS_GBAS_FIX)
        cov_ok = worst_stddev <= self.max_stddev
        finite_position_ok = all(
            math.isfinite(v) for v in (last_lat, last_lon, last_alt))
        fixed_stream_ok = (not self.require_rtk_fixed) or bool(fixed_samples)
        ok = status_ok and cov_ok and finite_position_ok and fixed_stream_ok
        fixed_quality = (
            worst_status >= STATUS_GBAS_FIX
            and cov_ok
            and finite_position_ok
            and bool(fixed_samples)
        )

        detail = (
            f"{len(samples)} samples, worst status={_FIX_NAMES.get(worst_status,'?')}, "
            f"worst σ={worst_stddev:.3f} m (threshold {self.max_stddev:.2f} m); "
            f"Fixed-only odom={len(fixed_samples)} samples on {self.fixed_topic}; "
            f"adapter IMU={len(imu_samples)} samples on {self.imu_topic}; "
            f"last fix=({last_lat:.6f}, {last_lon:.6f}, {last_alt:.2f})"
        )
        worst_heading = max(heading_stddevs) if heading_stddevs else None
        if worst_heading is not None:
            detail += f"; worst yaw σ={worst_heading:.4f} rad"
        return RtkResult("rtk_fix", ok, detail,
                         samples=len(samples), fix_status=worst_status,
                         pos_stddev=worst_stddev, threshold=self.max_stddev,
                         fixed_samples=len(fixed_samples),
                         imu_samples=len(imu_samples),
                         fixed_quality=fixed_quality,
                         heading_stddev=worst_heading)


def show_rtk_report(result: RtkResult) -> bool:
    banner("GNSS quality verification")
    tag = color("[ok]  ", "green") if result.ok else color("[FAIL]", "red")
    print(f"  {tag}  {result.name:14s}  {result.detail}")
    return result.ok


def handle_rtk_outcome(ok: bool, mode: str) -> bool:
    """Return True to proceed, False to abort.

    Modes mirror SyncVerifier:
      - "prompt": warn and ask y/N
      - "hard"  : refuse to start
      - "log"   : note in metadata and continue
    """
    if ok:
        info("Atlas adapter preflight passed the selected profile policy.")
        return True
    if mode == "hard":
        err("GNSS quality preflight failed — aborting (mode=hard).")
        return False
    if mode == "log":
        warn("GNSS quality preflight failed — continuing (mode=log).")
        return True
    # mode == "prompt"
    bar = "=" * 78
    print(color(bar, "red"))
    print(color("  GNSS QUALITY PREFLIGHT FAILED", "red"))
    print(color(bar, "red"))
    print()
    print("  The configured GNSS position/covariance policy did not pass.")
    print("  Review the preflight detail above before accepting this session.")
    print()
    try:
        ans = input("Proceed with recording anyway? [y/N] ").strip().lower()
    except EOFError:
        ans = ""
    return ans in ("y", "yes")


# ---------------------------------------------------------------------
# Subprocess management
# ---------------------------------------------------------------------
# Mandatory session requirements
# ---------------------------------------------------------------------
#
# Every profile requires the Atlas adapter IMU, synchronized clocks, and its
# declared LiDAR set. The GLIM profile additionally requires RTK-fixed and
# commissioned dual-antenna heading; GICP legitimately uses the front LiDAR
# alone. These distinctions are carried by requirements.profile.


@dataclass
class RequirementResult:
    name: str
    ok: bool
    detail: str


def _secondary_antenna_baseline_m(cfg_dir: Path, rel_path: str) -> Optional[float]:
    """Baseline length from config/sensor_dome_tf.yaml, or None if unreadable."""
    path = (cfg_dir / rel_path).resolve()
    try:
        doc = yaml.safe_load(path.read_text())
    except Exception:
        return None
    for entry in (doc or {}).get("static_transforms", []) or []:
        if entry.get("child_frame_id") == "gnss_antenna_secondary_link":
            t = entry.get("translation", {}) or {}
            try:
                x, y, z = (float(t.get(k, 0.0)) for k in ("x", "y", "z"))
            except (TypeError, ValueError):
                return None
            return math.sqrt(x * x + y * y + z * z)
    return None


def verify_requirements(
    cfg: Dict[str, Any],
    sensors: List[SensorEntry],
    sync_ok: bool,
    rtk: Optional[RtkResult],
    profile: str,
    cfg_dir: Path,
) -> List[RequirementResult]:
    req = cfg.get("requirements", {}) or {}
    out: List[RequirementResult] = []
    enabled = [s for s in sensors if s.enabled and s.detected]
    pcfg = (req.get("profiles", {}) or {}).get(profile) or {}
    rtk_required = bool(pcfg.get("require_rtk_fixed", False))

    # 1) Atlas Duo present and enabled — ALWAYS. Its IMU feeds both pipelines,
    #    so a bag without it is useless to either. RTK-FIXED is a separate
    #    question: GLIM++ cannot initialize without it, but GICP++ localizes
    #    from LiDAR + IMU against an existing map and does not need one.
    if req.get("gnss_imu", True):
        gnss = [s for s in enabled if s.kind == "gnss_imu"]
        if not gnss:
            out.append(RequirementResult(
                "gnss_imu", False,
                "Atlas Duo is not both detected and enabled. It is the IMU source for both "
                "pipelines; a bag without it is unusable by either."))
        elif rtk is None or rtk.imu_samples < 1:
            out.append(RequirementResult(
                "gnss_imu", False,
                f"Atlas hardware is reachable, but no valid adapter IMU sample "
                f"was observed on the configured topic"))
        elif not rtk_required:
            note = "not required by this profile"
            if rtk is not None and rtk.fixed_quality:
                note = "RTK-fixed (not required by this profile, but present)"
            elif rtk is not None:
                note = ("no RTK fix — accepted: this profile localizes from "
                        "LiDAR + IMU against an existing map")
            out.append(RequirementResult("gnss_imu", True,
                                         f"Atlas Duo enabled; {note}"))
        elif rtk is None:
            out.append(RequirementResult(
                "gnss_imu", False,
                "Atlas Duo enabled but the RTK pre-flight did not run."))
        elif not rtk.fixed_quality:
            out.append(RequirementResult(
                "gnss_imu", False,
                f"profile '{profile}' requires RTK-fixed at session start "
                f"(GLIM++ will not initialize without it): {rtk.detail}"))
        else:
            out.append(RequirementResult(
                "gnss_imu", True,
                f"Atlas Duo RTK-fixed ({rtk.fixed_samples} Fixed-only samples)"))

    # 2) Clock synchronization verified, not merely attempted.
    if req.get("clock_sync", True):
        out.append(RequirementResult(
            "clock_sync", bool(sync_ok),
            "all clock-sync checks passed" if sync_ok else
            "clock-sync checks did not pass; per-point time and GNSS factor "
            "association are both meaningless across unsynchronized sensors"))

    # 3) Dual antenna: configured, commissioned, and runtime-quality checked.
    # Pose yaw covariance alone cannot identify the heading source; a
    # single-antenna INS can also report a finite yaw estimate.
    if req.get("dual_antenna", True) and not pcfg.get(
            "require_dual_antenna", False):
        out.append(RequirementResult(
            "dual_antenna", True,
            f"not required by profile '{profile}' — GICP++ localizes from "
            "LiDAR + IMU against an existing map, so heading is not gating"))
    elif req.get("dual_antenna", True):
        baseline = _secondary_antenna_baseline_m(
            cfg_dir, req.get("tf_config_path", "../config/sensor_dome_tf.yaml"))
        min_base = float(req.get("min_antenna_baseline_m", 0.05))
        max_sigma = float(req.get("max_heading_stddev_rad", 0.05))
        commissioned = bool(req.get("dual_antenna_commissioned", False))
        if baseline is None:
            out.append(RequirementResult(
                "dual_antenna", False,
                "could not read gnss_antenna_secondary_link from the TF config"))
        elif not math.isfinite(baseline) or baseline < min_base:
            out.append(RequirementResult(
                "dual_antenna", False,
                f"secondary antenna is still at the ({baseline:.3f} m) sentinel "
                f"in the TF config; measure the real offset and enter it"))
        elif not commissioned:
            out.append(RequirementResult(
                "dual_antenna", False,
                f"baseline {baseline:.3f} m is configured, but "
                "requirements.dual_antenna_commissioned is false. Verify an "
                "RTK-fixed HeadingOutput in the Atlas UI/diagnostics first, "
                "then record that commissioning attestation in the config"))
        elif rtk is None or rtk.heading_stddev is None:
            out.append(RequirementResult(
                "dual_antenna", False,
                f"TF declares a {baseline:.3f} m baseline, but no finite yaw "
                "covariance was seen on Fixed-only odometry. Commissioning "
                "proves the source; this runtime check proves current quality"))
        elif rtk.heading_stddev > max_sigma:
            out.append(RequirementResult(
                "dual_antenna", False,
                f"heading σ={rtk.heading_stddev:.4f} rad exceeds "
                f"{max_sigma:.4f} rad (baseline {baseline:.3f} m)"))
        else:
            out.append(RequirementResult(
                "dual_antenna", True,
                f"commissioned dual antenna, baseline {baseline:.3f} m, "
                f"runtime yaw σ={rtk.heading_stddev:.4f} rad"))

    # 4) LiDAR set for the intended consumer.
    profiles = req.get("profiles", {}) or {}
    entry = profiles.get(profile)
    required = (entry or {}).get("lidars") if isinstance(entry, dict) else entry
    if required is None:
        out.append(RequirementResult(
            "lidar_profile", False,
            f"unknown profile '{profile}'; known: {sorted(profiles)}"))
    else:
        have = {s.name for s in enabled if s.kind == "lidar"}
        missing = [n for n in required if n not in have]
        extra = sorted(have - set(required))
        if missing:
            out.append(RequirementResult(
                "lidar_profile", False,
                f"profile '{profile}' requires {required}; not detected and enabled: "
                f"{missing}"))
        else:
            # The profile is a MINIMUM, never a cap: every LiDAR that is
            # actually fitted and responding gets recorded, so a three-unit
            # rig captured under 'gicp' still yields a three-LiDAR bag.
            note = (f"; also recording {len(extra)} further unit(s) "
                    f"beyond that minimum: {extra}") if extra else ""
            out.append(RequirementResult(
                "lidar_profile", True,
                f"profile '{profile}' requires {required} — present. "
                f"Recording all {len(have)} fitted unit(s): {sorted(have)}"
                f"{note}"))
    return out


def show_requirements(results: List[RequirementResult]) -> bool:
    banner("Mandatory requirements")
    for r in results:
        tag = color("[ok]  ", "green") if r.ok else color("[FAIL]", "red")
        print(f"  {tag}  {r.name:14s}  {r.detail}")
    ok = all(r.ok for r in results)
    if not ok:
        print()
        err("Session requirements not met — refusing to record.")
        print("  These are preconditions, not warnings: a bag missing any of")
        print("  them looks valid on disk and only fails at map-build time.")
        print("  To collect GICP++ data with the front LiDAR alone, pass")
        print("  --profile gicp (live adapter IMU and clock requirements")
        print("  still apply; RTK-fixed and dual heading do not).")
    return ok


# ---------------------------------------------------------------------

@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen
    critical: bool = True


class ProcManager:
    """Tiny supervisor that keeps every spawned subprocess in a list,
    forwards SIGINT/SIGTERM, and joins them on shutdown."""

    def __init__(self):
        self.procs: List[ManagedProcess] = []
        self._stop = threading.Event()
        self._shutdown_requested = threading.Event()

    def spawn(self, name: str, cmd: List[str], *,
              critical: bool = True, **kw) -> subprocess.Popen:
        info(f"launching {name}: {' '.join(shlex.quote(c) for c in cmd)}")
        stdout = kw.pop("stdout", subprocess.DEVNULL)
        try:
            p = subprocess.Popen(
                cmd,
                stdout=stdout,
                stderr=kw.pop("stderr", subprocess.STDOUT),
                start_new_session=True,
                **kw,
            )
        except (FileNotFoundError, OSError) as ex:
            if hasattr(stdout, "close"):
                stdout.close()
            raise RuntimeError(f"could not start {name}: {ex}") from ex
        self.procs.append(ManagedProcess(name, p, critical))
        return p

    def critical_failures(self) -> List[str]:
        return [
            f"{entry.name} exited({entry.process.returncode})"
            for entry in self.procs
            if entry.critical and entry.process.poll() is not None
        ]

    def wait_for_startup(self, grace_s: float) -> List[str]:
        deadline = time.monotonic() + max(0.0, grace_s)
        while time.monotonic() < deadline:
            failures = self.critical_failures()
            if failures:
                return failures
            time.sleep(0.05)
        return self.critical_failures()

    def statuses(self) -> List[Dict[str, Any]]:
        return [
            {
                "name": entry.name,
                "pid": entry.process.pid,
                "critical": entry.critical,
                "returncode": entry.process.poll(),
            }
            for entry in self.procs
        ]

    def request_shutdown(self) -> None:
        self._shutdown_requested.set()

    def shutdown_requested(self) -> bool:
        return self._shutdown_requested.is_set()

    def stop_all(self, sig: int = signal.SIGINT, grace_s: float = 5.0):
        if self._stop.is_set():
            return
        self._stop.set()
        # Reverse order: bag first so it can flush, then drivers.
        for entry in reversed(self.procs):
            name, p = entry.name, entry.process
            if p.poll() is None:
                step(f"stopping {name} (pid {p.pid})")
                with suppress(ProcessLookupError):
                    os.killpg(os.getpgid(p.pid), sig)
        deadline = time.time() + grace_s
        for entry in reversed(self.procs):
            name, p = entry.name, entry.process
            remaining = max(0.05, deadline - time.time())
            try:
                p.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                warn(f"{name} did not exit in {grace_s}s — sending SIGKILL")
                with suppress(ProcessLookupError):
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)


# ---------------------------------------------------------------------
# Driver and recorder construction
# ---------------------------------------------------------------------

def _expand(template: List[str], scope: Dict[str, Any]) -> List[str]:
    """Substitute {dotted.path} placeholders from `scope` into a cmd list."""
    out: List[str] = []
    for tok in template:
        def repl(m: re.Match) -> str:
            path = m.group(1).split(".")
            v: Any = scope
            for p in path:
                if isinstance(v, dict) and p in v:
                    v = v[p]
                else:
                    return m.group(0)  # leave unchanged
            return str(v)
        out.append(re.sub(r"\{([\w.]+)\}", repl, tok))
    return out


def build_topic_list(cfg: Dict[str, Any], sensors: List[SensorEntry]) -> List[str]:
    topics: List[str] = []
    for s in sensors:
        if not s.enabled:
            continue
        if s.kind == "gnss_imu":
            for t in s.cfg.get("topics", {}).values():
                topics.append(t)
        elif s.kind == "lidar":
            topics.append(s.cfg["topic"])
        elif s.kind == "camera":
            topics.append(s.cfg["topic"])
            if s.cfg.get("info_topic"):
                topics.append(s.cfg["info_topic"])
    if cfg.get("recording", {}).get("include_tf", True):
        topics.extend(["/tf", "/tf_static"])
    rate_topic = cfg.get("recording", {}).get("rate_topic")
    if rate_topic:
        topics.append(rate_topic)
    # Dedup, preserve order.
    seen, dedup = set(), []
    for t in topics:
        if t not in seen:
            seen.add(t)
            dedup.append(t)
    return dedup


def make_session_dir(cfg: Dict[str, Any], config_path: Path) -> Path:
    """Resolve recording.output_dir.

    A relative path (e.g. the default "data") is anchored to the
    directory of the loaded sensor_config.yaml so sessions land at
    <repo>/recording/data/ by default. Absolute paths and ~ are
    honored as-is.
    """
    raw = cfg.get("recording", {}).get("output_dir", "data")
    expanded = Path(os.path.expanduser(str(raw)))
    if expanded.is_absolute():
        root = expanded
    else:
        root = config_path.resolve().parent / expanded
    root.mkdir(parents=True, exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = cfg.get("recording", {}).get("session_name", "session_{ts}")
    name = name.replace("{ts}", ts)
    sess = root / name
    sess.mkdir(parents=True, exist_ok=True)
    return sess


def start_drivers(cfg: Dict[str, Any], sensors: List[SensorEntry],
                  pm: ProcManager, log_dir: Path) -> None:
    """Spawn ROS 2 sensor publishers without configuring their clocks.

    PTP_sync/ owns chrony, ptp4l, phc2sys, and device PTP configuration.
    These publishers expose the already-configured timestamps for verification
    and recording.
    """
    drivers = cfg.get("drivers", {})

    # ---- static TF ----
    # Publishes every imu_link -> sensor_link transform from
    # ../config/sensor_dome_tf.yaml so Foxglove can superimpose
    # all three Robin W point clouds (each tagged with its own
    # frame_id) into the IMU frame in the 3D panel.
    here = Path(__file__).resolve().parent
    tf_yaml = here.parent / "config" / "sensor_dome_tf.yaml"
    launch_file = here / "launch" / "static_tf.launch.py"
    if "static_tf" in drivers:
        if not tf_yaml.exists():
            warn(f"sensor_dome_tf.yaml not found at {tf_yaml} — "
                 "Foxglove will not be able to align lidar frames")
        elif not launch_file.exists():
            warn(f"static_tf.launch.py missing at {launch_file}")
        else:
            cmd = _expand(drivers["static_tf"]["cmd"], {
                "tf_yaml": str(tf_yaml),
                "launch_file": str(launch_file),
            })
            info(f"static TF source: {tf_yaml}")
            pm.spawn("static_tf", cmd,
                     stdout=open(log_dir / "static_tf.log", "wb"))

    # ---- Atlas Duo ----
    # The native FusionEngine publisher and adapter are external prerequisites.
    # The open-source fusion_engine_ros_driver emits generic PoseStamped/Imu
    # messages and cannot supply the adapter's native solution type/covariance
    # contract. Starting it here after the RTK preflight was both too late and
    # the wrong data source.
    if any(s.kind == "gnss_imu" and s.enabled for s in sensors):
        info("Atlas native-message driver + adapter are managed externally "
             "and will be checked by the live preflight")

    # ---- Robin W LiDARs (one driver instance per unit) ----
    # Each driver gets its own frame_id (lidar_front_link / _rear_left_
    # / _rear_right_), matching the child_frame_id strings in
    # sensor_dome_tf.yaml, so Foxglove's 3D panel can transform every
    # PointCloud2 from its lidar frame into imu_link and superimpose
    # the three sectors into a single 360° scene.
    for s in sensors:
        if s.kind == "lidar" and s.enabled and "seyond_lidar" in drivers:
            cmd = _expand(drivers["seyond_lidar"]["cmd"], s.cfg)
            pm.spawn(f"seyond_{s.name}", cmd,
                     stdout=open(log_dir / f"{s.name}.log", "wb"))

    # ---- Cameras (one node each) ----
    for s in sensors:
        if s.kind == "camera" and s.enabled:
            cmd = _expand(drivers["routecam"]["cmd"], s.cfg)
            pm.spawn(f"camera_{s.name}", cmd,
                     stdout=open(log_dir / f"{s.name}.log", "wb"))


def ros_package_available(package: str) -> bool:
    if not shutil.which("ros2"):
        return False
    result = subprocess.run(
        ["ros2", "pkg", "prefix", package],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def resolve_bag_storage(rec: Dict[str, Any]) -> str:
    storage = rec.get("storage", "mcap")
    if storage == "mcap" and shutil.which("ros2") and \
            not ros_package_available("rosbag2_storage_mcap"):
        warn("rosbag2_storage_mcap not found; falling back to sqlite3. "
             "Install ros-$ROS_DISTRO-rosbag2-storage-mcap for MCAP output.")
        return "sqlite3"
    return storage


def start_bag(cfg: Dict[str, Any], session_dir: Path, topics: List[str],
              pm: ProcManager) -> Tuple[Path, str]:
    rec = cfg.get("recording", {})
    bag_dir = session_dir / "rosbag2"
    storage = resolve_bag_storage(rec)
    cmd = ["ros2", "bag", "record",
           "-s", storage,
           "-o", str(bag_dir)]
    cm = rec.get("compression_mode", "none")
    if cm != "none":
        raise ValueError(
            "recording.compression_mode must be 'none'; online compression "
            "is disabled to avoid capture load and latency")
    cmd += topics
    pm.spawn("rosbag2", cmd,
             stdout=open(session_dir / "rosbag2.log", "wb"))
    return bag_dir, storage


def start_foxglove(cfg: Dict[str, Any], pm: ProcManager,
                   session_dir: Path) -> Optional[int]:
    fx = cfg.get("foxglove", {})
    if not fx.get("enabled", True):
        return None
    port = int(fx.get("bridge_port", 8765))
    cmd = _expand(cfg["drivers"]["foxglove_bridge"]["cmd"], {"port": port})
    try:
        pm.spawn(
            "foxglove_bridge", cmd, critical=False,
            stdout=open(session_dir / "foxglove_bridge.log", "wb"))
    except RuntimeError as ex:
        warn(f"{ex}; continuing without live visualization")
        return None
    return port


def start_rate_monitor(cfg: Dict[str, Any], topics: List[str],
                       pm: ProcManager, session_dir: Path) -> None:
    """Spawn the topic-rate monitor as a sibling Python process so it
    can crash without killing the recorder."""
    monitor = Path(__file__).resolve().parent / "rate_monitor.py"
    rate_topic = cfg.get("recording", {}).get("rate_topic",
                                              "/sensor_dome/rates")
    window = float(cfg.get("recording", {}).get("rate_window_s", 2.0))
    cmd = ["python3", str(monitor),
           "--rate-topic", rate_topic,
           "--window", str(window),
           "--topics", *topics]
    try:
        pm.spawn(
            "rate_monitor", cmd, critical=False,
            stdout=open(session_dir / "rate_monitor.log", "wb"))
    except RuntimeError as ex:
        warn(f"{ex}; continuing without topic-rate visualization")


# ---------------------------------------------------------------------
# Live status loop
# ---------------------------------------------------------------------

def status_loop(pm: ProcManager, session_dir: Path,
                headless: bool = False) -> bool:
    print()
    if headless:
        info("Recording headlessly. Send SIGINT or SIGTERM to stop.")
    else:
        info("Recording. Press [Q] to stop, [H] for health, [Enter] to refresh.")
    while True:
        if pm.shutdown_requested():
            return True
        failures = pm.critical_failures()
        if failures:
            for failure in failures:
                err(f"critical process failure: {failure}")
            return False
        if headless:
            time.sleep(0.5)
            continue
        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.5)
        except (OSError, ValueError):
            readable = [sys.stdin]
        if not readable:
            continue
        try:
            raw = sys.stdin.readline()
        except KeyboardInterrupt:
            return True
        if raw == "":
            warn("stdin closed; continuing in headless mode")
            headless = True
            continue
        line = raw.strip().lower()
        if line in ("q", "quit", "exit"):
            return True
        if line in ("h", "health"):
            for entry in pm.procs:
                name, p = entry.name, entry.process
                live = "alive" if p.poll() is None else f"exited({p.returncode})"
                print(f"  {name:25s} pid={p.pid:<6d} {live}")
        else:
            print(f"  session: {session_dir}")
            alive = sum(entry.process.poll() is None for entry in pm.procs)
            print(f"  procs:   {alive}/{len(pm.procs)} alive")


def verify_bag_output(bag_dir: Path,
                      expected_storage: Optional[str] = None) -> Tuple[bool, str]:
    metadata = bag_dir / "metadata.yaml"
    if not metadata.is_file() or metadata.stat().st_size == 0:
        return False, f"missing or empty rosbag metadata: {metadata}"
    try:
        document = yaml.safe_load(metadata.read_text()) or {}
        info = document["rosbag2_bagfile_information"]
        message_count = int(info["message_count"])
        storage = str(info["storage_identifier"])
        relative_paths = info["relative_file_paths"]
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as ex:
        return False, f"invalid rosbag metadata in {metadata}: {ex}"
    if message_count <= 0:
        return False, f"rosbag contains no messages: {metadata}"
    if expected_storage and storage != expected_storage:
        return False, (
            f"rosbag storage mismatch: metadata={storage}, "
            f"expected={expected_storage}")
    storage_token = ".mcap" if storage == "mcap" else \
        ".db3" if storage == "sqlite3" else ""
    if not storage_token:
        return False, f"unsupported rosbag storage identifier: {storage}"
    if not isinstance(relative_paths, list) or not relative_paths:
        return False, f"rosbag metadata lists no payload files: {metadata}"
    payloads: List[Path] = []
    bag_root = bag_dir.resolve()
    for relative in relative_paths:
        if not isinstance(relative, str) or storage_token not in relative:
            return False, f"invalid {storage} payload path in metadata: {relative!r}"
        path = (bag_dir / relative).resolve()
        try:
            path.relative_to(bag_root)
        except ValueError:
            return False, f"rosbag payload escapes bag directory: {relative}"
        if not path.is_file() or path.stat().st_size <= 0:
            return False, f"missing or empty rosbag payload: {path}"
        payloads.append(path)
    total = sum(path.stat().st_size for path in payloads)
    return True, (
        f"{message_count} messages in {len(payloads)} {storage} payload "
        f"file(s), {total} bytes")


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def parse_args(argv: List[str]) -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(
        description="Hitch Sensor Dome — ROS 2 MCAP recorder + Foxglove dashboard",
    )
    p.add_argument("--config", type=Path,
                   default=here / "sensor_config.yaml",
                   help="Path to sensor_config.yaml")
    p.add_argument("--eth", help="Override network.interface")
    p.add_argument("--output", help="Override recording.output_dir")
    p.add_argument("--no-foxglove", action="store_true",
                   help="Skip foxglove_bridge (recording only)")
    p.add_argument("--headless", action="store_true",
                   help="Auto-confirm detected sensors and record until "
                        "SIGINT/SIGTERM; hard requirements still apply")
    p.add_argument("--yes", "-y", action="store_true",
                   help="Auto-confirm sensor checklist")
    p.add_argument("--profile", choices=["glim", "gicp"],
                   help="Intended consumer of this recording. 'glim' requires "
                        "all three Robin W units (the offline map is built "
                        "from them); 'gicp' requires only the front unit and "
                        "is a legitimate localization data-collection run. "
                        "Live adapter IMU and clock sync apply to both; RTK "
                        "and dual heading are profile-specific. "
                        "Default: requirements.profile.")
    p.add_argument("--sync-mode", choices=["prompt", "hard", "log"],
                   help="Override sync.mode; only 'hard' is accepted while "
                        "requirements.clock_sync is true")
    p.add_argument("--rtk-mode", choices=["prompt", "hard", "log"],
                   help="Control handling of GNSS preflight failure only when "
                        "the profile does not require RTK-fixed. 'hard' then "
                        "requires finite position and covariance within the "
                        "configured threshold; it does not require a Fixed "
                        "solution. Fixed-only profiles cannot be downgraded.")
    p.add_argument("--skip-rtk-check", action="store_true",
                   help="Allow non-fixed GNSS only on a profile that does not "
                        "require RTK-fixed; rejected by the GLIM profile")
    p.add_argument("--dry-run", action="store_true",
                   help="Detect + sync-check only, do not record")
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    if not args.config.exists():
        err(f"config not found: {args.config}")
        return 2
    try:
        with open(args.config) as f:
            cfg = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as ex:
        err(f"failed to load config {args.config}: {ex}")
        return 2
    if not isinstance(cfg, dict):
        err(f"config root must be a mapping: {args.config}")
        return 2

    if args.eth:
        cfg.setdefault("network", {})["interface"] = args.eth
    if args.output:
        cfg.setdefault("recording", {})["output_dir"] = args.output
    if args.no_foxglove:
        cfg.setdefault("foxglove", {})["enabled"] = False
    if args.sync_mode:
        cfg.setdefault("sync", {})["mode"] = args.sync_mode
    if args.rtk_mode:
        cfg.setdefault("rtk", {})["mode"] = args.rtk_mode
    if args.skip_rtk_check:
        cfg.setdefault("rtk", {})["mode"] = "log"
    iface = cfg.get("network", {}).get("interface", "eth0")
    banner(f"Hitch Sensor Dome recorder  —  iface={iface}")

    # 1) Detect.
    step("Auto-detecting sensors")
    detector = SensorDetector(cfg)
    sensors = detector.detect_all()
    sensors = confirm_sensors(sensors,
                              assume_yes=args.yes or args.headless)

    enabled = [s for s in sensors if s.enabled]
    if not enabled:
        err("No sensors selected — nothing to record.")
        return 1

    reqs = cfg.get("requirements", {}) or {}
    profile = args.profile or reqs.get("profile", "glim")
    profiles = reqs.get("profiles", {}) or {}
    if profile not in profiles or not isinstance(profiles[profile], dict):
        err(f"unknown recording profile '{profile}'; "
            f"known profiles: {sorted(profiles)}")
        return 2
    required_lidars = profiles[profile].get("lidars")
    if not isinstance(required_lidars, list) or not all(
            isinstance(name, str) and name for name in required_lidars):
        err(f"requirements.profiles.{profile}.lidars must be a list "
            "of nonempty sensor names")
        return 2

    info(f"Recording profile: {profile} "
         f"(minimum LiDAR set: {required_lidars})")
    # A mandatory requirement is not something a mode knob may downgrade.
    if reqs.get("clock_sync", True):
        if args.sync_mode and args.sync_mode != "hard":
            err("requirements.clock_sync is enabled; "
                "--sync-mode cannot downgrade it")
            return 2
        cfg.setdefault("sync", {})["mode"] = "hard"
    rtk_required = bool(profiles[profile].get("require_rtk_fixed", False))
    cfg.setdefault("rtk", {})["require_rtk_fixed"] = rtk_required
    if rtk_required:
        if args.skip_rtk_check or args.rtk_mode == "log":
            err(f"profile '{profile}' requires RTK-fixed; "
                "--skip-rtk-check/--rtk-mode log cannot downgrade it")
            return 2
        cfg.setdefault("rtk", {})["mode"] = "hard"
    else:
        # The pre-flight still RUNS and is still reported — it just does not
        # gate, because this profile can be localized from LiDAR + IMU.
        # --rtk-mode controls how its valid-position/covariance result is
        # handled; require_rtk_fixed remains false for this profile.
        if args.skip_rtk_check:
            cfg.setdefault("rtk", {})["mode"] = "log"
        elif args.rtk_mode:
            cfg.setdefault("rtk", {})["mode"] = args.rtk_mode
        else:
            cfg.setdefault("rtk", {})["mode"] = "log"

    # 2) Start each selected sensor driver once. The same processes feed the
    # direct timestamp preflight and, if it passes, the recorded bag.
    session_dir = make_session_dir(cfg, args.config)
    log_dir = session_dir / "logs"
    log_dir.mkdir(exist_ok=True)
    pm = ProcManager()
    install_signal_handlers(pm)

    step("Starting sensor drivers for live preflight")
    try:
        start_drivers(cfg, sensors, pm, log_dir)
    except (RuntimeError, KeyError, TypeError, ValueError) as ex:
        err(str(ex))
        pm.stop_all()
        return 1
    startup_grace = float(
        cfg.get("recording", {}).get("startup_grace_s", 2.0))
    failures = pm.wait_for_startup(startup_grace)
    if failures:
        for failure in failures:
            err(f"sensor startup failed: {failure}")
        pm.stop_all()
        return 1

    # 3) Measure every sensor header directly against CLOCK_REALTIME while the
    # existing Atlas preflight spins the ROS executor.
    try:
        timestamp_verifier = SensorTimestampVerifier(cfg, sensors)
    except (KeyError, TypeError, ValueError) as ex:
        err(str(ex))
        pm.stop_all()
        return 1

    try:
        chrony_result = SyncVerifier(cfg).check_chrony()
        rtk_result: Optional[RtkResult] = None
        rtk_ok = True
        if any(s.kind == "gnss_imu" and s.enabled for s in sensors):
            step("Measuring live sensor timestamps and Atlas GNSS quality")
            rtk_verifier = RtkVerifier(cfg)
            rtk_result = rtk_verifier.check(timestamp_verifier)
            rtk_ok = show_rtk_report(rtk_result)
        else:
            step("Measuring live sensor timestamps")
            timestamp_verifier.collect()
            info("No Atlas Duo selected — skipping GNSS quality preflight.")
        sync_results = [chrony_result, *timestamp_verifier.results()]
    except (KeyError, RuntimeError, TypeError, ValueError) as ex:
        err(f"live preflight failed: {ex}")
        pm.stop_all()
        return 1

    failures = pm.critical_failures()
    if failures:
        for failure in failures:
            err(f"sensor failed during live preflight: {failure}")
        pm.stop_all()
        return 1

    all_ok = show_sync_report(sync_results)
    mode = cfg.get("sync", {}).get("mode", "prompt")
    if not handle_sync_outcome(all_ok, mode):
        err("Recording aborted by sensor timestamp policy.")
        pm.stop_all()
        return 1
    if rtk_result is not None:
        rtk_mode = cfg.get("rtk", {}).get("mode", "prompt")
        if not handle_rtk_outcome(rtk_ok, rtk_mode):
            err("Recording aborted by GNSS quality policy.")
            pm.stop_all()
            return 1

    req_results = verify_requirements(
        cfg, sensors, all_ok, rtk_result, profile, args.config.resolve().parent)
    if not show_requirements(req_results):
        pm.stop_all()
        return 1

    if args.dry_run:
        info("--dry-run: live preflight passed; stopping before bag creation.")
        pm.stop_all()
        return 0

    # 4) Start the bag and noncritical visualization helpers. Sensor drivers
    # remain the exact processes whose timestamp streams passed above.
    topics = build_topic_list(cfg, sensors)
    step(f"Starting recording session at {session_dir}")
    try:
        bag_dir, bag_storage = start_bag(
            cfg, session_dir, topics, pm)
        start_rate_monitor(cfg, topics, pm, session_dir)
        fx_port = start_foxglove(cfg, pm, session_dir)
    except (RuntimeError, KeyError, TypeError, ValueError) as ex:
        err(str(ex))
        pm.stop_all()
        return 1

    failures = pm.wait_for_startup(startup_grace)
    if failures:
        for failure in failures:
            err(f"startup failed: {failure}")
        pm.stop_all()
        return 1

    # 5) Persist metadata up-front (in case of crash).
    meta = SessionInfo(
        started_utc=datetime.now(timezone.utc).isoformat(),
        output_dir=str(session_dir),
        storage=bag_storage,
        sensors=[asdict(s) for s in sensors],
        sync=[asdict(r) for r in sync_results],
        requirements=[asdict(r) for r in req_results],
        profile=profile,
        topics=topics,
        notes=(f"foxglove_bridge ws://localhost:{fx_port}"
               if fx_port else "foxglove disabled"),
    )
    (session_dir / "session_metadata.json").write_text(
        json.dumps(asdict(meta), indent=2, default=str))

    if fx_port:
        layout = (Path(__file__).resolve().parent
                  / cfg["foxglove"].get("layout_file",
                                        "foxglove/sensor_dome_layout.json"))
        if layout.exists():
            info(f"Foxglove layout: {layout}")
            info(f"In Foxglove Studio: Open Connection → "
                 f"Foxglove WebSocket → ws://localhost:{fx_port}")
            info(f"Then: Layouts → Import from file → {layout}")

    # 6) Run.
    run_ok = False
    try:
        run_ok = status_loop(
            pm, session_dir,
            headless=args.headless or not sys.stdin.isatty())
    finally:
        step("Tearing down")
        pm.stop_all()
        info(f"Session saved to: {session_dir}")
        info(f"Rosbag ({bag_storage}): {bag_dir}")
    bag_ok, bag_detail = verify_bag_output(bag_dir, bag_storage)
    if bag_ok:
        info(f"Rosbag validation passed: {bag_detail}")
    else:
        err(f"Rosbag validation failed: {bag_detail}")
    meta.ended_utc = datetime.now(timezone.utc).isoformat()
    meta.success = run_ok and bag_ok
    meta.processes = pm.statuses()
    meta.notes += f"; rosbag validation: {bag_detail}"
    (session_dir / "session_metadata.json").write_text(
        json.dumps(asdict(meta), indent=2, default=str))
    return 0 if meta.success else 1


def install_signal_handlers(pm: ProcManager) -> None:
    def _handler(signum, frame):
        warn(f"received signal {signum} — shutting down")
        pm.request_shutdown()
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, _handler)


if __name__ == "__main__":
    sys.exit(main())
