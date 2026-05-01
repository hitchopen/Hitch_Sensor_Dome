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
#   3. Verify the PTP / chrony clock-sync chain. If any sensor
#      exceeds its tolerance, warn and prompt the user (y/N) to
#      proceed (configurable: prompt | hard | log).
#   4. Spawn ROS 2 driver nodes for every confirmed sensor,
#      static-TF publisher, foxglove_bridge, an internal rate
#      monitor, and `ros2 bag record -s mcap` on the relevant
#      topics.
#   5. Print live status. R/H/Q for re-arm / health / quit.
#   6. On exit, write session_metadata.json and tear everything
#      down cleanly.
#
# Run:
#   sudo python3 sensor_recorder.py                     # YAML defaults
#   sudo python3 sensor_recorder.py --config my.yaml
#   sudo python3 sensor_recorder.py --no-foxglove --headless
#   python3 sensor_recorder.py --dry-run                # detect only
#
# =============================================================

from __future__ import annotations

import argparse
import json
import os
import re
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
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("ERROR: pyyaml is required. Install with: pip install pyyaml")


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
    sensors: List[Dict[str, Any]] = field(default_factory=list)
    sync: List[Dict[str, Any]] = field(default_factory=list)
    topics: List[str] = field(default_factory=list)
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
        port = c.get("device_port", "/dev/ttyUSB0")
        if Path(port).exists():
            entry.detected = True
            entry.detail = f"serial port {port} present"
        else:
            entry.detail = f"{port} not found (Atlas Duo unplugged?)"
        # Bonus: confirm gpsd / NMEA reachability if it's running.
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
                    entries[idx].enabled = not entries[idx].enabled

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
    """Inspect the GPS → chrony → ptp4l → PTP-slave chain.

    Each check returns a SyncResult so the caller can decide what to do.
    All commands are read-only and safe to invoke without root, except
    `pmc` which requires CAP_NET_ADMIN — we degrade gracefully if it
    isn't available.
    """

    def __init__(self, cfg: Dict[str, Any], iface: str):
        self.cfg = cfg
        self.iface = iface
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
        ok = offset_ns is not None and offset_ns <= tol_ns
        # Reference must be GPS/PPS/NMEA, not an IP-style NTP server.
        gps_disciplined = bool(ref) and (
            ref_name.upper() in ("GPS", "PPS", "NMEA") or
            ref_name.startswith("PPS") or ref_name.startswith("NMEA")
        )
        if not gps_disciplined:
            ok = False
        return SyncResult(
            "chrony", ok,
            f"ref={ref_name} offset={offset_ns} ns (tol {tol_ns} ns)",
            offset_ns=offset_ns, tolerance_ns=tol_ns,
        )

    # ptp4l master
    def check_ptp_master(self) -> SyncResult:
        # Hardware vs software timestamping decides the tolerance.
        hw = self._iface_hw_ts(self.iface)
        tol_key = "ptp_master_offset_ns_hw" if hw else "ptp_master_offset_ns_sw"
        tol_ns = int(self.tol.get(tol_key, 1000 if hw else 50000))

        # Tail the ptp4l-grandmaster journal for the most recent offset.
        if not shutil.which("journalctl"):
            return SyncResult("ptp4l", False, "journalctl not available")
        try:
            r = subprocess.run(
                ["journalctl", "-u", "ptp4l-grandmaster",
                 "-n", "200", "--no-pager"],
                capture_output=True, text=True, timeout=3,
            )
        except Exception as ex:
            return SyncResult("ptp4l", False, f"journalctl failed: {ex}")
        # Look for "master offset <N>" — last occurrence.
        offs = [int(m.group(1)) for m in re.finditer(
            r"master offset\s+(-?\d+)", r.stdout)]
        if not offs:
            return SyncResult("ptp4l", False,
                              "no 'master offset' lines (service down?)")
        last = offs[-1]
        ok = abs(last) <= tol_ns
        return SyncResult(
            "ptp4l", ok,
            f"master offset={last} ns (tol {tol_ns} ns, "
            + ("hardware TS" if hw else "software TS") + ")",
            offset_ns=last, tolerance_ns=tol_ns,
        )

    @staticmethod
    def _iface_hw_ts(iface: str) -> bool:
        if not shutil.which("ethtool"):
            return False
        try:
            r = subprocess.run(["ethtool", "-T", iface],
                               capture_output=True, text=True, timeout=3)
            return "hardware-transmit" in r.stdout and \
                   "hardware-receive" in r.stdout
        except Exception:
            return False

    # PTP slaves (LiDARs + cameras) via pmc
    def check_ptp_slaves(self, expect: List[str]) -> List[SyncResult]:
        results: List[SyncResult] = []
        if not shutil.which("pmc"):
            results.append(SyncResult("pmc", False,
                                      "pmc not installed; cannot verify slaves"))
            return results
        try:
            r = subprocess.run(
                ["sudo", "-n", "pmc", "-u", "-b", "0", "GET PORT_DATA_SET"],
                capture_output=True, text=True, timeout=4,
            )
        except Exception as ex:
            results.append(SyncResult("pmc", False, f"pmc failed: {ex}"))
            return results

        # We treat the presence of SLAVE entries as confirmation.
        slaves = re.findall(r"portState\s+SLAVE", r.stdout)
        results.append(SyncResult(
            "ptp_slaves", len(slaves) >= len(expect),
            f"{len(slaves)} SLAVE port(s) reported, expected ≥ {len(expect)}",
        ))
        return results

    def all_checks(self, expected_slaves: List[str]) -> List[SyncResult]:
        out: List[SyncResult] = [self.check_chrony(), self.check_ptp_master()]
        out.extend(self.check_ptp_slaves(expected_slaves))
        return out


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
# This pre-flight check briefly subscribes to /gps/fix and verifies that
# every sample in the capture window passes the same gate the GLIM++
# C++ wrapper uses. It mirrors the SyncVerifier pattern: a Result
# dataclass, a show_*_report function, a handle_*_outcome function that
# implements the prompt / hard / log modes.

@dataclass
class RtkResult:
    name: str
    ok: bool
    detail: str
    samples: int = 0
    fix_status: Optional[int] = None
    pos_stddev: Optional[float] = None
    threshold: Optional[float] = None


_FIX_NAMES = {
    -1: "NO_FIX",
    0:  "FIX (single-point)",
    1:  "SBAS_FIX",
    2:  "GBAS_FIX (RTK-class)",
}


class RtkVerifier:
    """Subscribe briefly to /gps/fix and verify RTK-fixed quality.

    Reuses rclpy. If rclpy isn't importable (no ROS environment sourced),
    we degrade to "skipped" rather than aborting — the user can still
    record without ROS-side gating, accepting the operational risk.
    """

    def __init__(self, cfg: Dict[str, Any]):
        rcfg = cfg.get("rtk", {})
        self.topic = rcfg.get("gnss_topic", "/gps/fix")
        self.window_s = float(rcfg.get("capture_window_s", 3.0))
        self.max_stddev = float(rcfg.get("max_position_stddev", 0.10))
        self.require_rtk_fixed = bool(rcfg.get("require_rtk_fixed", True))

    def check(self) -> RtkResult:
        try:
            import rclpy
            from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                                   HistoryPolicy)
            from sensor_msgs.msg import NavSatFix
        except ImportError as ex:
            return RtkResult("rtk_fix", False,
                             f"rclpy/sensor_msgs not available: {ex} "
                             "(skipped — source ROS 2 first)")

        # Statuses to compare against (avoids importing NavSatStatus const).
        STATUS_GBAS_FIX = 2

        rclpy.init()
        node = rclpy.create_node("rtk_preflight_check")
        samples: list = []

        def on_fix(msg):
            cov = msg.position_covariance
            sx, sy, sz = (max(0.0, cov[0]) ** 0.5,
                          max(0.0, cov[4]) ** 0.5,
                          max(0.0, cov[8]) ** 0.5)
            samples.append((msg.status.status, max(sx, sy, sz),
                            msg.latitude, msg.longitude, msg.altitude))

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        node.create_subscription(NavSatFix, self.topic, on_fix, qos)
        end = time.monotonic() + self.window_s
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()

        if not samples:
            return RtkResult(
                "rtk_fix", False,
                f"no NavSatFix on {self.topic} during {self.window_s:.1f} s "
                "(driver not running, no antenna lock, or wrong topic?)")

        # Worst sample drives the verdict.
        worst_status = min(s[0] for s in samples)
        worst_stddev = max(s[1] for s in samples)
        last_lat = samples[-1][2]
        last_lon = samples[-1][3]
        last_alt = samples[-1][4]

        status_ok = (not self.require_rtk_fixed) or (worst_status >= STATUS_GBAS_FIX)
        cov_ok = worst_stddev <= self.max_stddev
        ok = status_ok and cov_ok

        detail = (
            f"{len(samples)} samples, worst status={_FIX_NAMES.get(worst_status,'?')}, "
            f"worst σ={worst_stddev:.3f} m (threshold {self.max_stddev:.2f} m); "
            f"last fix=({last_lat:.6f}, {last_lon:.6f}, {last_alt:.2f})"
        )
        return RtkResult("rtk_fix", ok, detail,
                         samples=len(samples), fix_status=worst_status,
                         pos_stddev=worst_stddev, threshold=self.max_stddev)


def show_rtk_report(result: RtkResult) -> bool:
    banner("RTK-fix verification")
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
        info("RTK-fix check passed — Atlas Duo reports RTK-fixed with "
             "cm-grade covariance.")
        return True
    if mode == "hard":
        err("RTK not at fixed quality — aborting (mode=hard).")
        return False
    if mode == "log":
        warn("RTK not at fixed quality — continuing (mode=log).")
        return True
    # mode == "prompt"
    bar = "=" * 78
    print(color(bar, "red"))
    print(color("  ⚠  RTK NOT YET FIXED — GLIM++ will not be able to init", "red"))
    print(color(bar, "red"))
    print()
    print("  GLIM++ requires Atlas Duo RTK-fixed status with cm-grade")
    print("  covariance to start a mapping session. If you record now, the")
    print("  bag will need either:")
    print("    - To run long enough that RTK locks mid-bag (early data lost)")
    print("    - Or to be replayed with the GLIM++ gate relaxed:")
    print("        ros2 launch ... ins_require_rtk_fixed:=false")
    print()
    print("  Common remediations before recording:")
    print("    1. Park with clear sky view; wait 30–120 s for RTK convergence.")
    print("    2. Verify NTRIP corrections are flowing (Atlas Duo web UI).")
    print("    3. Check the GNSS antenna cable and SP1 connector.")
    print()
    try:
        ans = input("Proceed with recording anyway? [y/N] ").strip().lower()
    except EOFError:
        ans = ""
    return ans in ("y", "yes")


# ---------------------------------------------------------------------
# Subprocess management
# ---------------------------------------------------------------------

class ProcManager:
    """Tiny supervisor that keeps every spawned subprocess in a list,
    forwards SIGINT/SIGTERM, and joins them on shutdown."""

    def __init__(self):
        self.procs: List[Tuple[str, subprocess.Popen]] = []
        self._stop = threading.Event()

    def spawn(self, name: str, cmd: List[str], **kw) -> subprocess.Popen:
        info(f"launching {name}: {' '.join(shlex.quote(c) for c in cmd)}")
        try:
            p = subprocess.Popen(
                cmd,
                stdout=kw.pop("stdout", subprocess.DEVNULL),
                stderr=kw.pop("stderr", subprocess.STDOUT),
                start_new_session=True,
                **kw,
            )
        except FileNotFoundError as ex:
            err(f"could not start {name}: {ex}")
            return None  # type: ignore[return-value]
        self.procs.append((name, p))
        return p

    def stop_all(self, sig: int = signal.SIGINT, grace_s: float = 5.0):
        if self._stop.is_set():
            return
        self._stop.set()
        # Reverse order: bag first so it can flush, then drivers.
        for name, p in reversed(self.procs):
            if p.poll() is None:
                step(f"stopping {name} (pid {p.pid})")
                with suppress(ProcessLookupError):
                    os.killpg(os.getpgid(p.pid), sig)
        deadline = time.time() + grace_s
        for name, p in reversed(self.procs):
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
    """Spawn ROS 2 driver nodes for every enabled sensor."""
    drivers = cfg.get("drivers", {})

    # ---- static TF ----
    # Publishes every imu_link -> sensor_link transform from
    # ../ROS2 config/sensor_dome_tf.yaml so Foxglove can superimpose
    # all three Robin W point clouds (each tagged with its own
    # frame_id) into the IMU frame in the 3D panel.
    here = Path(__file__).resolve().parent
    tf_yaml = here.parent / "ROS2 config" / "sensor_dome_tf.yaml"
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
    for s in sensors:
        if s.kind == "gnss_imu" and s.enabled:
            cmd = _expand(drivers["point_one_nav"]["cmd"], s.cfg)
            pm.spawn("point_one_nav", cmd,
                     stdout=open(log_dir / "point_one_nav.log", "wb"))

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


def start_bag(cfg: Dict[str, Any], session_dir: Path, topics: List[str],
              pm: ProcManager) -> Path:
    rec = cfg.get("recording", {})
    bag_dir = session_dir / "rosbag2"
    cmd = ["ros2", "bag", "record",
           "-s", rec.get("storage", "mcap"),
           "-o", str(bag_dir)]
    cm = rec.get("compression_mode", "none")
    cf = rec.get("compression_format", "")
    if cm and cm != "none":
        cmd += ["--compression-mode", cm]
        if cf:
            cmd += ["--compression-format", cf]
    cmd += topics
    pm.spawn("rosbag2", cmd,
             stdout=open(session_dir / "rosbag2.log", "wb"))
    return bag_dir


def start_foxglove(cfg: Dict[str, Any], pm: ProcManager,
                   session_dir: Path) -> Optional[int]:
    fx = cfg.get("foxglove", {})
    if not fx.get("enabled", True):
        return None
    port = int(fx.get("bridge_port", 8765))
    cmd = _expand(cfg["drivers"]["foxglove_bridge"]["cmd"], {"port": port})
    pm.spawn("foxglove_bridge", cmd,
             stdout=open(session_dir / "foxglove_bridge.log", "wb"))
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
    pm.spawn("rate_monitor", cmd,
             stdout=open(session_dir / "rate_monitor.log", "wb"))


# ---------------------------------------------------------------------
# Live status loop
# ---------------------------------------------------------------------

def status_loop(pm: ProcManager, session_dir: Path) -> None:
    print()
    info("Recording. Press [Q] to stop, [H] for health, [Enter] to refresh.")
    while True:
        try:
            line = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if line in ("q", "quit", "exit"):
            break
        if line in ("h", "health"):
            for name, p in pm.procs:
                live = "alive" if p.poll() is None else f"exited({p.returncode})"
                print(f"  {name:25s} pid={p.pid:<6d} {live}")
        else:
            print(f"  session: {session_dir}")
            print(f"  procs:   {len(pm.procs)} running")


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
                   help="No interactive prompts; auto-confirm + non-blocking")
    p.add_argument("--yes", "-y", action="store_true",
                   help="Auto-confirm sensor checklist")
    p.add_argument("--sync-mode", choices=["prompt", "hard", "log"],
                   help="Override sync.mode")
    p.add_argument("--rtk-mode", choices=["prompt", "hard", "log"],
                   help="Override rtk.mode (RTK-fix pre-flight check). "
                        "GLIM++ requires RTK-fixed at session start; "
                        "default 'prompt' warns and asks y/N.")
    p.add_argument("--skip-rtk-check", action="store_true",
                   help="Equivalent to --rtk-mode log: record regardless of "
                        "RTK status (Atlas Duo cold-start, indoor smoke "
                        "tests, etc.). The captured bag may not satisfy "
                        "GLIM++'s init gate at replay time.")
    p.add_argument("--dry-run", action="store_true",
                   help="Detect + sync-check only, do not record")
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv or sys.argv[1:])

    if not args.config.exists():
        err(f"config not found: {args.config}")
        return 2
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

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
    if args.headless:
        cfg.setdefault("sync", {})["mode"] = "log"
        cfg.setdefault("rtk",  {}).setdefault("mode", "log")

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

    # 2) Verify clock sync.
    step("Verifying time synchronization")
    verifier = SyncVerifier(cfg, iface)
    expected = [s.name for s in enabled if s.kind in ("lidar", "camera")]
    sync_results = verifier.all_checks(expected)
    all_ok = show_sync_report(sync_results)
    mode = cfg.get("sync", {}).get("mode", "prompt")
    if not handle_sync_outcome(all_ok, mode):
        err("Aborted by operator.")
        return 1

    # 2a) RTK-fix pre-flight (only if Atlas Duo is enabled).
    rtk_result: Optional[RtkResult] = None
    if any(s.kind == "gnss_imu" and s.enabled for s in sensors):
        step("Verifying RTK-fixed GNSS for GLIM++ initialization")
        rtk_verifier = RtkVerifier(cfg)
        rtk_result = rtk_verifier.check()
        rtk_ok = show_rtk_report(rtk_result)
        rtk_mode = cfg.get("rtk", {}).get("mode", "prompt")
        if not handle_rtk_outcome(rtk_ok, rtk_mode):
            err("Aborted by operator (RTK not fixed).")
            return 1
    else:
        info("No Atlas Duo selected — skipping RTK-fix pre-flight.")

    if args.dry_run:
        info("--dry-run: stopping before recording.")
        return 0

    # 3) Build session dir + topic list.
    session_dir = make_session_dir(cfg, args.config)
    log_dir = session_dir / "logs"
    log_dir.mkdir(exist_ok=True)
    topics = build_topic_list(cfg, sensors)

    # 4) Spawn everything.
    step(f"Starting recording session at {session_dir}")
    pm = ProcManager()
    install_signal_handlers(pm)

    start_drivers(cfg, sensors, pm, log_dir)
    bag_dir = start_bag(cfg, session_dir, topics, pm)
    start_rate_monitor(cfg, topics, pm, session_dir)
    fx_port = start_foxglove(cfg, pm, session_dir)

    # 5) Persist metadata up-front (in case of crash).
    meta = SessionInfo(
        started_utc=datetime.now(timezone.utc).isoformat(),
        output_dir=str(session_dir),
        storage=cfg.get("recording", {}).get("storage", "mcap"),
        sensors=[asdict(s) for s in sensors],
        sync=[asdict(r) for r in sync_results],
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
    try:
        status_loop(pm, session_dir)
    finally:
        step("Tearing down")
        pm.stop_all()
        info(f"Session saved to: {session_dir}")
        info(f"MCAP bag:         {bag_dir}")
    return 0


def install_signal_handlers(pm: ProcManager) -> None:
    def _handler(signum, frame):
        warn(f"received signal {signum} — shutting down")
        pm.stop_all()
        sys.exit(130)
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, _handler)


if __name__ == "__main__":
    sys.exit(main())
