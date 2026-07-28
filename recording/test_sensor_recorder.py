#!/usr/bin/env python3

import sys
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import sensor_recorder as recorder


def sensor(kind, name, *, detected=True, enabled=True):
    return recorder.SensorEntry(
        kind=kind,
        name=name,
        cfg={},
        detected=detected,
        enabled=enabled,
    )


def base_config():
    return {
        "requirements": {
            "gnss_imu": True,
            "clock_sync": True,
            "dual_antenna": False,
            "profiles": {
                "gicp": {
                    "lidars": ["robin_w_front"],
                    "require_rtk_fixed": False,
                    "require_dual_antenna": False,
                },
                "glim": {
                    "lidars": [
                        "robin_w_front",
                        "robin_w_rear_left",
                        "robin_w_rear_right",
                    ],
                    "require_rtk_fixed": True,
                    "require_dual_antenna": False,
                },
            },
        }
    }


def result_map(results):
    return {result.name: result for result in results}


class RequirementTests(unittest.TestCase):
    @staticmethod
    def p1_result(*, fixed=False):
        return recorder.RtkResult(
            "rtk_fix",
            fixed,
            "fixed" if fixed else "IMU only",
            fix_status=2 if fixed else None,
            fixed_samples=5 if fixed else 0,
            imu_samples=5,
            fixed_quality=fixed,
        )

    def test_camera_free_gicp_accepts_one_to_three_lidars(self):
        cfg = base_config()
        rear_lidars = ["robin_w_rear_left", "robin_w_rear_right"]
        for rear_count in range(3):
            with self.subTest(lidar_count=rear_count + 1):
                sensors = [
                    sensor("gnss_imu", "point_one_nav_atlas_duo"),
                    sensor("lidar", "robin_w_front"),
                    *[
                        sensor("lidar", name)
                        for name in rear_lidars[:rear_count]
                    ],
                ]
                results = recorder.verify_requirements(
                    cfg, sensors, True, self.p1_result(), "gicp", Path("."))
                self.assertTrue(all(result.ok for result in results))

    def test_camera_free_glim_requires_and_accepts_all_three_lidars(self):
        cfg = base_config()
        sensors = [
            sensor("gnss_imu", "point_one_nav_atlas_duo"),
            sensor("lidar", "robin_w_front"),
            sensor("lidar", "robin_w_rear_left"),
            sensor("lidar", "robin_w_rear_right"),
        ]
        results = recorder.verify_requirements(
            cfg, sensors, True, self.p1_result(fixed=True), "glim", Path("."))
        self.assertTrue(all(result.ok for result in results))

        results = result_map(recorder.verify_requirements(
            cfg, sensors[:-1], True, self.p1_result(fixed=True),
            "glim", Path(".")))
        self.assertFalse(results["lidar_profile"].ok)

    def test_gicp_still_requires_the_front_primary_lidar(self):
        cfg = base_config()
        sensors = [
            sensor("gnss_imu", "point_one_nav_atlas_duo"),
            sensor("lidar", "robin_w_rear_left"),
            sensor("lidar", "robin_w_rear_right"),
        ]
        results = result_map(recorder.verify_requirements(
            cfg, sensors, True, self.p1_result(), "gicp", Path(".")))
        self.assertFalse(results["lidar_profile"].ok)

    def test_missing_but_enabled_sensors_do_not_pass(self):
        cfg = base_config()
        sensors = [
            sensor("gnss_imu", "point_one_nav_atlas_duo", detected=False),
            sensor("lidar", "robin_w_front", detected=False),
        ]
        rtk = recorder.RtkResult(
            "rtk_fix", False, "no data", imu_samples=1)
        results = result_map(recorder.verify_requirements(
            cfg, sensors, True, rtk, "gicp", Path(".")))
        self.assertFalse(results["gnss_imu"].ok)
        self.assertFalse(results["lidar_profile"].ok)

    def test_glim_does_not_trust_generic_ok_without_fixed_evidence(self):
        cfg = base_config()
        sensors = [
            sensor("gnss_imu", "point_one_nav_atlas_duo"),
            sensor("lidar", "robin_w_front"),
        ]
        downgraded = recorder.RtkResult(
            "rtk_fix",
            True,
            "covariance-only",
            imu_samples=5,
            fixed_samples=0,
            fixed_quality=False,
        )
        results = result_map(recorder.verify_requirements(
            cfg, sensors, True, downgraded, "glim", Path(".")))
        self.assertFalse(results["gnss_imu"].ok)

    def test_every_profile_requires_live_adapter_imu(self):
        cfg = base_config()
        sensors = [
            sensor("gnss_imu", "point_one_nav_atlas_duo"),
            sensor("lidar", "robin_w_front"),
        ]
        rtk = recorder.RtkResult("rtk_fix", False, "no IMU")
        results = result_map(recorder.verify_requirements(
            cfg, sensors, True, rtk, "gicp", Path(".")))
        self.assertFalse(results["gnss_imu"].ok)

    def test_dual_antenna_requires_commissioning_and_runtime_quality(self):
        cfg = base_config()
        cfg["requirements"].update({
            "dual_antenna": True,
            "dual_antenna_commissioned": False,
            "tf_config_path": "tf.yaml",
            "min_antenna_baseline_m": 0.05,
            "max_heading_stddev_rad": 0.05,
        })
        cfg["requirements"]["profiles"]["glim"]["require_dual_antenna"] = True
        sensors = [
            sensor("gnss_imu", "point_one_nav_atlas_duo"),
            sensor("lidar", "robin_w_front"),
        ]
        rtk = recorder.RtkResult(
            "rtk_fix",
            True,
            "fixed",
            fix_status=2,
            fixed_samples=5,
            imu_samples=5,
            fixed_quality=True,
            heading_stddev=0.01,
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "tf.yaml").write_text(yaml.safe_dump({
                "static_transforms": [{
                    "child_frame_id": "gnss_antenna_secondary_link",
                    "translation": {"x": 1.0, "y": 0.0, "z": 0.0},
                }]
            }))
            results = result_map(recorder.verify_requirements(
                cfg, sensors, True, rtk, "glim", root))
            self.assertFalse(results["dual_antenna"].ok)

            cfg["requirements"]["dual_antenna_commissioned"] = True
            results = result_map(recorder.verify_requirements(
                cfg, sensors, True, rtk, "glim", root))
            self.assertTrue(results["dual_antenna"].ok)


class ProcessTests(unittest.TestCase):
    def test_missing_executable_raises(self):
        manager = recorder.ProcManager()
        with self.assertRaises(RuntimeError):
            manager.spawn(
                "missing", ["/definitely/not/a/real/hitch-command"])

    def test_bag_command_is_uncompressed(self):
        class Manager:
            command = None

            def spawn(self, _name, command, **kwargs):
                self.command = command
                kwargs["stdout"].close()

        with tempfile.TemporaryDirectory() as tmp:
            manager = Manager()
            recorder.start_bag(
                {"recording": {
                    "storage": "sqlite3",
                    "compression_mode": "none",
                }},
                Path(tmp),
                ["/gps_p1/imu"],
                manager,
            )
        self.assertNotIn("--compression-mode", manager.command)
        self.assertNotIn("--compression-format", manager.command)

    def test_bag_command_rejects_compression(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                recorder.start_bag(
                    {"recording": {
                        "storage": "sqlite3",
                        "compression_mode": "file",
                    }},
                    Path(tmp),
                    ["/gps_p1/imu"],
                    mock.Mock(),
                )

    def test_critical_early_exit_is_reported(self):
        manager = recorder.ProcManager()
        manager.spawn("fails", ["/bin/sh", "-c", "exit 7"])
        failures = manager.wait_for_startup(0.5)
        self.assertEqual(len(failures), 1)
        self.assertIn("exited(7)", failures[0])
        manager.stop_all()

    def test_bag_validation_requires_metadata_and_payload(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp)
            ok, _ = recorder.verify_bag_output(bag, "mcap")
            self.assertFalse(ok)
            (bag / "metadata.yaml").write_text(yaml.safe_dump({
                "rosbag2_bagfile_information": {
                    "message_count": 3,
                    "storage_identifier": "mcap",
                    "relative_file_paths": ["bag_0.mcap"],
                }
            }))
            (bag / "bag_0.mcap").write_bytes(b"data")
            ok, detail = recorder.verify_bag_output(bag, "mcap")
            self.assertTrue(ok)
            self.assertIn("3 messages", detail)

    def test_bag_validation_rejects_zero_message_bag(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp)
            (bag / "metadata.yaml").write_text(yaml.safe_dump({
                "rosbag2_bagfile_information": {
                    "message_count": 0,
                    "storage_identifier": "mcap",
                    "relative_file_paths": ["bag_0.mcap.zstd"],
                }
            }))
            (bag / "bag_0.mcap.zstd").write_bytes(b"header only")
            ok, detail = recorder.verify_bag_output(bag, "mcap")
            self.assertFalse(ok)
            self.assertIn("no messages", detail)

    def test_bag_validation_accepts_file_compressed_mcap(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp)
            (bag / "metadata.yaml").write_text(yaml.safe_dump({
                "rosbag2_bagfile_information": {
                    "message_count": 7,
                    "storage_identifier": "mcap",
                    "relative_file_paths": ["bag_0.mcap.zstd"],
                }
            }))
            (bag / "bag_0.mcap.zstd").write_bytes(b"compressed data")
            ok, detail = recorder.verify_bag_output(bag, "mcap")
            self.assertTrue(ok)
            self.assertIn("7 messages", detail)


class ClockTests(unittest.TestCase):
    @staticmethod
    def timestamp_sensor(kind="lidar", name="robin_w_front"):
        return recorder.SensorEntry(
            kind=kind,
            name=name,
            cfg={"topic": f"/{name}/data"},
            detected=True,
            enabled=True,
        )

    @staticmethod
    def stable_samples(*, age_ms=100.0, count=20, period_ms=100.0):
        base = 1_800_000_000_000_000_000
        samples = []
        for index in range(count):
            received = base + int(index * period_ms * 1_000_000)
            jitter_ms = (-1.0, 0.0, 1.0)[index % 3]
            stamp = received - int((age_ms + jitter_ms) * 1_000_000)
            samples.append((received, stamp))
        return samples

    @staticmethod
    def timestamp_config():
        return {
            "sync": {
                "sensor_timestamp": {
                    "capture_window_s": 5.0,
                    "min_samples": 10,
                    "max_future_ms": 5.0,
                    "max_drift_ms": 5.0,
                    "lidar": {
                        "max_age_ms": 250.0,
                        "max_jitter_ms": 30.0,
                    },
                    "camera": {
                        "max_age_ms": 150.0,
                        "max_jitter_ms": 30.0,
                    },
                }
            }
        }

    def test_stable_lidar_header_age_passes(self):
        sensor_entry = self.timestamp_sensor()
        verifier = recorder.SensorTimestampVerifier(
            self.timestamp_config(), [sensor_entry])
        verifier.samples[sensor_entry.name] = self.stable_samples()

        result = verifier.results()[0]

        self.assertTrue(result.ok)
        self.assertAlmostEqual(result.offset_ns / 1e6, 100.0, delta=0.1)

    def test_subscription_records_ros_stamp_against_realtime_receipt(self):
        lidar = self.timestamp_sensor()
        camera = self.timestamp_sensor("camera", "cam_front_left")
        verifier = recorder.SensorTimestampVerifier(
            self.timestamp_config(), [lidar, camera])
        callbacks = {}

        class FakeNode:
            def create_subscription(self, msg_type, topic, callback, qos):
                callbacks[topic] = (msg_type, callback, qos)
                return SimpleNamespace(topic=topic)

        point_cloud_type = object()
        image_type = object()
        qos = object()
        verifier.attach(FakeNode(), point_cloud_type, image_type, qos)
        msg = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1_800_000_000, nanosec=123_456_789)))

        receive_ns = 1_800_000_000_223_456_789
        with mock.patch.object(
                recorder.time, "time_ns", return_value=receive_ns):
            callbacks[lidar.cfg["topic"]][1](msg)

        self.assertIs(callbacks[lidar.cfg["topic"]][0], point_cloud_type)
        self.assertIs(callbacks[camera.cfg["topic"]][0], image_type)
        self.assertEqual(
            verifier.samples[lidar.name],
            [(receive_ns, 1_800_000_000_123_456_789)])
        self.assertEqual(verifier.samples[camera.name], [])

    def test_gross_epoch_error_fails(self):
        sensor_entry = self.timestamp_sensor()
        verifier = recorder.SensorTimestampVerifier(
            self.timestamp_config(), [sensor_entry])
        verifier.samples[sensor_entry.name] = self.stable_samples(
            age_ms=5000.0)

        result = verifier.results()[0]

        self.assertFalse(result.ok)
        self.assertIn("exceeds 250.0 ms", result.detail)

    def test_free_running_drift_fails(self):
        sensor_entry = self.timestamp_sensor()
        verifier = recorder.SensorTimestampVerifier(
            self.timestamp_config(), [sensor_entry])
        base = 1_800_000_000_000_000_000
        verifier.samples[sensor_entry.name] = [
            (
                base + index * 100_000_000,
                base + index * 100_000_000
                - int((100.0 + index) * 1_000_000),
            )
            for index in range(20)
        ]

        result = verifier.results()[0]

        self.assertFalse(result.ok)
        self.assertIn("edge-median drift", result.detail)

    def test_each_sensor_is_gated_independently(self):
        lidar = self.timestamp_sensor()
        camera = self.timestamp_sensor("camera", "cam_front_left")
        verifier = recorder.SensorTimestampVerifier(
            self.timestamp_config(), [lidar, camera])
        verifier.samples[lidar.name] = self.stable_samples(age_ms=100.0)
        verifier.samples[camera.name] = self.stable_samples(
            age_ms=300.0, period_ms=50.0)

        results = {result.name: result for result in verifier.results()}

        self.assertTrue(results[f"timestamp_{lidar.name}"].ok)
        self.assertFalse(results[f"timestamp_{camera.name}"].ok)
        self.assertIn(
            "exceeds 150.0 ms",
            results[f"timestamp_{camera.name}"].detail)

    def test_nonmonotonic_stamps_fail(self):
        sensor_entry = self.timestamp_sensor()
        verifier = recorder.SensorTimestampVerifier(
            self.timestamp_config(), [sensor_entry])
        samples = self.stable_samples()
        samples[10] = (samples[10][0], samples[9][1])
        verifier.samples[sensor_entry.name] = samples

        result = verifier.results()[0]

        self.assertFalse(result.ok)
        self.assertIn("not strictly increasing", result.detail)

    def test_missing_sensor_messages_fail(self):
        sensor_entry = self.timestamp_sensor()
        verifier = recorder.SensorTimestampVerifier(
            self.timestamp_config(), [sensor_entry])

        result = verifier.results()[0]

        self.assertFalse(result.ok)
        self.assertIn("need at least 10", result.detail)

    @staticmethod
    def chrony_output(ref_time):
        return (
            "Reference ID    : 4E4D4541 (NMEA)\n"
            f"Ref time (UTC)  : {ref_time:%a %b %d %H:%M:%S %Y}\n"
            "System time     : 0.000001000 seconds slow of NTP time\n"
        )

    def test_chrony_requires_a_fresh_gnss_reference(self):
        verifier = recorder.SyncVerifier({"sync": {
            "chrony_offset_ns": 2000000,
            "chrony_max_age_s": 30,
        }})
        output = self.chrony_output(datetime.now(timezone.utc))
        with mock.patch.object(recorder.shutil, "which", return_value="/chronyc"), \
                mock.patch.object(
                    recorder.subprocess, "run",
                    return_value=SimpleNamespace(stdout=output)):
            result = verifier.check_chrony()
        self.assertTrue(result.ok)

    def test_chrony_rejects_a_stale_reference(self):
        verifier = recorder.SyncVerifier({"sync": {
            "chrony_offset_ns": 2000000,
            "chrony_max_age_s": 30,
        }})
        output = self.chrony_output(
            datetime.now(timezone.utc) - timedelta(minutes=2))
        with mock.patch.object(recorder.shutil, "which", return_value="/chronyc"), \
                mock.patch.object(
                    recorder.subprocess, "run",
                    return_value=SimpleNamespace(stdout=output)):
            result = verifier.check_chrony()
        self.assertFalse(result.ok)


if __name__ == "__main__":
    unittest.main()
