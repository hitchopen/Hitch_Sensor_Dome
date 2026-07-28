# Recording — ROS 2 MCAP Capture + Foxglove Live Dashboard

A Python orchestrator that read-only verifies the live Atlas boundary and the
result of the GPS → chrony → PTP clock chain, launches every selected
LiDAR/camera publisher, records GNSS / IMU / LiDAR / camera data into an MCAP
rosbag, and serves a Foxglove dashboard.

This folder is the run-time companion to the setup scripts in
[`../PTP_sync/`](../PTP_sync/) and the static-TF definitions in
[`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml).
**`PTP_sync/` exclusively owns clock configuration and synchronization: gpsd,
chrony, ptp4l, phc2sys, and every sensor's PTP settings. The recorder never
changes that state.** It only verifies the resulting host and ROS timestamps
and refuses to record when the configured policy fails.

**Cameras are optional.** A camera-free session is fully supported: missing
RouteCAMs are left disabled, create no driver process or timestamp subscription,
and are not part of either profile's requirements. GICP requires P1/Atlas plus
the front Robin W and records either rear unit when present. GLIM requires
P1/Atlas plus all three Robin W units. RTK and heading quality gates remain
profile-specific requirements in addition to those hardware inputs.

The Atlas native-message driver and [`adapter/`](../adapter/) are external
prerequisites. Start both before the recorder. The recorder requires live,
finite `/gps_p1/imu` data for every profile; the `glim` profile additionally
requires `/gps_p1/fix` plus authoritative
`/gps_p1/filtered_odom_rtk_fixed`. The generic open-source
`fusion_engine_ros_driver` publishes `PoseStamped` and cannot replace the
native `fusion_engine_msgs/msg/Pose` source because it discards the solution
class and covariance used by the RTK gate.

## Architecture

```
                       sensor_recorder.py
                              │
       ┌──────────────────────┼──────────────────────┐
       ▼                      ▼                      ▼
   detection             sync verify             session output
  ─────────────       ─────────────────        ───────────────
   probe sensors      chrony + direct           recording/data/
   show checklist     timestamp gate            session_<ts>/
       │                      │                      ▲
       └──── confirm ────────┘                       │
                              │                      │
                              ▼                      │
                   ┌──────── spawned ─────────┐      │
                   │  static_transform        │      │
                   │  seyond × N (one each)   ├──────┤
                   │  camera_aravis2 × M      │      │
                   │  rate_monitor.py         │      │
                   │  ros2 bag record -s mcap │──────┘
                   │  foxglove_bridge :8765   │
                   └──────────┬───────────────┘
                              │ ws://localhost:8765
                              ▼
                       Foxglove Studio
                  (live dashboard + replay)
```

Each managed LiDAR/camera driver, the static-TF publisher, and rosbag2 are
critical processes. If one exits, the recorder stops, flushes the bag, marks
the session failed, and returns nonzero. The rate monitor and Foxglove bridge
are noncritical visualization helpers.

The Seyond driver installed by `PTP_sync/4_setup_lidar_ptp.sh` is pinned and
source-validated at build time to publish the repository contract directly:
`timestamp/FLOAT64/count=1`, numeric Unix seconds. The driver hydrates each
compact packet-relative point offset with its absolute packet start time
before PCL creates PointCloud2; no recording-side conversion is required.

## Running It

```bash
# Default GICP capture: front LiDAR minimum, live adapter IMU required
python3 recording/sensor_recorder.py

# Production GLIM map capture: three LiDARs + RTK-fixed + commissioned heading
python3 recording/sensor_recorder.py --profile glim

# Headless: no input prompt; hard requirements still apply
python3 recording/sensor_recorder.py --headless --yes

# Run every preflight and requirement check, but create no bag
python3 recording/sensor_recorder.py --dry-run
```

What happens on a normal run:

Run this from a shell where ROS 2 Humble or Jazzy and your colcon workspace are sourced.
The recorder stays completely unprivileged and requires no elevated access. It
starts the selected LiDAR/camera drivers before opening a bag, then measures
each live `msg.header.stamp` directly against host `CLOCK_REALTIME`.

Before starting the recorder, launch the deployment's Atlas ROS driver and
confirm that it publishes these exact input contracts:

```bash
ros2 topic type /atlas/pose_filtered
# fusion_engine_msgs/msg/Pose
ros2 topic type /atlas/imu_calibrated
# sensor_msgs/msg/Imu
```

Then start the adapter in a separate sourced terminal with exactly one
deployment origin:

```bash
ros2 launch adapter adapter.launch.py \
  use_sim_time:=false \
  use_p1_imu_pcap:=false \
  pose_input_topic:=/atlas/pose_filtered \
  imu_input_topic:=/atlas/imu_calibrated \
  local_enu_origin:="<lat_deg>,<lon_deg>,<alt_m>"
```

1. **Detect.** The script probes the Atlas Duo, Robin W LiDARs, and RouteCAM cameras listed in `sensor_config.yaml`, then prints a checklist tagged `[FOUND]` or `[MISSING]`. Press `ENTER` to record from everything found, or type space-separated indices to toggle items off.

2. **Verify sync.** During the same five-second ROS spin as the Atlas preflight,
   every selected LiDAR and camera is subscribed independently. The recorder
   computes `CLOCK_REALTIME - msg.header.stamp` at callback receipt and rejects
   missing, zero, nonmonotonic, future, stale, unstable, or drifting streams.
   The age includes framing, network, and driver latency, so LiDAR and camera
   limits are configured separately under `sync.sensor_timestamp`. The same
   driver processes continue into bag recording after the gate passes. With
   `requirements.clock_sync: true`, any sensor failure is fatal.
   This step is strictly observational: it does not enable PTP, discipline a
   clock, restart a time service, or repair a failed setup. Run the appropriate
   `PTP_sync/` setup or troubleshooting procedure when it fails.

3. **Verify Atlas data.** Every profile must produce live normalized IMU
   samples. `glim` must also pass the Fixed-only GNSS gate and the explicit
   dual-antenna commissioning/runtime-quality checks.

4. **Record.** LiDAR/camera drivers, bag recorder, rate monitor, and
   `foxglove_bridge` start. Press `H` for health and `Q` or Ctrl+C to flush.

5. **Validate.** Critical child failure stops the session. After shutdown the
   recorder requires parseable metadata, a positive message count, and a
   nonempty payload in the selected storage format before returning zero;
   final process statuses and the verdict are written to
   `session_metadata.json`.

## Foxglove

While the recorder is running:

1. Open Foxglove Studio.
2. **Open Connection → Foxglove WebSocket → `ws://localhost:8765`**.
3. **Layouts → Import from file → `recording/foxglove/sensor_dome_layout.json`**.

The layout shows the three Robin W point clouds superimposed in `imu_link` (resolved through `/tf_static` from `../config/sensor_dome_tf.yaml`), the four camera views, a GNSS map that follows `/gps_p1/fix`, an IMU plot, a fix-status indicator, and a Diagnostics panel bound to `/sensor_dome/rates` that lights up yellow / red when a sensor falls below its expected Hz.

The shipped configuration records uncompressed MCAP to minimize capture CPU
load and write latency. Open the recorded `.mcap` directly in Foxglove
(`File → Open local file`) and apply the same layout.
`recording.compression_mode` must remain `none`; the recorder rejects other
values rather than enabling online compression.

## Output

Sessions land in `recording/data/` by default (configurable via `--output` or `recording.output_dir` in the YAML):

```
recording/data/session_20260425_103022/
├── rosbag2/
│   ├── metadata.yaml
│   └── rosbag2_0.mcap        # selected sensors + TF + diagnostics
├── logs/                     # per-driver stdout/stderr
├── rosbag2.log
├── foxglove_bridge.log
└── session_metadata.json     # sensors, sync results, topics, foxglove URL
```

Sessions are gitignored; the `data/` folder itself is tracked so the path stays stable.

## Session requirements

Some things are preconditions for a usable session, not preferences: a bag
missing them looks valid on disk and only fails later, at map-build or
localization time, when the drive can no longer be repeated. But *which* things
depends on what the recording is for. The default GICP profile still requires
the front Robin W, live P1 adapter IMU, and synchronized clocks; either or both
rear LiDARs may also be recorded. GLIM requires all three LiDARs and adds the
mapping-specific requirements below. Neither profile requires a camera.

**Always required, every profile:**

| Requirement | Verified by | Why it cannot be optional |
|---|---|---|
| **Atlas Duo present and live** | FusionEngine TCP probe plus valid `/gps_p1/imu` samples | It is the IMU source for both pipelines. A bag without it is unusable by either. |
| **Clock synchronization** | read-only fresh chrony check plus independent `header.stamp` age/stability checks for every selected LiDAR and camera | Every sweep merge and GNSS association is computed from capture time. Configuration and repair belong to `PTP_sync/`. |
| **The profile's LiDARs** | per-sensor TCP probe | See below. |
| **Cameras** | Not required; detected cameras are recorded opportunistically | GLIM++ and GICP++ do not consume camera data. |

**Required only by the profile that needs them:**

| Requirement | `gicp` | `glim` | Why the split |
|---|---|---|---|
| LiDAR units | front required; zero, one, or two rears optional | all three | GLIM++ builds the offline map from all three. GICP++ runs `front_only` by default and can record extra rear streams without requiring them. |
| RTK-fixed at start | not required | required | GLIM++ will not initialize without a validated fix — the map origin must be globally referenced from the first factor. GICP++ localizes against an already-built map from LiDAR + IMU. |
| Dual-antenna heading | not required | required | Without a second antenna heading is gyro-integrated and drifts without bound, which is what produces multi-lap yaw and z drift in a map. GICP++ is not accumulating a map, so it is unaffected. |

```bash
# Default. Collects GICP++ data with the front Robin W and live adapter IMU.
python3 recording/sensor_recorder.py

# Capturing for a GLIM++ map. Enforces three LiDARs, RTK-fixed, and
# dual-antenna heading before a single message is recorded.
python3 recording/sensor_recorder.py --profile glim
```

Recording with only the front LiDAR, without an RTK fix, or on a rig whose
second antenna is not yet fitted is a legitimate GICP++ data-collection run and
is **not** an error. The profile is written into `session_metadata.json`, so a
later consumer can tell what the bag was captured for.

The dual-antenna check under `glim` has three parts. The secondary entry in
`../config/sensor_dome_tf.yaml` must be off its `(0,0,0)` sentinel; commissioning
must directly observe Atlas `HeadingOutput`/`RawHeadingOutput` with
`solution_type=RTKFixed` and record that fact by setting
`requirements.dual_antenna_commissioned: true`; and live Fixed-only odometry
must carry a finite, tight yaw σ. The last value checks current quality only.
General INS yaw covariance cannot prove that two antennas produced it.

Mode flags cannot downgrade a requirement. Clock synchronization remains hard
while `requirements.clock_sync=true`, and a profile with
`require_rtk_fixed=true` rejects `--skip-rtk-check` and `--rtk-mode log`. On a
supported non-RTK profile such as `gicp`, `rtk.mode` controls only how the
remaining GNSS quality report is handled: `--rtk-mode hard` hard-gates finite
position and the configured covariance threshold, but it does **not** require
RTK Fixed status or the Fixed-only odometry stream.

## See Also

- [`../PTP_sync/README.md`](../PTP_sync/README.md) — one-time install of `gpsd` + `chrony` + `ptp4l` + `phc2sys` and per-sensor PTP enablement
- [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) — source of truth for every `imu_link → sensor_link` static transform
- [`../3D files/README.md`](../3D%20files/README.md) — mechanical design, sensor layout, and BOM
