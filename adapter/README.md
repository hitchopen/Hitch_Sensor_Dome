# Atlas Adapter

The Atlas adapter is a **normalization boundary** for Point One Atlas GNSS/INS
input. It keeps sensor-specific WGS84 parsing, RTK covariance gating, and
Point One P1 time mapping at the edge so downstream mapping (GLIM++) and
localization (GICP++) can consume a stable, vendor-neutral contract:
`/gps_p1/*` (and optional `/gnss*`) already in the **local ENU** `map` frame.
LiDAR topics are **not** touched by the adapter — multi-LiDAR merge /
timestamp handling stays owned by GLIM++/GICP++ `lidar_concat`.

On the Hitch Sensor Dome this package is the production GNSS boundary for
GLIM++ mapping: `/gps_p1/filtered_odom_rtk_fixed` preserves FusionEngine's
authoritative `solution_type == kRtkFixed` decision, which REP-145
`NavSatFix` cannot express. `/gps_p1/fix` supplies a synchronized WGS84,
freshness, and covariance cross-check for that same sample. GICP++ may still
use its separate compatibility gate. See [README_HITCH_PORT.md](README_HITCH_PORT.md) for this package's role on
the dome; the frame the adapter publishes into is defined by
[`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml).

The normalization contract:

```text
Atlas pose:  WGS84 LLA
adapter:     LLA -> local ENU using one configured origin
/gnss:       PoseWithCovarianceStamped in map/local ENU
GLIM/GICP:   consume local ENU directly
```

The adapter does not publish a map-frame bridge topic from a transform
sidecar. `map` is the local ENU frame.

## External Message Dependency

`adapter` builds against Point One's native `fusion_engine_msgs` package and
requires a driver that publishes `fusion_engine_msgs/msg/Pose`. Obtain that
native-message ROS package/driver from the Atlas deployment or Point One
support and place it in the same colcon workspace before building. The public
generic `ros2-fusion-engine-driver` publishes `geometry_msgs/PoseStamped`;
installing it alone does not satisfy this contract.

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}   # use humble on Ubuntu 22.04 hosts
cd ~/ros2_ws
rosdep install -i --from-paths src --rosdistro "$ROS_DISTRO" -y --skip-keys fusion_engine_msgs
colcon build --packages-up-to adapter
```

The strict GLIM++ production profile requires this adapter output. A
LiDAR-only or raw-Pose compatibility profile can run without it, but must not
be described as Fixed-only GNSS mapping.

## Inputs

- `pose_input_topic` (default `/atlas/pose_filtered`): Atlas FusionEngine
  WGS84 pose as `fusion_engine_msgs/msg/Pose`. A generic
  `geometry_msgs/PoseStamped` publisher is not compatible because it has
  already discarded the native solution type and covariance.
- `imu_input_topic` (default `/atlas/imu_calibrated`): Atlas IMUOutput
  stream.
- Optionally, a Point One PCAP for IMU replay
  (`scripts/p1_imu_pcap_replay_node.py`).

## Outputs

- `/gnss` (`geometry_msgs/msg/PoseWithCovarianceStamped`): continuous Atlas INS
  pose in local ENU, `header.frame_id="map"`.
- `/gnss_rtk_fixed` (`geometry_msgs/msg/PoseWithCovarianceStamped`): same pose
  type, only when the configured covariance/RTK gate passes.
- `/gps_p1/filtered_odom` (`nav_msgs/msg/Odometry`): compatibility odom with
  the same local ENU pose; `child_frame_id` = `body_frame_id` (default
  `imu_link`, the Atlas Duo Center of Navigation in
  [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml)).
- `/gps_p1/filtered_odom_rtk_fixed` (`nav_msgs/msg/Odometry`): gated
  compatibility odom.
- `/gps_p1/fix` (`sensor_msgs/msg/NavSatFix`): synchronized WGS84 gate
  message. Exact adapter RTK-Fixed admission maps to `STATUS_GBAS_FIX`;
  other valid solution classes map below it, and invalid covariance is
  published as `COVARIANCE_TYPE_UNKNOWN`.
- `/gps_p1/imu` (`sensor_msgs/msg/Imu`): retimed Atlas IMU.
- `/gps_p1/local_enu_origin` (`std_msgs/msg/String`): canonical
  `lat,lon,alt` datum metadata, reliable and transient-local for GLIM/GICP
  frame-contract validation.

Set `publish_gnss_pose=false` when a prep/recording pipeline only needs the
`/gps_p1/*` compatibility streams and should not create `/gnss*` publishers.

## IMU frame

`/gps_p1/imu` is stamped with `imu_frame_id` (default `imu_link`, matching the
dome TF). The adapter overwrites the incoming `frame_id`, so the PCAP replay
node's placeholder (`p1_imu_pcap_frame_id`, default `cg`) is relabeled to
`imu_frame_id` on republish. This relabel is **correct and safe** — verified
against the Point One FusionEngine Message Specification v0.21, §3.4.1
IMUOutput (11000):

> "corrected for estimated accelerometer and gyro errors, including biases and
> scale factors, and has been **rotated into the vehicle body frame** from the
> original IMU orientation."

Key point: IMUOutput is **rotated** into body axes but **not**
lever-arm-projected (the spec keeps Device 0x10, GNSS 0x12, and Output 0x13
lever arms as distinct config items; the Output Lever Arm re-points the
*pose/position* output, not the IMU accel stream). The live
`/atlas/imu_calibrated` topic carries this **same** IMUOutput(11000) message,
so the live and PCAP IMU streams are the identical physical quantity
(device-located, body-axis-rotated). The relabel is therefore consistent
live-vs-replay — not a mismatch.

Residual modeling note (shared by GLIM++ + GICP++, **not** adapter-specific):
tagging the device-located IMU accelerometer with the navigation-point frame
under an identity IMU→base transform drops the small device→reference
accelerometer lever-arm term (`ω×(ω×r)+α×r`). This is negligible at slow
mapping speeds (~0.2 m/s² at 0.5 rad/s with a sub-metre arm) and is absorbed
by the accel-bias estimator plus scan matching; the gyro is unaffected (rigid
body). Model it only if pushing high-yaw-rate segments.

## Origin

Configure **exactly one** origin source:

- `local_enu_origin: "lat,lon,alt"`
- `local_enu_origin_ttl_path: "/path/to/ttl.csv"`

The TTL parser reads the first non-empty CSV row and uses its last three
fields as `(lat, lon, alt)`. Neither source has a built-in deployment default:
missing or conflicting sources are startup errors. The retired
`39.58227391,-86.74232215,260.4` value is rejected even when supplied
explicitly; set `allow_legacy_local_enu_origin=true` only when operating at
that exact earlier site.

The one hard constraint is a **single shared datum**: the GLIM++ map, seed
odometry, and GICP++ must all use the origin the adapter defines, or their
frames silently disagree.

## Example

```bash
# Live Atlas topics, deployment datum on the command line:
ros2 launch adapter adapter.launch.py \
  use_sim_time:=false \
  use_p1_imu_pcap:=false \
  pose_input_topic:=/atlas/pose_filtered \
  imu_input_topic:=/atlas/imu_calibrated \
  local_enu_origin:="<lat_deg>,<lon_deg>,<alt_m>"

# PCAP IMU replay:
ros2 launch adapter adapter.launch.py \
  p1_imu_pcap_path:=/path/to/ins_*.pcap \
  local_enu_origin_ttl_path:=/path/to/ttl.csv
```

The launch file overrides the YAML inline origin when
`local_enu_origin_ttl_path` is passed. The node rejects both sources being set
and refuses to start when neither source is supplied.

## Run summary and audit counters

The adapter logs (and optionally writes via `summary_output_path`) a one-line
summary designed so a run report can **prove** zero data loss instead of
inferring it from matching in/out totals:

```text
pose_in=… navsat_fix_out=… gnss_out=… gnss_rtk_out=… odom_out=… rtk_out=… imu_in=… imu_out=…
pose_dropped_invalid=… imu_dropped_invalid_stamp=… imu_sidecar_miss_drop=… imu_dropped_clock_not_ready=…
p1_clock_ready=… p1_clock_drift_ms=… p1_clock_reset_count=…
```

- `pose_dropped_invalid` — NaN / invalid-solution FusionEngine poses rejected
  before publication (cold-start samples land here); also counts poses dropped
  for an invalid, quarantined-forward-spike, or inconsistent P1/arrival pair.
- `imu_dropped_invalid_stamp` — IMU samples dropped at ingest for a non-finite /
  ≤0 / ≥4e9 (sentinel) header stamp, before they can reach a consumer buffer.
- `imu_sidecar_miss_drop` — IMU samples dropped because no sidecar P1 stamp
  matched within tolerance (sidecar replay mode only).
- `imu_dropped_clock_not_ready` — IMU samples dropped from the bounded
  not-ready queue before the P1→ROS clock mapper initialized.
- `p1_clock_ready` / `p1_clock_drift_ms` / `p1_clock_reset_count` —
  end-to-end P1→ROS retiming
  evidence: the mapper reached readiness, and the measured offset drift over
  the current epoch. A reset count above zero records an accepted device/bag
  epoch change that would otherwise make a fresh zero-drift envelope look
  identical to a clean uninterrupted run. Together with GLIM's
  `gnss_global summary` line (bracket widths,
  gap/non-monotonic rejections, factor counts) these close the RTK timing
  audit chain from PCAP to map factors.

All four drop counters (`pose_dropped_invalid`, `imu_dropped_invalid_stamp`,
`imu_sidecar_miss_drop`, `imu_dropped_clock_not_ready`) are expected to be **0**
on a healthy run; the sidecar match/miss/skip triple is additionally printed in
sidecar mode.

## Startup validation (fail loud, not degraded)

Bad parameter overrides refuse to start rather than silently degrading
retiming: `nominal_imu_period_sec`, `imu_flush_timeout_sec`,
`p1_like_threshold_sec`, `imu_p1_sidecar_match_tolerance_sec`, and
`pose_max_forward_jump_sec` must be finite and positive (a zero flush timeout strands the arrival-retime queue; a
nonpositive nominal period breaks synthesized spacing). The RTK gate
covariance thresholds require finite, nonnegative values per sample — the
same contract the downstream GICP gate and the legacy
`rtk_fixed_odom_filter.py` now enforce. The PCAP replay node also rejects IMU
samples whose `fraction_ns >= 1e9` (a corrupt fraction would otherwise raise
on ROS timestamp assignment or alias into a wrong stamp), alongside the
existing `0xFFFFFFFF` sentinel rejection. The adapter also requires a
non-empty `navsat_fix_topic` and exactly one valid local-ENU origin source;
the retired origin placeholder needs an explicit acknowledgment.
