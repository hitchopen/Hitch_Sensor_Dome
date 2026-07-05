# Atlas Adapter

The Atlas adapter is a **normalization boundary** for Point One Atlas GNSS/INS
input. It keeps sensor-specific WGS84 parsing, RTK covariance gating, and
Point One P1 time mapping at the edge so downstream mapping (GLIM++) and
localization (GICP++) can consume a stable, vendor-neutral contract:
`/gps_p1/*` (and optional `/gnss*`) already in the **local ENU** `map` frame.
LiDAR topics are **not** touched by the adapter — multi-LiDAR merge /
timestamp handling stays owned by GLIM++/GICP++ `lidar_concat`.

On the Hitch Sensor Dome this package is **optional** — the dome's default
driver chain (`/imu/data`, `/gps/fix`, `/pose`, and the `nav_sat_gated_odom`
republisher's `/odom_rtk_only`) already feeds GLIM++/GICP++ directly. See
[README_HITCH_PORT.md](README_HITCH_PORT.md) for when the adapter is the
better choice, and the [repo root README](../README.md) for the dome's
default pipeline.

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

`adapter` builds against Point One's `fusion_engine_msgs` package. That package
is not released as a binary for every supported ROS 2 distro, so a clean
machine needs the Point One driver source in the same colcon workspace before
building this package:

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}   # use humble on Ubuntu 22.04 hosts
cd ~/ros2_ws/src
git clone https://github.com/PointOneNav/ros2-fusion-engine-driver.git
cd ~/ros2_ws
rosdep install -i --from-paths src --rosdistro "$ROS_DISTRO" -y --skip-keys fusion_engine_msgs
colcon build --packages-up-to adapter
```

The default Hitch Sensor Dome pipeline can run without `adapter`; this package
is only needed when you want the normalized `/gps_p1/*` compatibility contract
described below.

## Inputs

- `pose_input_topic` (default `/atlas/pose_filtered`): Atlas FusionEngine
  WGS84 pose.
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
- `/gps_p1/imu` (`sensor_msgs/msg/Imu`): retimed Atlas IMU.

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

Exactly one origin source must be configured:

- `local_enu_origin: "lat,lon,alt"`
- `local_enu_origin_ttl_path: "/path/to/ttl.csv"`

The TTL parser reads the first non-empty CSV row and uses its last three
fields as `(lat, lon, alt)`. The checked-in default origin is a
**placeholder from an earlier deployment** — set `local_enu_origin` to your
own deployment datum before mapping. The one hard constraint is a **single
shared datum**: the GLIM++ map, the seed odometry, and GICP++ must all use
the origin the adapter defines, or the frames silently disagree.

## Example

```bash
# Live Atlas topics, deployment datum on the command line:
ros2 launch adapter adapter.launch.py \
  use_p1_imu_pcap:=false \
  local_enu_origin:="<lat_deg>,<lon_deg>,<alt_m>"

# PCAP IMU replay:
ros2 launch adapter adapter.launch.py \
  p1_imu_pcap_path:=/path/to/ins_*.pcap \
  local_enu_origin_ttl_path:=/path/to/ttl.csv
```

The launch file overrides the YAML default origin when
`local_enu_origin_ttl_path` is passed. The node fails at startup if both origin
sources are set or both are empty.
