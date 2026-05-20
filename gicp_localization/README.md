# GICP Localization for Hitch Sensor Dome

GICP scan-to-map localization with IMU dead-reckoning and optional RTK-driven recovery. Locks onto a pre-built PCD map produced by GLIM_plusplus offline mapping and publishes pose at IMU rate. Targets the Hitch Sensor Dome hardware: 3× Seyond Robin W LiDARs concatenated into a single primary frame, plus a Point One Nav Atlas Duo INS for IMU + RTK-fixed GNSS.

This package began as an adaptation of [VECTR's Direct LiDAR-Inertial Odometry](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) and keeps the VECTR copyright headers on the files that originated there as legal attribution.

## Pipeline overview

```
3× Robin W (front + rear-L + rear-R) ──→ lidar_concat ──→ deskew ──→ GICP scan-to-map ──→ geometric observer
       │                                                                                       ▲
       │                                                                                       │
Atlas Duo IMU (/imu/data) ────────────────────────── motion prior ─────────────────────────────┘
                                                                                               │
Atlas Duo INS (/odom) + NavSatFix (/gps/fix) ──→ nav_sat_gated_odom ──→ /odom_rtk_only ────────┘
                                                                       (cross-check + snap recovery)
```

## Features

- **GICP scan-to-map matching** against a single pre-built PCD map (no submap stitching at runtime).
- **IMU + LiDAR fusion**: IMU integrates a motion prior between scans; GICP refines; a geometric observer fuses the two and propagates pose at IMU rate (~100 Hz).
- **Multi-LiDAR concatenation** (`lidar_concat`): subscribes to the two rear Robin W aux topics, time-aligns to the front primary, transforms via URDF, and concatenates per-point timestamps onto the primary clock. Mirror of GLIM_plusplus's `lidar_concat`.
- **Layered GICP rejection gates**:
  - Hard fitness reject (`gicp/fitnessRejectThreshold`).
  - Combined geometric-degeneracy gate (`hessianCondMax` AND any of `fitness`/`trans`/`rot` warn floors) — catches optimizer slides on feature-poor corners without over-rejecting healthy edges.
  - Large-jump reject vs. the IMU-predicted prior.
- **IMU dead-reckoning fallback**: any non-accepted scan falls back to the IMU-integrated prior instead of freezing at the last accepted pose, so transient corner failures don't cascade.
- **RTK-gated ground-truth cross-check** (optional): subscribes to a `gt_odom` topic (default `/odom_rtk_only` — the Atlas Duo INS odometry gated on `/gps/fix` STATUS_GBAS_FIX). Reports per-scan `gt_err=[trans, rot, dt]` in debug. Diagnostic only — never feeds back into accept/reject decisions.
- **RTK-driven pose recovery** (optional): when GICP fails for N consecutive scans, snap pose + velocity to a time-matched RTK-gated INS sample so GICP can re-acquire from a known-good state.
- **RTK-driven IMU calibration**: uses the RTK-gated INS topic as the truth source for bias estimation, letting the vehicle calibrate while moving. Falls back to stationary calibration if no GT message arrives within the timeout.
- **UTM-frame output** (optional): if `T_world_utm.txt` from GLIM_plusplus is provided, publish pose / odom / path in a `utm` frame alongside `map`.

## Dependencies

- ROS 2 Humble or Jazzy
- PCL, Eigen3, OpenMP, nlohmann::json
- A vendored `nano_gicp` ships inside this package; no external `direct_lidar_inertial_odometry` dependency is required.
- For development: matplotlib (debug-script plots).

## Building

From the colcon workspace root:

```bash
colcon build --packages-select gicp_localization --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Prerequisites

1. **A PCD map** produced by GLIM_plusplus offline mapping. The map file path goes into `localization/map_path` in [`cfg/localization.yaml`](cfg/localization.yaml) (or via the `map_path:=` launch arg). For UTM-frame output, also point `localization/utm_transform_path` at the `T_world_utm.txt` from the same GLIM dump.

2. **The sensor URDF** at `GLIM_plusplus/config/sensor_dome.urdf`. Generate it once with:

   ```bash
   cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py
   ```

   The localization launch file walks up from itself to find this URDF automatically.

3. **An RTK-gated INS odometry topic** at `/odom_rtk_only` if you want the recovery / cross-check features. Run the `nav_sat_gated_odom` helper (see [Recovery and cross-check](#recovery-and-cross-check) below). Without it, set `localization/gt_odom/enable: false` and `localization/gt_recovery/enable: false` and run pure scan-to-map.

## Running

```bash
ros2 launch gicp_localization localization_with_tf.launch.py \
    rviz:=true \
    pointcloud_topic:=/robin_w_front/points \
    imu_topic:=/imu/data \
    gt_odom_topic:=/odom_rtk_only \
    map_path:=$PWD/maps/session_latest/map.pcd
```

The defaults already target the Hitch dome — `pointcloud_topic`, `imu_topic`, `gt_odom_topic`, and `child_frame` all match `recording/sensor_config.yaml` out of the box. Override at launch time only when you need to.

### Launch arguments

| Arg | Default | Purpose |
|---|---|---|
| `rviz` | `false` | Start RViz with the bundled config. |
| `pointcloud_topic` | `/robin_w_front/points` | Primary Robin W topic (gets remapped to `pointcloud`). |
| `imu_topic` | `/imu/data` | Atlas Duo IMU topic. Watch for typos — the launch arg is `imu_topic` (underscore), not `imu-topic`. |
| `odom_topic` | `/odom` | Pose-init odom topic when `localization/use_odom_init=true` and not bootstrapping from RTK. |
| `gt_odom_topic` | `/odom_rtk_only` | RTK-gated INS odometry. Used when `localization/gt_odom/enable=true` and/or `localization/gt_recovery/enable=true`. |
| `imu_only` | `false` | Disable GICP and propagate pose from IMU only (debug / sanity check). |
| `urdf_path` | auto-found | Override the URDF auto-walk. By default looks for `GLIM_plusplus/config/sensor_dome.urdf`. |
| `parent_frame` / `child_frame` | `base_link` / `lidar_front_link` | Used by the bundled static-TF helper. |
| `map_path` | (from YAML) | Override `localization/map_path` from the command line. |

### Initial pose

Three options, in priority order:

1. **`localization/gt_odom/enable: true` + `localization/use_odom_init: true`** (default in the shipped YAML): the first RTK-gated INS message bootstraps the pose. Works for any bag start-offset without hand-tuning numbers.
2. **`localization/initial_pose/use: true`**: use the numeric `x/y/z/roll/pitch/yaw` from the YAML. The `frame: "lidar"` mode is convenient for pasting from GLIM_plusplus's `traj_lidar.txt` first row — the node post-multiplies `inv(T_base_lidar)` automatically.
3. **RViz "2D Pose Estimate"**: publish to `/initialpose`. Always available as a manual override.

## Configuration

All parameters live in [`cfg/localization.yaml`](cfg/localization.yaml). The YAML has inline comments explaining each knob; the cheat sheet below covers the parts most worth tuning.

### Frames

```yaml
localization/map_frame:   "map"
localization/base_frame:  "base_link"         # Vehicle body frame (overridable)
localization/imu_frame:   "imu_link"          # Atlas Duo CoN — fixed by hardware
localization/lidar_frame: "lidar_front_link"  # Primary Robin W (front)
```

These all reference links in `sensor_dome.urdf`, which is in turn generated from `config/sensor_dome_tf.yaml`.

**Mapping vs. localization frames.** The map produced by GLIM_plusplus is anchored at `imu_link` (the Atlas Duo Center of Navigation at session start). This localization node reports pose in `base_link` — the ROS standard "robot body" frame that downstream consumers (controllers, planners, RViz robot models) expect. The two are bridged by a static `imu_link → base_link` transform in `config/sensor_dome_tf.yaml`. By default it's identity (`base_link` physically coincident with `imu_link`), so a fresh Hitch dome install can ignore the distinction.

**Configuring base_link for a non-trivial vehicle.** Edit the `imu_link → base_link` entry in `config/sensor_dome_tf.yaml` and regenerate the URDF:

```yaml
# Example: vehicle body frame at rear axle, 1.2 m behind the dome,
# 0.4 m below the dome's mounting plane.
- frame_id: "imu_link"
  child_frame_id: "base_link"
  translation: { x: -1.200, y: 0.000, z: -0.400 }
  rotation:    { x: 0.000, y: 0.000, z: 0.000, w: 1.000 }
```

```bash
cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py
```

After regenerating, the localizer's `localized_pose` / `localized_odom` outputs land in the new `base_link` frame on the next launch. **The map itself does not need to be rebuilt** — it stays anchored at `imu_link`, and the localizer applies the new `T_base_imu` at publish time. This is the whole point of routing the body-frame translation through TF rather than baking it into the map.

### GICP rejection gates

```yaml
gicp/fitnessRejectThreshold: 1.0       # hard reject: fitness > threshold
gicp/rejectLargeJumps: true            # reject if pose jumps > debug_jump_*
gicp/hessianCondMax: 5.0e9             # condition-number floor for combined gate
gicp/hessianFitnessWarnThreshold: 0.15 # OR trigger: fitness elevated
gicp/hessianTransWarnM: 1.0            # OR trigger: GICP correction > X m
gicp/hessianRotWarnDeg: 1.5            # OR trigger: GICP correction > X deg
```

The combined hessian gate fires when condition number is high AND any of the three warn floors is crossed. High hessian alone is harmless when GICP barely moved (good IMU prior, degenerate but well-anchored geometry); large corrections in degenerate geometry are the slide signature. Set any threshold ≤ 0 to disable that OR branch. The defaults in the YAML are conservative; tighten once you have a Hitch-dome run with ground-truth-fixed accepts.

### Multi-LiDAR concatenation

```yaml
localization/lidar_concat/enabled: true
localization/lidar_concat/aux_topics: ["/robin_w_rear_left/points", "/robin_w_rear_right/points"]
localization/lidar_concat/aux_frames: ["lidar_rear_left_link", "lidar_rear_right_link"]
localization/lidar_concat/time_threshold: 0.05  # seconds; drop aux scans farther than this from primary
```

### IMU + observer

```yaml
dlio/deskew: true              # Robin W keeps real per-point times — deskewing helps.
dlio/imu/bufferSize: 2000
dlio/imu/calibTime: 0.5        # Stationary fallback when RTK init unavailable.

odom/geo/Kp:  4.5    # Position correction gain
odom/geo/Kv:  11.25  # Velocity
odom/geo/Kq:  4.0    # Orientation
odom/geo/Kab: 2.25   # Accel bias
odom/geo/Kgb: 1.0    # Gyro bias
```

### Sensor type

```yaml
localization/sensor_type: "velodyne"
```

The Seyond Robin W in `coordinate_mode:=3` publishes per-point time as float32 seconds relative to scan start — identical to the Velodyne convention. We deliberately set `sensor_type: "velodyne"` so the deskewer uses its stock VELODYNE branch and the code base stays as close to upstream DLIO as possible.

## Recovery and cross-check

The localizer relies on a single GT-class odometry topic for both the **RTK-driven IMU calibration** and the **GT-snap recovery** path. On the Hitch dome the Atlas Duo's `/odom` publishes continuously regardless of RTK status, so a thin gating layer is needed to translate "/gps/fix shows STATUS_GBAS_FIX" into "this `/odom` message is trustworthy."

The recommended setup is a small republisher node (e.g., `nav_sat_gated_odom`) that:

1. Subscribes to `/odom` (Atlas Duo INS) and `/gps/fix` (NavSatFix).
2. Keeps the most recent NavSatFix and inspects `status.status` and `position_covariance` on every `/odom` arrival.
3. Forwards the Odometry message to `/odom_rtk_only` only when `status.status >= STATUS_GBAS_FIX` and the position covariance σ is below a threshold (matches GLIM_plusplus's factor-bridge defaults).
4. Silently drops messages during RTK-float / tunnel periods.

The localizer treats any message arriving on `gt_odom` as RTK-fixed, so all quality filtering must happen upstream of this topic. With the gate in place, **RTK loss is automatic** — `/odom_rtk_only` simply goes quiet during tunnels, the localizer keeps running on IMU dead-reckoning + scan matching, and recovery snaps wait for the next RTK-fixed sample.

If you are running offline against a recorded bag that contains pre-gated odometry, point `gt_odom_topic` directly at that topic and skip the republisher.

## Topics

### Subscribed

| Topic (remap) | Type | Purpose |
|---|---|---|
| `pointcloud` | `sensor_msgs/PointCloud2` | Primary Robin W scans. |
| `imu` | `sensor_msgs/Imu` | Atlas Duo IMU. Required for the motion prior and observer. |
| `<aux_topics>` | `sensor_msgs/PointCloud2` | Rear Robin W scans (when `lidar_concat/enabled=true`). |
| `gt_odom` | `nav_msgs/Odometry` | RTK-gated INS odometry for cross-check / recovery / bootstrap. |
| `odom` | `nav_msgs/Odometry` | External odom for init (when `use_odom_init=true` and GT not used). |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz initial-pose override. |

### Published

| Topic | Type | Purpose |
|---|---|---|
| `localized_pose` → `gicp/localization/pose` | `geometry_msgs/PoseStamped` | Localized pose (scan rate). |
| `localized_odom` → `gicp/localization/odom` | `nav_msgs/Odometry` | Localized odom propagated at IMU rate (~100 Hz). |
| `localized_path` → `gicp/localization/path` | `nav_msgs/Path` | Trajectory history. |
| `gicp/localization/pose_utm` / `odom_utm` / `path_utm` | same types | UTM-frame mirrors when `utm_transform_path` is set. |
| `aligned_cloud` → `gicp/localization/aligned_cloud` | `sensor_msgs/PointCloud2` | Aligned scan in map frame. |
| `map` → `gicp/localization/map` | `sensor_msgs/PointCloud2` | Downsampled visualization map. |
| TF: `map → base_frame` | — | Published when `publish_tf=true`. |

### Debug topics

Per-scan scalar metrics on `gicp/localization/debug/*` (require `localization/debug/enable_pub: true`):
`fitness`, `gicp_elapsed_ms`, `final_error`, `corr_norm`, `scan_dt`, `imu_age`, `imu_buffer_span_s`, `scan_to_latest_imu_lag_s`, `num_correspondences`, `correspondence_ratio`, `guess_to_solution_m`, `guess_to_solution_deg`, `jump_trans`, `jump_rot_deg`, `hessian_condition_proxy`, `gt_pos_err_m`, `gt_rot_err_deg`, `converged`.

## Algorithm

```
IMU ─────→ buffer ───────────────────────────────────┐
                                                     ↓
LiDAR ──→ lidar_concat ──→ preprocess ──→ T_prior = integrate(IMU, last lidarPose)
                                                     ↓
                                            GICP align (initial guess = T_prior)
                                                     ↓
                                            gate: fitness / hessian-combined / jump
                                  ┌───── accepted ──┴── rejected ──┐
                                  ↓                                ↓
                       updateState (geo observer)       dead-reckon: lidarPose ← T_prior
                                  ↓                                ↓
                       state ← merge(GICP, IMU)         consecutive_failures++
                                                                   ↓
                                                       ≥ N consecutive AND gt_recovery on?
                                                                   ↓
                                                       maybeSnapPoseToGT(reason)

propagateState (every IMU sample) → publish odom / TF at ~100 Hz
```

## Map preparation

GLIM_plusplus dumps submaps as PLY-format files with a `.pcd` extension. Convert them to a single merged PCD with:

```bash
python3 scripts/convert_ply_to_pcd.py /path/to/glim_map.pcd /path/to/output_map.pcd
```

For UTM-frame output, point `localization/utm_transform_path` at the `T_world_utm.txt` from the same GLIM dump.

## Troubleshooting

**"IMU never received" / pose stuck in dead-reckoning.** Symptoms: `imu_buffer_span=-1.000s` in SCAN DEBUG, `guess_from_last=0`, no "First IMU message received" log. Almost always a topic-mismatch problem. The launch arg is `imu_topic` (underscore); passing `imu-topic:=/X` silently does nothing and the launch falls back to the default `/imu/data`. Verify with:

```bash
ros2 topic list | grep -i imu
ros2 topic hz /imu/data
```

**GT recovery enabled but no snap fires.** Look in the log for one of:

- `GT recovery: deferring snap — no GT odom received yet` → the gating republisher isn't publishing. Check `nav_sat_gated_odom` (or whichever upstream node produces `/odom_rtk_only`), and confirm `/gps/fix` is currently RTK-fixed.
- `GT recovery: deferring snap — base→<frame> TF not cached yet` → the URDF doesn't include the GT publisher's `child_frame_id`. Either add the link to `sensor_dome.urdf` or repoint the publisher to a frame already in the URDF.
- `GT recovery: deferring snap — no GT sample within max_dt=...` → the gated INS topic is publishing slower than `gt_odom/max_dt`. Raise the parameter.

**Scan dropouts during sharp turns.** If you see SCAN DEBUG gaps > 200 ms during turns, `lidar_concat/time_threshold` is dropping aux scans that fell out of sync. Try raising it from `0.05` to `0.1–0.15`. The merged cloud will have slightly worse intra-frame alignment, but that's almost always cheaper than a 600 ms scan-stream gap during cornering.

**GICP slides at corners.** Watch for `GICP REJECTED (...— degenerate slide)` warns. Tunable knobs (in order of impact):

1. Lower `gicp/hessianTransWarnM` / `hessianRotWarnDeg` to catch slides earlier (defaults 1.0 m / 1.5°).
2. Lower `gicp/hessianCondMax` to be stricter about what counts as "degenerate" (default 5e9).
3. If the rejection cascades, enable `gt_recovery` to recover at corners.

**Frame check.** `ros2 run tf2_tools view_frames` should show `map → imu_link` (`base_frame`) and the URDF chain `base_link → lidar_front_link`, `base_link → imu_link`, etc. The localization node tracks `base_frame`; everything else is just the URDF.

## Debug scripts

`scripts/debug_pose_inspector.py` — prints / CSVs / plots per-scan initial-guess vs final-pose deltas. Requires `localization/debug/enable_pub: true`.

```bash
python3 scripts/debug_pose_inspector.py --csv-path /tmp/gicp_pose_debug.csv
python3 scripts/debug_pose_inspector.py --csv-path /tmp/x.csv --no-plot
python3 scripts/debug_pose_inspector.py --csv-path /tmp/x.csv --max-samples 300
```

`scripts/visualize_lidar_topic.py` — checks that the localization node is consuming the LiDAR topic you expect, by reconstructing the same cloud (TF + flip_y) and comparing against `gicp/localization/debug/input_cloud_base`. Defaults to `/robin_w_front/points` and `lidar_front_link`.

`scripts/profile_localization_resources.py` and `scripts/plot_localization_profile.py` — capture and chart per-scan resource usage and the full debug-topic time series. Useful for tuning real-time performance.

`scripts/plot_source_switches.py` — plots when the node switches between GICP, dead-reckoning, and GT snap; useful when investigating snap behavior.
