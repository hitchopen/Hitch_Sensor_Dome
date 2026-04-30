# GLIM++ — A heavily modified fork of GLIM for Hitch Sensor Dome

> **This is not stock GLIM.** The folder name `GLIM_plusplus/` is intentional: this version diverges from upstream [`koide3/glim`](https://github.com/koide3/glim) in ways that change algorithmic behavior — not just configuration. If you came here looking for the original GLIM, that lives at <https://github.com/koide3/glim> and we strongly recommend starting there if you do not have the [Hitch Sensor Dome](../README.md) hardware. All algorithmic and implementation credit for GLIM belongs to **Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno (AIST)**. See [Credits](#credits), [License](#license), and [Citation](#citation).

This document is a **complete change log** between GLIM++ and the upstream `koide3/glim`. It is organized so an adopter can decide, mechanism by mechanism, which changes apply to their use case and which would need to be reverted.

## Index of changes

1. [Sensor adaptation for Hitch Sensor Dome](#1-sensor-adaptation-for-hitch-sensor-dome)
2. [Vehicle-agnostic body frame (`base_frame_id = imu_link`)](#2-vehicle-agnostic-body-frame)
3. [Outdoor / vehicle-scale tuning (24 parameter changes)](#3-outdoor--vehicle-scale-tuning)
4. [Multi-lap loop closure fix](#4-multi-lap-loop-closure-fix)
5. [Initialization rewrite — INS-driven, gravity-from-accelerometer removed](#5-initialization-rewrite)
6. [RTK-fixed gating for the initial pose](#6-rtk-fixed-gating-for-the-initial-pose)
7. [RTK-gated GNSS factor bridge (post-init)](#7-rtk-gated-gnss-factor-bridge)
8. [Project integration — URDF generator, launch helper, diagnostics](#8-project-integration)
9. [Documentation added](#9-documentation-added)
10. [What was NOT changed](#10-what-was-not-changed)
11. [File-by-file diff summary](#11-file-by-file-diff-summary)

## 1. Sensor adaptation for Hitch Sensor Dome

Topic, frame, and field names throughout the configs are switched from the previous AV-24 deployment (Luminar Iris + custom IMU) to the Hitch Sensor Dome reference setup (3× Seyond Robin W + Point One Atlas Duo + 4× e-con RouteCAM).

| Surface | Was | Now |
|---------|-----|-----|
| IMU topic | `/gps_bot/imu` | `/imu/data` |
| Primary lidar | `/luminar_front/points` | `/robin_w_front/points` |
| Aux lidars | `/luminar_left/points`, `/luminar_right/points` | `/robin_w_rear_left/points`, `/robin_w_rear_right/points` |
| GNSS | `/gps_nav/odom` | `/gps/fix` (NavSatFix gate signal) |
| Camera | `/cam_front_left/image_raw` | (same — naming preserved) |
| `intensity_field` | `reflectance` (Luminar) | `intensity` (Robin W default) |
| `ring_field` | `line_index` (Luminar) | `ring` (Robin W default) |
| `flip_points_y` | `true` (Luminar SAE Y-right) | `false` (Robin W in `coordinate_mode:=3` already emits REP-103 axes) |
| LiDAR–IMU extrinsic source | hand-edited 7-element TUM array | URDF generated from [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) — see §8 |

Files touched: `glim/config/config_sensors.json`, `glim/config/config_ros.json`.

## 2. Vehicle-agnostic body frame

In [`glim/config/config_ros.json`](glim/config/config_ros.json):

```json
"imu_frame_id":  "imu_link",
"lidar_frame_id":"lidar_front_link",
"base_frame_id": "imu_link"      // was: "" (auto-detect)
```

Pinning `base_frame_id` to `imu_link` makes GLIM build the global map relative to the Atlas Duo Center of Navigation rather than to a vehicle-specific `base_link`. Each downstream vehicle integrator publishes its own static `imu_link → base_link` transform — the recorded map is then portable across platforms without re-running SLAM.

This is a deliberate departure from upstream GLIM's behaviour, which left `base_frame_id` blank and inherited the IMU frame's name (which on the AV-24 was `gps_bot`, vehicle-specific).

## 3. Outdoor / vehicle-scale tuning

Twenty-four parameter changes across the seven JSON configs, each annotated inline. The high-level grouping:

| Area | Files | Direction |
|------|-------|-----------|
| **IMU noise / bias** | `config_sensors.json` | Loosened: `acc 0.05 → 0.2`, `gyro 0.02 → 0.05`, `bias 1e-5 → 1e-4`. Less IMU trust, more LiDAR trust. |
| **Accelerometer scale** | `config_ros.json` | `acc_scale: 0.0 → 1.0`. Atlas Duo emits m/s² natively, no auto-detect needed. |
| **Initialization** | `config_odometry_gpu.json` | Window `1.0 → 3.0` s for cleaner gravity (later replaced entirely — see §5). |
| **GNSS prior** | `config_gnss_global.json` | `prior_inf_scale: [0,0,0] → [1e4, 1e4, 5e4]` (was silently disabled upstream). `min_baseline: 1.0 → 0.5`. |
| **Deskewing** | `config_sensors.json` | `global_shutter_lidar: true → false`. Re-enables motion deskewing (the upstream multi-LiDAR timestamp rebasing bug it was working around has since been fixed). |
| **Per-point time** | `config_sensors.json` | `autoconf_perpoint_times: false → true`, `perpoint_relative_time: false → true`. Robin W driver emits relative-to-frame timestamps. |
| **Downsampling** | `config_preprocess.json` | `random_downsample_target: 10000 → 20000`, `k_correspondences: 10 → 20`. Higher density required for the 360° stitched cloud from 3 sensors. |
| **VGICP voxels** | `config_odometry_gpu.json`, `config_sub_mapping_gpu.json` | Base resolutions raised for outdoor scale: `voxel_resolution 0.25 → 0.5`, `voxel_resolution_max 0.5 → 1.0`. Submap-internal `keyframe_voxel_resolution 0.25 → 0.15` for tighter local alignment. |
| **Smoother window** | `config_odometry_gpu.json` | `full_connection_window_size: 2 → 4`. Aggressive vehicle motion. |
| **Sub-mapping** | `config_sub_mapping_gpu.json` | `max_num_keyframes: 15 → 20`. Better submap coverage. |
| **Extension modules** | `config_ros.json` | `libimu_validator.so` enabled for bring-up diagnostics. |

Files touched: every `glim/config/*.json` and `glim_ext/config/*.json` listed above.

## 4. Multi-lap loop closure fix

A targeted three-layer fix in `glim/config/config_global_mapping_gpu.json` and `glim_ext/config/config_gnss_global.json` for the canonical "second-lap-tilts-to-the-sky" failure mode of upstream GLIM:

| Knob | Was | Now |
|------|-----|-----|
| `submap_voxel_resolution_max` | `1.0` m | `2.0` m (wider VGICP convergence basin) |
| `max_implicit_loop_distance` | `100` m | `200` m (covers a typical race-track lap) |
| `min_implicit_loop_overlap` | `0.2` | `0.1` (partial overlap still creates factors) |
| GNSS `prior_inf_scale[2]` (z) | `1e4` | `5e4` (z 5× stronger than horizontal) |
| GNSS `min_baseline` | `1.0` m | `0.5` m (twice the GNSS factor density) |

The chicken-and-egg between odometry initialization and VGICP convergence basin is described in [`docs/multi_lap_loop_closure.md`](docs/multi_lap_loop_closure.md). The first three knobs widen the convergence basin so closure factors fire even with residual drift; the GNSS knobs prevent the drift from accumulating in the first place. Both are required.

## 5. Initialization rewrite

This is the most invasive C++ change in the fork. **Upstream GLIM derives the world-frame orientation from the accelerometer mean over the first `initialization_window_size` seconds**, which assumes the IMU is stationary during that window. That assumption breaks whenever a recording starts with the vehicle in motion — common on race tracks, mid-session restarts, and bag replays trimmed to a moving segment. The integrated linear acceleration leaks into the gravity estimate, producing a tilted world frame that biases everything downstream.

GLIM++ removes the gravity-from-accelerometer pathway entirely and requires an external INS pose before the optimizer starts.

### 5.1 SLAM core changes

| File | Change |
|------|--------|
| [`glim/src/glim/odometry/initial_state_estimation.cpp`](glim/src/glim/odometry/initial_state_estimation.cpp) | `NaiveInitialStateEstimation::initial_pose()` returns `nullptr` until `force_init==true`. The `acc_dir` → `T_world_imu` derivation is gone. `insert_imu()` no longer accumulates `sum_acc`. |
| [`glim/src/glim/odometry/odometry_estimation_imu.cpp`](glim/src/glim/odometry/odometry_estimation_imu.cpp) | Constructor always instantiates `NaiveInitialStateEstimation`. The `LooseInitialStateEstimation` branch and the `estimate_init_state` branching are gone. New public method `OdometryEstimationIMU::set_init_state(T, v)` forwards to the Naive instance via `dynamic_cast`. |
| [`glim/include/glim/odometry/odometry_estimation_base.hpp`](glim/include/glim/odometry/odometry_estimation_base.hpp) | New virtual `set_init_state(T, v)` (default no-op) so the .so boundary is type-safe. |
| [`glim/include/glim/odometry/async_odometry_estimation.hpp`](glim/include/glim/odometry/async_odometry_estimation.hpp) and [its `.cpp`](glim/src/glim/odometry/async_odometry_estimation.cpp) | Public `set_init_state(T, v)` queues the values onto a mutex-protected slot; the worker thread drains it at the top of `run()` so the estimator's `set_init_state` is invoked from a single thread without racing `insert_imu` / `insert_frame`. Idempotent. |

### 5.2 ROS wrapper changes

| File | Change |
|------|--------|
| [`glim_ros2/include/glim_ros/glim_ros.hpp`](glim_ros2/include/glim_ros/glim_ros.hpp) and [`.cpp`](glim_ros2/src/glim_ros/glim_ros.cpp) | New subscriptions to `ins_pose_topic` (default `/pose`, `geometry_msgs/PoseStamped`) and `ins_odom_topic` (default empty, `nav_msgs/Odometry`). On the first valid message that passes the gate (§6), the orientation is forwarded to `odometry_estimation->set_init_state(T, v)` and the optimizer's gravity reference is fixed for the session. |

The pathway is documented in detail in [`docs/moving_start_initialization.md`](docs/moving_start_initialization.md).

**This change requires an INS to start GLIM.** Pure-LiDAR setups would need to revert §5 — the previous LOOSE / NAIVE pathways are gone. For Hitch Sensor Dome the Atlas Duo is always present, so the trade is unconditional: a working INS in exchange for a moving-start failure mode that no longer exists.

## 6. RTK-fixed gating for the initial pose

Naive "accept the first pose" would happily latch onto an INS that's still cold-starting, dead-reckoning on IMU only, or in RTK-float mode. Since the entire SLAM map is anchored to this single pose, accepting a bad one means re-recording the session.

The wrapper enforces a **three-stage gate** before calling `set_init_state`:

| Stage | Check | Default threshold |
|-------|-------|-------------------|
| 1. Fix status | `NavSatFix.status.status ≥ STATUS_GBAS_FIX` (RTK-class) | `ins_require_rtk_fixed = true` |
| 2. Covariance | `position_covariance` diagonal max σ ≤ threshold | `0.10 m` |
| 3. Stability | Last N consecutive INS poses are mutually consistent (translation jitter, `\|q1·q2\|`) | `N=10`, `0.05 m`, `0.999` |

While any stage is failing, a 2-second wall timer ticks `ins_init_timeout_tick()` and prints a **bold-RED multi-line warning every 10 s** naming the most recent rejection reason and listing remediation steps. After `ins_init_timeout_s = 60 s` the warning escalates to "TIMEOUT" — but **GLIM never auto-aborts**. The operator decides.

Eight new ROS parameters (declared in [`launch/hitch_sensor_dome.launch.py`](launch/hitch_sensor_dome.launch.py) and documented in [`glim/config/config_ros.json`](glim/config/config_ros.json)):

```
ins_pose_topic                    default /pose
ins_odom_topic                    default ""
ins_fix_topic                     default /gps/fix
ins_require_rtk_fixed             default true
ins_max_position_stddev           default 0.10
ins_min_pose_window_samples       default 10
ins_max_pose_jitter_trans         default 0.05
ins_min_quat_dot                  default 0.999
ins_init_timeout_s                default 60.0
```

## 7. RTK-gated GNSS factor bridge

Upstream's `libgnss_global.so` adds soft GNSS prior factors to the global graph, but its dispatcher only handles `nav_msgs/msg/Odometry` and `geometry_msgs/msg/PoseWithCovarianceStamped`. Pointing it directly at `/gps/fix` (NavSatFix) silently drops every message — earlier revisions of this fork had this exact misconfiguration, so the multi-lap z-drift tuning in §4 was a no-op until the bridge was added.

The upstream module also (per its own header comment) "ignores GNSS observation covariance" — RTK-fixed and RTK-float would have been weighted equally if both were passed through.

The fork adds an in-process bridge:

```
fusion_engine_driver
   ├── /pose          (PoseStamped)
   └── /gps/fix       (NavSatFix, RTK gate signal)
                  │
                  ▼
       GlimROS::try_publish_gnss_factor
           - require RTK-fixed status (configurable)
           - require pos σ ≤ threshold
                  │
                  ▼
       /gnss/pose_rtk_only (PoseWithCovarianceStamped)
                  │
                  ▼
       libgnss_global.so subscribes here
                  │
                  ▼
       Soft factors added to global graph
       — exclusively from RTK-fixed periods.
```

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `gnss_factor_topic` | `/gnss/pose_rtk_only` | Where the bridge publishes; `""` disables the bridge |
| `gnss_factor_require_rtk_fixed` | `true` | If true, only republish during RTK-class fixes |
| `gnss_factor_max_position_stddev` | `0.10 m` | Reject poses whose NavSatFix covariance exceeds this |

A periodic 10-second log line reports `N published, M rejected` so the operator can see the bridge actually doing work mid-session.

**Behavior on RTK loss.** When RTK degrades to float / no-fix (tunnel, urban canyon), the bridge silently suspends; `gnss_global` sees a quiet topic and adds no factor for that interval. The optimizer's LiDAR cost carries the trajectory through the gap. When RTK locks again, factors resume on the next `/pose`. The session-long RTK requirement only applies to the initial pose (§6); per-message gating during the session is a soft suspend, not a hard block.

Files touched: [`glim_ros2/src/glim_ros/glim_ros.cpp`](glim_ros2/src/glim_ros/glim_ros.cpp), [`glim_ros2/include/glim_ros/glim_ros.hpp`](glim_ros2/include/glim_ros/glim_ros.hpp), [`glim_ext/config/config_gnss_global.json`](glim_ext/config/config_gnss_global.json), [`launch/hitch_sensor_dome.launch.py`](launch/hitch_sensor_dome.launch.py).

## 8. Project integration

Three new top-level folders inside `GLIM_plusplus/` that hold integration-only code (no upstream GLIM source touched here):

| Folder | Contents |
|--------|----------|
| [`config/`](config/) | `generate_sensor_dome_urdf.py` converts [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) into `sensor_dome.urdf`, which GLIM consumes via the `urdf_path` field in `config_sensors.json` for both `T_lidar_imu` and multi-LiDAR `lidar_concat`. Single source of truth across recording, visualization, and mapping. Re-run when the TF YAML changes. |
| [`launch/`](launch/) | `hitch_sensor_dome.launch.py` — publishes static TFs from `sensor_dome_tf.yaml`, starts `glim_rosnode` against the project's tuned configs, spawns `foxglove_bridge` for visualization, runs the pre-flight stationarity check. |
| [`scripts/`](scripts/) | `check_init_stationarity.py` — pre-flight diagnostic; reads first 3 s of `/imu/data` and prints a bold-RED warning if the bag is non-stationary. Now informational only (the C++ INS-init pathway handles moving starts), but useful for diagnosing slow Atlas Duo lock and for CI gating. |

## 9. Documentation added

Inside [`docs/`](docs/):

- [`moving_start_initialization.md`](docs/moving_start_initialization.md) — full design of §5 + §6 + §7. Covers the data flow, the RTK gate semantics, sample warning output, sample success output, and how to operate without RTK.
- [`multi_lap_loop_closure.md`](docs/multi_lap_loop_closure.md) — root-cause analysis of the chicken-and-egg between odometry initialization and VGICP convergence basin, the three-layer fix, post-run verification checks (GNSS factor count, pose-graph edges, `T_world_utm.txt` stability, trajectory altitude vs GNSS altitude), and a five-step escalation if the default fix isn't sufficient.

## 10. What was NOT changed

Important — so adopters know what stayed identical to upstream and can rely on existing GLIM literature:

- The factor graph optimizer (`gtsam_points::IncrementalFixedLagSmootherExtWithFallback`) — unchanged.
- VGICP scan matching itself (the cost function, gradient, voxel structure) — unchanged.
- Sub-mapping and global mapping module structure — unchanged.
- `gtsam_points` / GTSAM dependencies — unchanged.
- Iridescence native viewer — unchanged.
- IMU integration math (`imu_integration.cpp`) — unchanged.
- Cloud preprocessing (`cloud_preprocessor.cpp`, `cloud_deskewing.cpp`) — unchanged.
- All extension modules other than `gnss_global` — unchanged (gravity_estimator, flat_earther, deskewing, imu_validator, imu_prediction, velocity_suppressor, orb_slam, scan_context_loop_detector, dbow_loop_detector).
- License texts — every upstream LICENSE / NOTICE preserved verbatim.
- Sub-package READMEs (`glim/README.md`, `glim_ext/README.md`, `glim_ros2/README.md`) — unchanged. They document the upstream packages on their own terms and modifying them would muddy the upstream provenance.

## 11. File-by-file diff summary

| File | Status | Change category |
|------|--------|-----------------|
| `glim/include/glim/odometry/odometry_estimation_base.hpp` | modified | §5 (new virtual) |
| `glim/include/glim/odometry/odometry_estimation_imu.hpp` | modified | §5 (override decl) |
| `glim/include/glim/odometry/async_odometry_estimation.hpp` | modified | §5 (forwarder decl + state) |
| `glim/src/glim/odometry/initial_state_estimation.cpp` | modified | §5 (gravity-from-acc removed) |
| `glim/src/glim/odometry/odometry_estimation_imu.cpp` | modified | §5 (force NAIVE, set_init_state impl) |
| `glim/src/glim/odometry/async_odometry_estimation.cpp` | modified | §5 (forwarder impl + queue drain) |
| `glim/config/config_ros.json` | modified | §1 §2 §3 §6 (topics, frames, IMU, RTK gate params) |
| `glim/config/config_sensors.json` | modified | §1 §3 (Robin W adaptation, IMU noise, deskewing) |
| `glim/config/config_preprocess.json` | modified | §3 (downsample target, k_correspondences) |
| `glim/config/config_odometry_gpu.json` | modified | §3 (voxel scale, smoother window) |
| `glim/config/config_sub_mapping_gpu.json` | modified | §3 (sub-map keyframes, voxels) |
| `glim/config/config_global_mapping_gpu.json` | modified | §3 §4 (loop-closure thresholds) |
| `glim_ext/config/config_gnss_global.json` | modified | §4 §7 (prior_inf_scale, bridge topic/type) |
| `glim_ros2/include/glim_ros/glim_ros.hpp` | modified | §5 §6 §7 (subs, params, factor bridge state) |
| `glim_ros2/src/glim_ros/glim_ros.cpp` | modified | §5 §6 §7 (full ROS-side implementation) |
| `config/generate_sensor_dome_urdf.py` | new | §8 |
| `config/sensor_dome.urdf` | new (generated) | §8 |
| `launch/hitch_sensor_dome.launch.py` | new | §8 |
| `scripts/check_init_stationarity.py` | new | §8 |
| `docs/moving_start_initialization.md` | new | §9 |
| `docs/multi_lap_loop_closure.md` | new | §9 |
| All other upstream files | unchanged | §10 |

## Quick start

Edit [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) once for your installation, then:

```bash
cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py
cd ../..

# Build (CMake / colcon — gtsam, gtsam_points, Iridescence as upstream)
colcon build --packages-select glim glim_ext glim_ros \
             --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Live mapping (subscribes directly to the recording stack):
ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py

# Offline replay against an MCAP bag:
ros2 run glim_ros glim_rosbag recording/data/session_<ts>/rosbag2 \
    --ros-args -p config_path:=GLIM_plusplus/glim/config \
                -p dump_path:=glim_maps/session_<ts>
```

The ROS 2 package names (`glim`, `glim_ext`, `glim_ros`) are unchanged — only the workspace folder differs from upstream. So `colcon build --packages-select glim …` works identically.

## Build dependencies

Identical to upstream GLIM. See the upstream documentation for the canonical list.

```bash
sudo apt install -y libeigen3-dev libboost-all-dev libfmt-dev libomp-dev \
                    libmetis-dev ros-${ROS_DISTRO}-tf2-eigen \
                    ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-foxglove-bridge
# Then GTSAM, gtsam_points, Iridescence — see upstream README.
```

## Credits

GLIM is the work of:

- **GLIM** — Kenji Koide, Masashi Yokozuka, Shuji Oishi, Atsuhiko Banno (AIST). <https://github.com/koide3/glim>
- **gtsam_points** — Kenji Koide. <https://github.com/koide3/gtsam_points>
- **Iridescence** — Kenji Koide. <https://github.com/koide3/iridescence>
- **GTSAM** — Frank Dellaert and the Georgia Tech Borg Lab. <https://github.com/borglab/gtsam>

The integration work in `GLIM_plusplus/{config, launch, scripts, docs}/` and the modifications detailed in §1 – §7 are part of the **Hitch Sensor Dome** project, designed and maintained by Dr. Allen Y. Yang (Hitch Interactive · University of California, Berkeley). Implementation testing by the **Berkeley AI Racing Tech** team (see [`../README.md`](../README.md) Credits).

## License

This fork inherits the license of every constituent package. **No upstream license text has been altered.**

- **GLIM** — MIT License (Kenji Koide / AIST)
- **gtsam_points** — MIT License
- **GTSAM** — BSD License
- **Iridescence** — MIT License
- **Hitch Sensor Dome integration code** — see [`../LICENSE`](../LICENSE).

The full upstream license texts are preserved inside each package directory (`glim/`, `glim_ext/`, `glim_ros2/`).

## Citation

If you use GLIM++ in academic work, please cite the upstream GLIM paper:

```bibtex
@article{koide2024glim,
  title   = {GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors},
  author  = {Koide, Kenji and Yokozuka, Masashi and Oishi, Shuji and Banno, Atsuhiko},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2024}
}
```

If the modifications documented in this README were specifically useful, also cite:

```bibtex
@misc{yang2026hitchsensordome,
  title  = {Hitch Sensor Dome: a 3D-printable modular multi-sensor mount for vehicle-roof mapping},
  author = {Yang, Allen Y.},
  year   = {2026},
  note   = {GitHub repository — includes the GLIM++ fork at GLIM_plusplus/}
}
```
