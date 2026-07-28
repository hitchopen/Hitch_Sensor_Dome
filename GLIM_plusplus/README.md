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
8. [Optional GNSS yaw prior — dual-antenna only](#8-optional-gnss-yaw-prior)
9. [Project integration — URDF generator, launch helper, diagnostics](#9-project-integration)
10. [Design notes — loop closure, moving start, TF verification, merge record](#10-design-notes)
11. [What was NOT changed](#11-what-was-not-changed)
12. [File-by-file diff summary](#12-file-by-file-diff-summary)
13. [2026-07 P1–P5 improvements — merge evidence, yaw-quality gate, GNSS backfill](#13-2026-07-p1p5-improvements)
14. [2026-07-27 upstream re-merge — point-time sweep matching, GNSS anchor health, PR #15](#14-2026-07-27-upstream-re-merge)

## 1. Sensor adaptation for Hitch Sensor Dome

Topic, frame, and field names throughout the configs target the Hitch Sensor Dome reference setup (3× Seyond Robin W + Point One Atlas Duo + 4× e-con RouteCAM).

| Surface | Hitch Sensor Dome setting |
|---------|---------------------------|
| IMU topic | `/imu/data` (fusion_engine_driver, Atlas Duo) |
| Primary lidar | `/robin_w_front/points` |
| Aux lidars | `/robin_w_rear_left/points`, `/robin_w_rear_right/points` |
| GNSS | `/gps_p1/fix` (synchronized adapter NavSatFix gate) + `/gps_p1/filtered_odom_rtk_fixed` (adapter Fixed-only odometry) |
| Camera | `/cam_front_left/image_raw` |
| `intensity_field` | `intensity` (Robin W default) |
| `ring_field` | `ring` (Robin W default) |
| `flip_points_y` | `false` (Robin W in `coordinate_mode:=3` already emits REP-103 axes) |
| LiDAR–IMU extrinsic source | URDF generated from [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) — see §8 |

Files touched: `glim/config/config_sensors.json`, `glim/config/config_ros.json`.

### 1.1 Single-antenna vs dual-antenna mode

The Hitch Sensor Dome supports either one or two GNSS antennas (a second antenna at a known offset enables drift-free RTK heading). The mode is auto-detected from [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml): the secondary antenna's translation defaults to the sentinel `(0, 0, 0)` (single-antenna), and any non-zero translation (norm ≥ 0.05 m) flips GLIM++ into dual-antenna mode at launch time. The full root-README walkthrough for enabling dual-antenna is in the project's [main README](../README.md#-dual-gnss-antenna--strongly-recommended); the algorithmic differences GLIM++ makes between the two modes are summarized below.

| Aspect | Single-antenna | Dual-antenna |
|--------|----------------|--------------|
| **Heading source** | IMU gyroscope integration (drifts with bias over time) | RTK-fixed dual-antenna baseline (drift-free, ≈ 0.1°–1° depending on baseline) |
| **Init gate `ins_max_attitude_residual_deg`** | `2.5°` attitude residual | auto-tightened to `0.8°` |
| **Init gate `ins_min_pose_window_samples`** | `10` consecutive consistent samples | auto-shortened to `5` (orientation locks faster) |
| **Init gate `ins_init_timeout_s`** | `60 s` | auto-shortened to `30 s` |
| **Factor-bridge orientation covariance** | not populated (no usable orientation info from single antenna's PoseStamped) | tight yaw σ derived from baseline length, loose pitch/roll — see §7 |
| **Session-long heading drift** | accumulates with IMU bias; mitigated only by LiDAR scan matching | bounded by RTK heading available throughout the session (data path in place, factor module deferred — see §10 "Session-long heading correction") |
| **Map quality on long / multi-lap trajectories** | depends on LiDAR feature richness for yaw stability; ground-truth z-anchor still good if RTK position is fixed | better orientation accuracy at init, with a clean data path for further session-long heading correction |
| **Hardware needed** | one SP1 (or compatible) | two antennas at fixed offset (1.0–1.5 m baseline recommended) |
| **Operator UX at launch** | `Hitch fork: SINGLE-antenna mode — heading derived from IMU (drift-prone).` | `Hitch fork: DUAL-antenna mode — baseline=1.000 m, expected heading σ=0.010 rad (0.57°). Init gates auto-tightened.` |

The mode switch is **fully automatic** — there is no separate "dual-antenna" launch arg. GLIM++ reads the TF YAML at startup, computes the secondary translation norm, and chooses. To revert from dual to single (e.g., the second antenna failed in the field), edit `sensor_dome_tf.yaml` and zero out the secondary translation; everything else continues to work because the primary antenna alone provides RTK position, which is what GLIM++'s init gate (§6) and factor bridge (§7) actually require.

## 2. Vehicle-agnostic body frame

In [`glim/config/config_ros.json`](glim/config/config_ros.json):

```json
"imu_frame_id":  "imu_link",
"lidar_frame_id":"lidar_front_link",
"base_frame_id": "imu_link"      // was: "" (auto-detect)
```

Pinning `base_frame_id` to `imu_link` makes GLIM build the global map relative to the Atlas Duo Center of Navigation rather than to a vehicle-specific `base_link`. Each downstream vehicle integrator publishes its own static `imu_link → base_link` transform — the recorded map is then portable across platforms without re-running SLAM.

This is a deliberate departure from upstream GLIM's behaviour, which left `base_frame_id` blank and inherited the IMU frame's name (a vehicle-specific, driver-chosen string).

## 3. Outdoor / vehicle-scale tuning

Twenty-four parameter changes across the seven JSON configs, each annotated inline. The high-level grouping:

| Area | Files | Direction |
|------|-------|-----------|
| **IMU noise / bias** | `config_sensors.json` | Loosened: `acc 0.05 → 0.2`, `gyro 0.02 → 0.05`, `bias 1e-5 → 1e-4`. Less IMU trust, more LiDAR trust. |
| **Accelerometer scale** | `config_ros.json` | `acc_scale: 0.0 → 1.0`. Atlas Duo emits m/s² natively, no auto-detect needed. |
| **Initialization** | `config_odometry_gpu.json` | Window `1.0 → 3.0` s for cleaner gravity (later replaced entirely — see §5). |
| **GNSS prior** | `config_gnss_global.json` | `prior_inf_scale: [0,0,0] → [100, 100, 25]` with covariance-aware `prior_inf_floor`/`prior_inf_cap` and a Huber width (was silently disabled upstream). `min_baseline: 1.0 → 10.0` (world↔UTM fit gate; the earlier `0.5` was set on a false factor-density premise). |
| **Deskewing** | `config_sensors.json` | `global_shutter_lidar: true → false`. Re-enables motion deskewing (the upstream multi-LiDAR timestamp rebasing bug it was working around has since been fixed). |
| **Per-point time** | `config_sensors.json` | `timestamp/FLOAT64`, numeric Unix seconds, with `autoconf_perpoint_times: false`, `perpoint_relative_time: false`, and scale `1.0`. The schema and absolute axis are validated before deskew. |
| **Downsampling** | `config_preprocess.json` | `random_downsample_target: 10000 → 30000`, `k_correspondences: 10 → 20`. Higher density required for the 360° stitched cloud from 3 sensors; the 3× upstream factor is safe because the dome pipeline runs GLIM **offline** against recorded MCAP bags, so there is no real-time per-scan budget. Lower to 15-20K if switching to live mapping. |
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
| GNSS `prior_inf_scale[2]` (z) | `1e4` | `25` — with `prior_inf_scale = [100, 100, 25]`, `prior_inf_floor = [100, 100, 25]`, `prior_inf_cap = [2500, 2500, 1000]`. **Vertical is deliberately *weaker* than horizontal, not stronger.** |
| GNSS `min_baseline` | `1.0` m | `10.0` m (world↔UTM fit gate — **not** a factor-density knob; see below) |

> **Correction (2026-07-27).** `min_baseline` was previously listed here as `0.5` m "twice the GNSS factor density". That premise was false: `min_baseline` gates only the one-shot `T_world_utm` alignment fit and never the factor-emission loop, so lowering it adds no factors — it merely lets the world↔UTM alignment be fitted from a shorter, noisier baseline, which latched several degrees of yaw error for entire runs. Restored to `10.0` m alongside `fit_min_samples` and out-of-sample fit validation. Weight GNSS against LiDAR with `prior_inf_scale` / `prior_inf_floor` / `prior_inf_cap` instead.
>
> The `prior_inf_scale[2]` row above was wrong in the same revision, and in the opposite direction: it claimed z was made *5× stronger* than horizontal. The shipped config does the reverse — precision `25` vertical against `100` horizontal (σ ≈ 0.2 m vs 0.1 m), and cap `1000` against `2500` (σ ≈ 3.2 cm vs 2.0 cm). That matches how RTK actually behaves: vertical accuracy is roughly half of horizontal, so over-trusting z pulls the map's height against the LiDAR solution. The absolute magnitudes also dropped from `1e4` to `100` because the covariance-aware floor/cap path is now the active weighting mechanism; `prior_inf_scale` is only the fallback for samples carrying no usable covariance, and at `1e4` (1 cm) such a sample would have been *stiffer* than the capped adaptive maximum.

The first three knobs widen the convergence basin so closure factors fire even with residual drift; the GNSS knobs prevent the drift from accumulating in the first place. Both are required, because of a chicken-and-egg described in full in §10 "Multi-lap z-drift": a loop-closure factor can only be created if VGICP converges between the two laps, but VGICP only converges if the accumulated drift is already smaller than its basin — so past a threshold the mechanism that would correct the drift is exactly the one the drift has disabled.

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
| [`glim_ros2/include/glim_ros/glim_ros.hpp`](glim_ros2/include/glim_ros/glim_ros.hpp) and [`.cpp`](glim_ros2/src/glim_ros/glim_ros.cpp) | New subscriptions to `ins_pose_topic` and `ins_odom_topic` (production default `/gps_p1/filtered_odom_rtk_fixed`, `nav_msgs/Odometry`). On the first valid message that passes the gate (§6), the orientation is forwarded to `odometry_estimation->set_init_state(T, v)` and the optimizer's gravity reference is fixed for the session. |

§10 "Moving-start initialization" carries the full pathway: gate semantics, what the operator sees on success and on refusal, initial-velocity seeding, and how the run behaves through an RTK dropout.

**This change requires an INS to start GLIM.** Pure-LiDAR setups would need to revert §5 — the previous LOOSE / NAIVE pathways are gone. For Hitch Sensor Dome the Atlas Duo is always present, so the trade is unconditional: a working INS in exchange for a moving-start failure mode that no longer exists.

## 6. RTK-fixed gating for the initial pose

Naive "accept the first pose" would happily latch onto an INS that's still cold-starting, dead-reckoning on IMU only, or in RTK-float mode. Since the entire SLAM map is anchored to this single pose, accepting a bad one means re-recording the session.

The wrapper enforces an authoritative source check plus three independent
guards before calling `set_init_state`:

| Stage | Check | Default threshold |
|-------|-------|-------------------|
| 0. Solution class | Input arrives on adapter's Fixed-only odometry topic | FusionEngine `kRtkFixed` |
| 1. Fix status | `NavSatFix.status.status ≥ STATUS_GBAS_FIX` (RTK-class cross-check) | `ins_require_rtk_fixed = true` |
| 2. Covariance | Known type; finite, non-negative diagonal; max σ ≤ threshold | `0.10 m` |
| 3. Smoothness | Last N consecutive INS poses are *contiguous* and each fits a constant-velocity / constant-angular-rate extrapolation of its predecessors. The thresholds bound the **residual**, not the raw inter-sample delta, so a steady drive or a steady turn passes at any speed — see §10 | `N=10`, residual `0.05 m` / `2.5°`, max gap `0.5 s` |

While any stage is failing, a 2-second wall timer ticks `ins_init_timeout_tick()` and prints a **bold-RED multi-line warning every 10 s** naming the most recent rejection reason and listing remediation steps. After `ins_init_timeout_s = 60 s` the warning escalates to "TIMEOUT" — but **GLIM never auto-aborts**. The operator decides.

The production values are read from [`glim/config/config_ros.json`](glim/config/config_ros.json). The similarly named launch arguments are documentation-only because this platform maps offline:

```
ins_pose_topic                    default ""
ins_odom_topic                    default /gps_p1/filtered_odom_rtk_fixed
ins_fix_topic                     default /gps_p1/fix
ins_require_rtk_fixed             default true
ins_max_position_stddev           default 0.10
ins_min_pose_window_samples       default 10
ins_max_pose_jitter_trans         default 0.05
ins_max_attitude_residual_deg     default 2.5   (was ins_min_quat_dot 0.999)
ins_init_timeout_s                default 60.0
```

## 7. RTK-gated GNSS factor bridge

Upstream's `libgnss_global.so` adds soft GNSS prior factors to the global graph, but its dispatcher only handles `nav_msgs/msg/Odometry` and `geometry_msgs/msg/PoseWithCovarianceStamped`. Pointing it directly at `/gps/fix` (NavSatFix) silently drops every message — earlier revisions of this fork had this exact misconfiguration, so the multi-lap z-drift tuning in §4 was a no-op until the bridge was added.

The upstream module also (per its own header comment) "ignores GNSS observation covariance" — RTK-fixed and RTK-float would have been weighted equally if both were passed through.

The fork adds an in-process bridge:

```
fusion_engine_driver + adapter
   ├── /gps_p1/filtered_odom_rtk_fixed (Odometry, solution_type == kRtkFixed)
   └── /gps_p1/fix                    (NavSatFix, freshness/covariance cross-check)
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
| `ins_odom_topic` | `/gps_p1/filtered_odom_rtk_fixed` | Adapter stream emitted only for FusionEngine `kRtkFixed` |
| `gnss_factor_topic` | `/gnss/pose_rtk_only` | Where the fixed-source bridge publishes; `""` disables the bridge |
| `gnss_factor_require_rtk_fixed` | `true` | If true, only republish during RTK-class fixes |
| `gnss_factor_max_position_stddev` | `0.10 m` | Reject poses whose NavSatFix covariance exceeds this |

A periodic 10-second log line reports `N published, M rejected` so the operator can see the bridge actually doing work mid-session.

**Behavior on RTK loss.** When RTK degrades to float / no-fix (tunnel, urban canyon), the adapter's Fixed-only odometry topic goes silent; `gnss_global` sees no bridge output and adds no factor for that interval. The optimizer's LiDAR cost carries the trajectory through the gap. When RTK locks again, factors resume on the next Fixed odometry sample. The session-long RTK requirement only applies to the initial pose (§6); per-message gating during the session is a soft suspend, not a hard block.

Files touched: [`glim_ros2/src/glim_ros/glim_ros.cpp`](glim_ros2/src/glim_ros/glim_ros.cpp), [`glim_ros2/include/glim_ros/glim_ros.hpp`](glim_ros2/include/glim_ros/glim_ros.hpp), [`glim_ext/config/config_gnss_global.json`](glim_ext/config/config_gnss_global.json), [`launch/hitch_sensor_dome.launch.py`](launch/hitch_sensor_dome.launch.py).

## 8. Optional GNSS yaw prior

Upstream `gnss_global` adds **translation-only** prior factors (`PoseTranslationPrior`) — the GNSS quaternion is ignored even when the source publishes one. This is correct for single-antenna setups (where the quaternion is gyro-integrated and drifts) but throws away real information from dual-antenna RTK heading.

GLIM++ extends [`glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp`](glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp) so the module can also emit a `PoseRotationPrior` factor on each submap, pulling its yaw toward the heading carried in the incoming `PoseWithCovarianceStamped` / `Odometry` quaternion. The change is opt-in via two JSON keys; the checked-in default is **OFF** because `sensor_dome_tf.yaml` ships with the single-antenna sentinel for the secondary antenna.

| Config key (`glim_ext/config/config_gnss_global.json`) | Default | Meaning |
|---|---|---|
| `enable_orientation_prior` | `false` | Emit a `PoseRotationPrior` factor each submap when dual-antenna heading is configured and the interpolated GNSS sample carries a valid quaternion. |
| `orientation_prior_inf_scale` | `[1e-6, 1e-6, 1e2]` | Information matrix diagonal in `(roll, pitch, yaw)`. Only yaw is meaningfully constrained; roll/pitch get a tiny ε to keep the noise model strictly positive-definite. |

**Algorithmic changes inside the module.**

- The internal sample type changes from `Eigen::Vector4d (stamp, x, y, z)` to a `GNSSData` struct adding `orientation` (`Eigen::Quaterniond`) + `has_orientation` flag.
- `gnss_callback` (both PoseWithCovarianceStamped and Odometry overloads) extracts the quaternion in addition to the position.
- `push_gnss_data` validates the quaternion (norm in `[1e-3, 1.5]` → catches zero quaternions, NaNs, severely denormalized data) and normalizes before storing.
- Submap-stamp association uses `Quaterniond::slerp` to interpolate orientation between bracketing samples. The SLERP only runs when **both** bracketing samples are valid; half-valid pairs produce a position-only association for that submap.
- When `enable_orientation_prior && latest.has_orientation`, after the existing translation prior, the module composes `R_world_imu = R_world_utm · R_utm_imu` and inserts `gtsam::PoseRotationPrior<gtsam::Pose3>` with the configured information matrix.
- `T_world_utm` is still initialized from position alone (SVD of the planar centered covariance) — orientation does not help us solve for the unknown UTM-to-world rotation; we use it only after the alignment is locked.

**Dual-antenna gating end-to-end.** The factor is only safe after a secondary antenna is installed and the Atlas is configured for dual-antenna heading. The Hitch Sensor Dome therefore enforces dual-antenna intent at three independent points (see `README.md` "Three-layered defense"): the Atlas firmware (operator setup), the launch-time consistency check (mismatch between `sensor_dome_tf.yaml` and `enable_orientation_prior`), and the runtime yaw-σ sanity check inside `try_publish_gnss_factor`. The third one is the only check that can catch a misconfigured Atlas firmware.

**Why no lever-arm compensation here.** A related branch of work in the broader GLIM ecosystem pairs the orientation prior with antenna-to-IMU lever-arm compensation (`urdf_gnss_frame`-style). GLIM++ intentionally **does not** apply that compensation, because the Atlas Duo is a tightly-coupled GNSS+INS that already resolves antenna observations to the IMU origin in firmware and publishes `/pose` there. Adding a second lever-arm correction would double-compensate. The prerequisite is that the Atlas firmware's `gnss_lever_arm_primary` / `gnss_lever_arm_secondary` are programmed to match the dome's `sensor_dome_tf.yaml`; see the root README's "GLIM++ GNSS antenna lever-arm compensation" callout. If a future deployment swaps the Atlas Duo for a non-tightly-coupled GNSS, lever-arm compensation must be re-introduced inside the `try_publish_gnss_factor` bridge before publishing to `libgnss_global.so`.

**Behavior on RTK loss.** Same as §7: the wrapper bridge drops samples that fail the RTK gate, so the orientation prior never sees them and never fires during outages. There is no fallback to IMU-derived yaw — when RTK heading isn't available, yaw stays under LiDAR scan-matching control, which is correct (substituting drifting INS yaw for missing GNSS yaw would defeat the purpose of the factor).

Files touched: [`glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp`](glim_ext/modules/mapping/gnss_global/include/glim_ext/gnss_global_module.hpp), [`glim_ext/config/config_gnss_global.json`](glim_ext/config/config_gnss_global.json), [`glim_ros2/src/glim_ros/glim_ros.cpp`](glim_ros2/src/glim_ros/glim_ros.cpp) (runtime sanity check), [`glim_ros2/include/glim_ros/glim_ros.hpp`](glim_ros2/include/glim_ros/glim_ros.hpp) (member fields), [`launch/hitch_sensor_dome.launch.py`](launch/hitch_sensor_dome.launch.py) (launch-time consistency check).

## 9. Project integration

Three new top-level folders inside `GLIM_plusplus/` that hold integration-only code (no upstream GLIM source touched here):

| Folder | Contents |
|--------|----------|
| [`config/`](config/) | `generate_sensor_dome_urdf.py` converts [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) into `sensor_dome.urdf`, which GLIM consumes via the `urdf_path` field in `config_sensors.json` for both `T_lidar_imu` and multi-LiDAR `lidar_concat`. Single source of truth across recording, visualization, and mapping. Re-run when the TF YAML changes. |
| [`launch/`](launch/) | `hitch_sensor_dome.launch.py` — publishes static TFs from `sensor_dome_tf.yaml`, starts `glim_rosnode` against the project's tuned configs, spawns `foxglove_bridge` for visualization, runs the pre-flight stationarity check. |
| [`scripts/`](scripts/) | `check_init_stationarity.py` — pre-flight diagnostic; reads first 3 s of `/imu/data` and prints a bold-RED warning if the bag is non-stationary. Now informational only (the C++ INS-init pathway handles moving starts), but useful for diagnosing slow Atlas Duo lock and for CI gating. |

## 10. Design notes

This section is deliberately self-contained. Everything an adopter needs in
order to operate, debug, or re-derive the fork's behavior is in this file —
there is no companion document to chase, and no link here points at anything
outside the repository except published upstream references.

### Multi-lap z-drift — the chicken-and-egg behind §4

The failure mode: lap 1 maps cleanly, and from lap 2 onward the trajectory
tilts upward, so a closed circuit ends tens of centimetres to several metres
above where it started. It is not a scan-matching bug. It is a loop-closure
*non-event*.

Small per-scan z-errors accumulate monotonically because nothing bounds them.
Returning to a previously-mapped location should create an implicit
loop-closure factor and pull the graph flat, but that factor only exists if
VGICP converges between the lap-1 and lap-2 submaps — and VGICP only converges
if the accumulated offset is already inside its voxel convergence basin. Past
that threshold the very mechanism that would correct the drift is the one the
drift has switched off. Widening the basin alone is not enough either: at some
drift magnitude any finite basin loses, so the drift also has to be prevented
from accumulating. That is why §4 changes both a basin knob and a GNSS knob;
either alone leaves the failure reachable.

**Verifying it after a run.** Four checks, in the order worth doing them:

1. **GNSS factors are being inserted.** Read the machine-parseable
   `gnss_global summary:` line at exit. `factors_delivered` should grow
   steadily — one factor per associated submap, *independent* of
   `min_baseline`, which gates only the one-shot world↔UTM fit and never the
   emission loop. If the count stays low, GNSS is being filtered: check
   `gap_unanchored` (RTK dropouts), `yaw_gate_skips`, and
   `submaps_dropped_no_bracket`, and confirm the driver's reported covariance
   is not so pessimistic that the RTK gate rejects every sample.
2. **Loop-closure factors fire when lap 2 begins.** Pose-graph edges between
   temporally distant submaps should appear in the viewer as the vehicle
   re-enters mapped ground.
3. **`T_world_utm.txt` is written once and stays stable.** If the run finishes
   without it, GNSS was never aligned — compare `min_baseline` against the
   distance actually travelled before the first RTK lock.
4. **Trajectory altitude tracks GNSS altitude.** A widening gap between the two
   over successive laps is the drift, directly plotted.

**Escalation if the shipped defaults are not enough.** Widen
`submap_voxel_resolution_max` further, or raise `prior_inf_scale` /
tighten `prior_inf_floor` + `prior_inf_cap` so GNSS carries more weight against
the LiDAR factor mass. **Do not lower `min_baseline`** — see the correction in
§4; it adds no factors and degrades the world↔UTM fit.

### Moving-start initialization — the pathway behind §5–§7

Upstream GLIM estimates the world-frame orientation from the accelerometer mean
over the first `initialization_window_size` seconds, which is only valid if the
IMU is stationary throughout that window. This fork replaces that with an
external INS prior taken from the Atlas Duo, so a recording may begin with the
vehicle already moving.

**Gate semantics.** SLAM starts only when a validated, fresh RTK-**Fixed**
solution exists. That is a deliberate choice, not a convenience default: the
map origin has to be globally referenced and the INS attitude validated before
any factor is built. Until then the init watchdog warns every 10 s with the
current reject reason, in replay as well as live.

**Why the gate tests smoothness rather than stationarity.** The original
stability check compared raw inter-sample displacement against
`ins_max_pose_jitter_trans` (0.05 m). At the Atlas Duo's 10 Hz pose rate that is
a hard ceiling of 0.5 m/s — it forbids exactly the moving start the feature
exists to support, because it cannot distinguish INS noise from real vehicle
translation. Both halves of the gate are now residual-based: a
constant-velocity residual on position and a constant-angular-rate residual on
orientation, plus a contiguity requirement. An orientation test of the form
`|q1·q2| > 0.999` has the same defect in the rotational axis — it silently caps
yaw rate at roughly 25°/s — so it was replaced rather than kept.

A second, quieter defect lived in that same threshold and is fixed as of
2026-07-27. For unit quaternions `|q_a·q_b| = cos(θ/2)`, so a dot-product bound
always admits **twice** the angle a reader would compute from it. `0.999` was
documented as "2.5°" throughout this fork but actually admitted **5.125°**, and
the dual-antenna `0.9999` was documented as "0.8°" but admitted **1.621°**. The
bound is now configured in degrees (`ins_max_attitude_residual_deg`, default
`2.5`, auto-tightened to `0.8` in dual-antenna mode) and converted to a dot
product internally, so the unit is carried in the name and this class of error
cannot recur. The gate is genuinely 2× tighter than in any run recorded before
that date; if an old bag now fails to initialize on attitude, raise the degree
value deliberately rather than reverting, because 5.125° was never an intended
bound.

**Initial velocity.** The INS velocity is used directly rather than
finite-differenced from pose, which is why `nav_msgs/Odometry` is the preferred
initialization source. Point One reports this velocity in the platform body
frame (forward-left-up); GLIM expects it in the world frame, so the fork
applies `v_world = T.linear() * v_body`. Passing it through unrotated
initializes the estimator along the wrong world axis by the vehicle's heading —
at 90° yaw, a 20 m/s forward velocity is injected sideways.

**Through an RTK dropout.** The shipped profile never accepts Float as a GNSS
factor. Once a session has initialized from Fixed, losing RTK simply makes the
adapter's Fixed-only topic go quiet and LiDAR–IMU SLAM continues unanchored;
factors resume when Fixed returns. Loosening the NavSatFix covariance threshold
cannot bypass the authoritative solution-type check, and is not a supported way
to start a run without RTK.

**Verifying a moving start.** Confirm the run initialized at non-zero speed and
that the reported initial velocity magnitude matches the INS. Expect the
optimizer to take somewhat longer to settle than from a standing start, since
there is more IMU bias to estimate from a non-zero initial state;
`scripts/check_init_stationarity.py` remains available as an informational
pre-flight diagnostic and CI gate, but is no longer a precondition.

### Session-long heading correction — data path in place, factor deferred

The factor bridge already stamps the correct orientation covariance on every
published pose, and in dual-antenna mode the yaw σ is tight. Upstream
`gnss_global` ignores covariance entirely and consumes position only, so that
orientation information currently stops at the bridge. Two ways to use it:
patch `gnss_global` to add an orientation-with-covariance factor when yaw σ is
tight (invasive in an upstream module), or add a separate extension module that
subscribes to the bridge topic, extracts yaw and yaw σ, and contributes a
yaw-only prior to the global graph (cleanly separated, opt-in via
`extension_modules`). The data path for either is already in place; the factor
module is deliberately left to a future iteration.

### TF verification — extrinsics against the 3D design

The P1 and Robin W extrinsics were re-verified end to end after the upstream
merges, from the OpenSCAD source through to the GLIM configs:

```
3D files/sensor_dome.scad          (single geometric source)
        │
        ▼
config/sensor_dome_tf.yaml         (single TF source of truth)
        │  generate_sensor_dome_urdf.py     │ launch: one static_transform_publisher
        ▼                                   ▼ per YAML entry (tf2_ros)
GLIM_plusplus/config/sensor_dome.urdf
        │
        ▼
config_sensors.json (T_lidar_imu via urdf_path, lidar_concat aux frames)
config_gnss_global.json (lever arm)
```

All checks pass:

| Frame | YAML / URDF value | Design derivation | STL measurement |
|---|---|---|---|
| `lidar_front_link` | (0.080, 0, 0.1145), yaw 0° | ring r = 80 mm @ 0° | 4× M6 holes within **0.05 mm** of nominal |
| `lidar_rear_left_link` | (−0.040, 0.069282, 0.1145), yaw 120° | 80·(cos 120°, sin 120°) = (−40, 69.282) | within **0.05 mm** |
| `lidar_rear_right_link` | (−0.040, −0.069282, 0.1145), yaw 240° | 80·(cos 240°, sin 240°) | within **0.05 mm** |

The z offset of 0.1145 m follows from the mount surface at 6 mm (L1 plate) +
133 mm (pillar) = 139 mm against the Atlas Center of Navigation at 6 + 18.5 =
24.5 mm. Measured unibody height of 151.0 mm confirms 139 mm as the as-printed
value. Yaw-only rotations are valid because the Seyond driver is pinned to
`coordinate_mode:=3` (REP-103), which remaps the native Robin W axes at the
driver.

For the IMU: the Atlas assembly drawing places the Center of Navigation at
(68.7, 47.8) mm from the hole-pattern datum and 18.5 mm above the mounting
surface, and the SCAD positions the part so the CoN lands on the design origin.
The four M4 mount features measure within 0.1 mm of their SCAD positions over a
220 × 100 mm span. `enable_lever_arm` is `false` and `urdf_gnss_frame` empty,
which is correct for a tightly-coupled Atlas Duo that already outputs pose at
the CoN. `sensor_dome.urdf` is byte-identical to a fresh regeneration from the
YAML, so no second copy can drift.

Two standing caveats. The TF treats `imu_link` axes as vehicle-aligned; the
Atlas nav-frame alignment to its mechanical orientation is a Point One output
configuration and is not derivable from the 3D files (assumed correct, and
validated by prior on-vehicle runs). And `gnss_antenna_primary_link` z = 0.273
is a nominal stand height that should be measured per installation — it is
documentation-only for GLIM, since the lever arm is disabled.

### Upstream merge record

The §14 summary below is the merge record. It states what was ported from
`ucb-roar` and PR #15, what was deliberately skipped, and how each dome-specific
behavior survived conflict resolution. Upstream sources are linked directly:
[`koide3/glim`](https://github.com/koide3/glim) for the base project and
[`augcog/DLIO_plusplus`](https://github.com/augcog/DLIO_plusplus) for the
hardening branch.

## 11. What was NOT changed

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

## 12. File-by-file diff summary

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
| All other upstream files | unchanged | §10 |


## 13. 2026-07 P1–P5 improvements

Added 2026-07-05 (the P1–P5 turn-error campaign, plus review fixes); every
dome adaptation above (Robin W sensors, INS-driven init, RTK gating,
dual-antenna yaw prior, multi-lap loop tuning) is preserved.

**lidar_concat extracted + hardened** (`glim_ros2/include/glim_ros/lidar_concat.hpp`,
replacing the in-file copy in `glim_rosbag.cpp`):

- Full PointCloud2 **schema-equality gate** before byte-appending an aux scan
  (name/offset/datatype/count + point_step + endianness) plus tight-cloud
  guards — a malformed aux is rejected BEFORE it is appended (all guards are
  pre-append; nothing is rolled back after the fact).
- **Strict merge guard** (`require_all_aux` / `abort_on_merge_failure` /
  `max_consecutive_aux_merge_failures`, config-exposed, GICP++-parity): an
  incomplete *required* merge skips the scan instead of silently mapping on
  fewer LiDARs.
- **Per-frame merge evidence** (`frame_diag_log: true`): one parseable
  `CONCAT DEBUG | stamp=… merged=n/N dt<i>=…s pts<i>=… span=…s total_pts=…`
  INFO line per primary scan, plus per-aux signed header-offset stats every
  512 merges. (The |mean| > 20 ms warning documented here was **removed** in
  the 2026-07-27 re-merge — see §14: the header dt is acquisition phase, not
  a residual clock-offset estimate, and must never be copied into a
  correction.)
- Robin W note: per-point time handling was reworked in the 2026-07-27
  re-merge and is now **encoding-agnostic**. See §14 — including an open
  question about which encoding the driver actually emits.

**GNSS global module** (extends §7/§8):

- **Backfill fix**: prior factors are emitted for *every* associated submap
  (cursor-based), not just the newest — the pre-`T_world_utm` backlog and
  multi-submap offline batches were silently dropping factors.
- **`needs_wait()` drain contract** so `save()` cannot serialize the graph
  while bracketing GNSS factors are still in flight (bag-EOF race).
- **P5 yaw-quality gate** (`orientation_prior_max_yaw_sigma_deg: 3.0`) on the
  §8 dual-antenna heading prior: a position-FIXED sample whose reported yaw
  sigma (√`pose.covariance[35]`) is degraded skips the heading factor
  (position prior still applied; skips logged). Interpolated samples carry
  the conservative max of the bracketing variances. Healthy dual-antenna
  heading is 0.1–0.3° sigma at a ≥1 m baseline. Publishers that leave the
  covariance unpopulated keep the old behavior.
- Hitch defaults now match the checked-in single-antenna sentinel:
  `enable_orientation_prior: false`. Set it true only when the secondary
  antenna translation is configured; yaw weighting remains
  `[1e-6, 1e-6, 1e2]`.

**Offline pipeline**: epoch-anchored multi-LiDAR rebase plumbing
(`points_callback(msg, epoch_anchor_count)`) — engages only for absolute
per-point time encodings (see §14), kept for parity with GICP++'s concat.
GLIM maps
OFFLINE by default (`glim_ros/enable_online_mapping: false`):
`glim_rosbag` feeds `/gps_p1/filtered_odom_rtk_fixed` + `/gps_p1/fix` from the
bag into the same INS init gate and RTK-gated GNSS factor bridge the live
path uses. The adapter must run during capture so its Fixed-only output is
present in the bag. Flip the flag deliberately if you map live.

**Dense localization-map profile — NOT enabled here.** A denser
preprocess/submap profile exists as an option, but the Robin W density/CPU
budget is unvalidated, so `config.json` keeps the platform defaults with a
commented pointer. Derive dense variants from this platform's configs,
rebuild a map, then **re-baseline the GICP++ fitness floor** with
`GICP_plusplus/scripts/analyze_scan_debug_log.py` before tuning any GICP++
ratio thresholds.

## 14. 2026-07-27 upstream re-merge

Re-synced against `augcog/DLIO_plusplus` branch `ucb-roar` (fork point
`edfede3` → tip `69eb574`) plus upstream **PR #15** (`19c1537`+`2aeb05f`,
still an OPEN PR at merge time). Every dome adaptation in §1–§13 is
preserved: P1 GNSS stays the GNSS source with its own bridge contract
(`/gnss/pose_rtk_only`, `PoseWithCovarianceStamped`), Robin W stays the
LiDAR, and the Naive/INS-driven initialization is unchanged. Full merge
record: §10 "Upstream merge record" and the per-area detail below. Upstream
sources: [`koide3/glim`](https://github.com/koide3/glim) and
[`augcog/DLIO_plusplus`](https://github.com/augcog/DLIO_plusplus).

> The shipped production path now also starts from
> `/gps_p1/filtered_odom_rtk_fixed`. The local bridge retains its
> `PoseWithCovarianceStamped` output contract so map-status accounting and
> dual-antenna covariance propagation remain unchanged.

**Per-point time and multi-LiDAR merge — the headline change.** Aux sweep
selection no longer trusts header time. Where an *absolute* per-point time
axis is decodable, an aux sweep is accepted only when its point-time
endpoints match the primary's within `sweep_time_threshold` (10 ms);
header time survives only as a tie-break and as a scheduling hint
(`aux_match_time_offsets`). Residual clock corrections are a separate,
explicitly-measured knob (`aux_point_time_offsets`) applied to point times,
never inferred from the header phase. The offline reader gained a
deterministic **two-pass point-time join** (`two_pass_point_time_join`):
pass 1 indexes every scan's point-time range and plans each merge, pass 2
releases each primary exactly when its planned sweeps have arrived, bounded
by `future_sweep_wait_timeout`. Where no absolute axis exists, the reader
instead holds each primary until every aux header passes it (PR #15), which
fixes the "always merge the *previous* side sweep" bias of naive
header-nearest matching. New `lidar_concat` keys in `config_sensors.json`:
`sweep_time_threshold`, `future_sweep_wait_timeout`,
`two_pass_point_time_join`, `aux_match_time_offsets`,
`aux_point_time_offsets` (all zero on the PTP-locked dome).

> #### Robin W timestamp contract
>
> The Hitch Sensor Dome profile is definitive: its canonical Seyond Robin W
> topic carries `timestamp/FLOAT64/count=1`, numeric Unix seconds. The pinned
> driver reconstructs each value from the packet's absolute start time plus
> the compact per-point offset. `coordinate_mode:=3` controls axes only.
>
> GLIM decodes the values as IEEE-754 doubles, verifies that they are absolute,
> then TimeKeeper converts them to offsets from the first point for deskew.
> The offline concat planner matches sweeps on their absolute endpoint ranges,
> and `shift_cloud_timestamps()` does not add the inter-header delta to Robin W
> points. `float64_time_is_epoch_ns` remains false; that flag is only for a
> legacy raw-uint64 encoding mislabeled FLOAT64.
>
> See the root README's
> [LiDAR per-point timestamp standard](../README.md#lidar-per-point-timestamp-standard)
> for the complete five-vendor contract and the live-topic validation command.

**LiDAR quality gate.** GLIM++ measures every raw stream before preprocessing
and measures each primary/auxiliary separately before offline concatenation.
The shipped Robin W profile requires a robust vertical elevation span of at
least 27 degrees (30 degrees nominal) and at least 100 finite, nonzero sampled
returns. The outer 0.5 percent of samples is trimmed at each end so one outlier
cannot hide a narrow stream. Invalid clouds are logged and rejected before
SLAM; the first passing cloud from each sensor is logged as startup evidence.
The run-config generator always emits `lidar_quality.min_vertical_fov_deg` and
`lidar_quality.min_valid_points`.

**GNSS global module** (extends §7/§8/§13):

- **RTK-dropout un-anchoring** (`max_interp_gap_sec: 1.0`): a submap whose
  bracketing GNSS samples straddle a dropout is left un-anchored rather than
  pinned to a straight-line chord between dropout entry and exit — the chord
  sagitta was warping curved GNSS-denied segments at ~1 cm stiffness.
- **One-shot `T_world_utm` fit gates**: post-fit RMS residual gate
  (`fit_max_rms`), *both-sides* `min_baseline` requirement (a frozen GNSS
  could previously latch a garbage rotation forever), and an optional
  recent-suffix fit (`fit_recent_baseline_window`) that rejects
  stationary-launch transients.
- **Covariance-adaptive position priors** (PR #15): precision =
  `clamp(1/variance, prior_inf_floor, prior_inf_cap)` per axis, with optional
  Huber wrapping (`position_prior_robust_width`). Fed by the per-sample
  covariance the P1 bridge already publishes. **Off by default** (negative
  floor/cap ⇒ legacy fixed `prior_inf_scale`).
- **Anchor-divergence health gate** (PR #15): rolling median of the
  *post-optimization* GNSS residual; if it exceeds `anchor_abort_median_m`
  for `anchor_abort_consecutive_updates` global updates, the module reports
  unhealthy via `ok()` and the offline run aborts instead of writing a
  quietly-misanchored map. **Off by default** (`0.0`).
- **Optional gravity prior** (`gravity_prior_sigma_deg`, PR #15): a
  `Pose3AttitudeFactor` constraining roll/pitch from the INS orientation
  without touching yaw. **Off by default** — enable only for a validated
  fused-INS attitude stream (some publishers emit identity quaternions as an
  "unavailable" placeholder).
- **Delivery accounting**: emitted-vs-delivered factor counts and a
  machine-parseable `gnss_global summary: …` line at exit
  (`factors_undelivered`, `submap_anchor_coverage`, `anchor_residual_median_m`,
  bracket stats). Gate map acceptance on this line, not on a clean exit code —
  a run can exit 0 while being completely un-anchored.
- Also fixed: submap-origin snapshot race against post-optimization rewrites,
  GNSS stamp monotonicity/non-finite guards, `T_world_utm.txt` write mutex,
  and an offline-replay throttle caused by the old `needs_wait()` predicate.

**Core / ROS**

- **Epoch-reset detection** in `TimeKeeper`: an IMU or LiDAR timestamp rewind
  > 5 s (bag loop, sensor power-cycle) is reported loudly and stalls, instead
  of silently mixing epochs through the preintegration and GNSS queues.
- `points_time_offset` moved *inside* `TimeKeeper::process()` so it survives
  the absolute-time stamp overwrite (it was previously discarded for
  absolute-time clouds).
- `fix_imu_bias` now pins every state to the **initial** bias, not the
  unconfigured (zero) `sensors/imu_bias`.
- URDF `T_lidar_imu` override is verified to have persisted to disk and
  aborts loudly on a read-only install prefix (it is consumed by other
  modules *via* that file).
- Online mapping combined with `lidar_concat` is now **refused at startup**:
  the live path has no future-sweep release and would silently drop the late
  aux sweep. Build concat maps offline with `glim_rosbag` — which is already
  this platform's default (`enable_online_mapping: false`), so no
  operational change here.
- Optional `sensors/imu_input_rotation` quaternion (PR #15) for a measured
  IMU mount tilt; identity here.
- **Long-run pose-graph quality** (PR #15): loop-candidate bounding,
  correction-magnitude rejection gates, and bounded forced relinearization in
  `global_mapping_pose_graph`. Only active with the pose-graph backend — this
  platform runs the GPU `GlobalMapping` backend, so it is currently latent.
  A post-merge **P2 fix** was applied there so extension-module factors are
  delivered on the final flush (details in the merge doc).

**Tests**: `glim_ros2/test/lidar_concat_point_time_test.cpp` (12 cases:
upstream point-time matching + the fork's Robin W encoding cases). Built
under `BUILD_TESTING`; run with `colcon test --packages-select glim_ros`.

**Tooling** (PR #15): `scripts/generate_glim_mapping_config.py` generates a
self-contained high-quality mapping profile (defaults adapted to this
platform's Robin W topics/frames; everything overridable via CLI), and
`scripts/export_glim_dump_to_pcd.py` exports a dump to PCD.

**TF verification**: the P1 and Robin W extrinsics were re-verified against
the 3D design after the merges, from the OpenSCAD source through the URDF to
the GLIM configs. Results and caveats are tabulated in §10 "TF verification".

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

Optional extras:

```bash
# Unit tests (per-point time decoding + aux sweep matching, 12 cases)
colcon build --packages-select glim_ros --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select glim_ros && colcon test-result --verbose

# Generate a self-contained high-quality mapping profile for one run
# (defaults target this platform's Robin W topics/frames; --help for all knobs)
python3 GLIM_plusplus/scripts/generate_glim_mapping_config.py \
    --imu-topic /gps_p1/imu \
    --t-lidar-imu -0.080 0 -0.1145 0 0 0 1 \
    --urdf-path GLIM_plusplus/config/sensor_dome.urdf \
    --aux-lidar /robin_w_rear_left/points:lidar_rear_left_link \
    --aux-lidar /robin_w_rear_right/points:lidar_rear_right_link \
    --offload-dir /tmp/glim_run1 --output-dir glim_profiles/run1

# Export a finished dump to a single PCD
python3 GLIM_plusplus/scripts/export_glim_dump_to_pcd.py --help
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

The integration work in `GLIM_plusplus/{config, launch, scripts}/` and the modifications detailed in §1 – §7 are part of the **Hitch Sensor Dome** project, designed and maintained by Dr. Allen Y. Yang (Hitch Interactive · University of California, Berkeley). Implementation testing by the **Berkeley AI Racing Tech** team (see [`../README.md`](../README.md) Credits).

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
