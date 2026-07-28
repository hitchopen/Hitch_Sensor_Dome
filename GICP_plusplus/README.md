# GICP++ — A heavily modified fork of DLIO for Hitch Sensor Dome

> **This is not stock DLIO.** The folder name `GICP_plusplus/` is intentional: this version diverges from upstream [`vectr-ucla/direct_lidar_inertial_odometry`](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) in ways that change algorithmic behavior, not just configuration. The DLIO foundation is credited to **Kenny J. Chen, Ryan Nemiroff, and Brett T. Lopez (UCLA VECTR Lab)**. Registration now uses Kenji Koide's [`small_gicp`](https://github.com/koide3/small_gicp), migrated through the reviewed `augcog/DLIO_plusplus` implementation. See [Credits](#credits-and-license).

This document is a **complete change log** between GICP++ and upstream `vectr-ucla/direct_lidar_inertial_odometry`. It is organized so an adopter can decide, mechanism by mechanism, which changes apply to their use case and which would need to be reverted.

The supported ROS 2 targets are **Humble on Ubuntu 22.04** and **Jazzy on
Ubuntu 24.04**. Humble compatibility is a hard requirement for the official
Seyond deployment path; GICP++ and the adapter therefore stay on the common
Humble/Jazzy C++17 and `rclcpp` API surface.

## Index of changes

1. [Hardware retargeting — Robin W + Atlas Duo](#1-hardware-retargeting)
2. [Two-mode operation — race vs. safe](#2-two-mode-operation)
3. [RTK-gated INS odometry republisher (`nav_sat_gated_odom`)](#3-rtk-gated-ins-odometry-republisher)
4. [GLIM++ map bridge (`merge_glim_submaps.py`)](#4-glim-map-bridge)
5. [small_gicp backend and warm-start](#5-small_gicp-backend-and-warm-start)
6. [Yaw-rate-adaptive observer gains](#6-yaw-rate-adaptive-observer)
7. [Motion-variance gate on stationary IMU calibration](#7-motion-variance-gate)
8. [Operator-side health checks](#8-operator-side-health-checks)
9. [`base_link` / `imu_link` frame split](#9-base_link--imu_link-frame-split)
10. [Race-mode parameter tuning](#10-race-mode-parameter-tuning)
11. [What was removed from upstream](#11-what-was-removed-from-upstream)
12. [What was NOT changed](#12-what-was-not-changed)
13. [File-by-file diff summary](#13-file-by-file-diff-summary)
14. [2026-07 P1–P5 improvements — confidence-weighted gating, delta observer, merge evidence](#14-2026-07-p1p5-improvements)
15. [Credits and license](#credits-and-license)

## 1. Hardware retargeting

Topic, frame, and URDF defaults throughout the configs target the Hitch Sensor Dome (3× Seyond Robin W + Point One Atlas Duo + 4× e-con RouteCAM), matching [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml).

| Surface | Hitch Sensor Dome setting |
|---------|---------------------------|
| Primary LiDAR topic | `/robin_w_front/points` |
| Aux LiDAR topics | `/robin_w_rear_left/points`, `/robin_w_rear_right/points` |
| IMU topic | `/imu/data` live default; `/gps_p1/imu` supported for normalized replay |
| GT odom topic | `/gps_p1/filtered_odom_rtk_fixed` (adapter fixed-only output) |
| ENU datum metadata | `/gps_p1/local_enu_origin` (`std_msgs/String`, transient-local) |
| `base_frame` | `base_link` (see §9) |
| `imu_frame` | `imu_link` (Atlas Duo CoN) |
| `lidar_frame` | `lidar_front_link` |
| Aux LiDAR frames | `lidar_rear_left_link`, `lidar_rear_right_link` |
| URDF auto-discovery | walks up for `GLIM_plusplus/config/sensor_dome.urdf` |
| `sensor_type` | `seyond` (`timestamp/FLOAT64`, numeric Unix seconds) |
| Absolute timestamp handling | Robin W uses numeric `FLOAT64` Unix seconds, a 10 us point-time quantum, and a 100 ms frame contract |

Files touched: [`cfg/localization.yaml`](cfg/localization.yaml), [`launch/localization_with_tf.launch.py`](launch/localization_with_tf.launch.py), [`include/dlio/dlio.h`](include/dlio/dlio.h), [`include/gicp_localization/localization.h`](include/gicp_localization/localization.h), [`src/localization.cc`](src/localization.cc).

## 2. Two-mode operation

Upstream DLIO ships one config. GICP++ ships two clearly-defined modes selectable at launch time via `mode:=race|safe|custom`.

The base [`cfg/localization.yaml`](cfg/localization.yaml) carries **race-mode** defaults; [`cfg/localization_safe.yaml`](cfg/localization_safe.yaml) is an overlay applied on top when `mode:=safe`. ROS 2's parameter chain layers the overlay's keys over the base, so the safe-mode YAML only carries the entries that differ.

### Side-by-side knob comparison

| Knob | 🏁 Race mode | 🛡 Safe mode | Why they differ |
|---|---|---|---|
| `lidar_concat/enabled` | `false` (front-only) | `true` (3× LiDARs) | Race trades 360° coverage for ~50 ms less jitter + ~3× faster GICP. Safe restores full coverage. |
| `cropBoxFilter/size` | `40.0` m | `100.0` m | Race clips to the immediate vicinity. Safe uses ~Robin W's effective range. |
| `voxelFilter/res` | `0.5` m | `0.3` m | Safe spends ~2.4× more points per scan on cleaner alignment. |
| `gicp/maxIterations` | `32` | `128` | Race caps iterations; safe uses the upstream safety margin. |
| `gicp/transformationEpsilon` | `0.004` | `0.001` | Safe insists on tight convergence. |
| `gicp/rotationEpsilon` | `0.004` | `0.001` | Same. |
| `gicp/correspondenceRandomness` | `20` | `30` | Safe uses more NN samples for more stable covariances. |
| `yawrate_attenuation/enable` | `true` | `false` | Race protects against corner slides; safe gives GICP full authority. |
| `calib/motion_sigma_max` | `0.10` m/s² | `0.05` m/s² | Safe insists on a quieter window for higher-quality bias estimate. |
| `gt_recovery/min_consecutive_failures` | `5` (shipped; raised from 3 in the P2 turn-error fixes) | `1` | Race avoids snap on transient corners; safe restores upstream behavior. |
| `verbose`, `debug/enable_jump_log` | `false` | `true` | Race kills the noisiest per-scan logging; safe turns the firehose on. |
| `debug/enable_pub`, `debug/verbose_scan_log` | `true` (evidence-first defaults, §14 — the scorecard consumes them; ~14 MB/36 min) | `true` | Disable in race mode only for resource-constrained live deployment. |

Each mode also gets its own systemd unit:

- [`launch/gicp_localization-race.service`](launch/gicp_localization-race.service): `SCHED_FIFO` priority 50, `CPUAffinity=4-7`, `Nice=-15`, `OOMScoreAdjust=-1000`.
- [`launch/gicp_localization-safe.service`](launch/gicp_localization-safe.service): default scheduling, unlimited cores, `OOMScoreAdjust=-500`.

The two units carry `Conflicts=` directives so starting one auto-stops the other; you can never accidentally run both.

Files touched: [`cfg/localization.yaml`](cfg/localization.yaml), [`cfg/localization_safe.yaml`](cfg/localization_safe.yaml), [`launch/localization_with_tf.launch.py`](launch/localization_with_tf.launch.py), [`launch/gicp_localization-race.service`](launch/gicp_localization-race.service), [`launch/gicp_localization-safe.service`](launch/gicp_localization-safe.service).

## 3. RTK-gated INS odometry republisher

Production uses the adapter's `/gps_p1/filtered_odom_rtk_fixed` output. It is
published only when FusionEngine reports `solution_type == RTK_FIXED` and all
position variances are finite, positive, and within the configured bounds.
Initial pose, RTK calibration, diagnostics, and recovery apply the same
fail-closed covariance gate. RTK float or no-fix therefore falls back to
LiDAR+IMU state estimation.

`nav_sat_gated_odom` remains as an optional compatibility bridge for legacy
bags. It requires a fresh, non-future NavSatFix, non-UNKNOWN finite positive
covariance, an RTK-class status, and the configured sigma limit. It never
reuses a stale fix. Because REP-145 `STATUS_GBAS_FIX` does not distinguish
RTK float from RTK fixed, this bridge is not the production discriminator.
The launch default is `run_rtk_gate:=false`.

Files added: [`src/nav_sat_gated_odom.cc`](src/nav_sat_gated_odom.cc).  
Files touched: [`CMakeLists.txt`](CMakeLists.txt), [`launch/localization_with_tf.launch.py`](launch/localization_with_tf.launch.py).

## 4. GLIM++ map bridge

Use [`../GLIM_plusplus/scripts/export_glim_dump_to_pcd.py`](../GLIM_plusplus/scripts/export_glim_dump_to_pcd.py).
It reads the exact `float32 × 3` compact format, applies
`T_world_origin`, then `inverse(T_world_utm)`, and writes the PCD in the same
surveyed local-ENU frame as the adapter. It also writes
`<map>.manifest.yaml` with the datum and transform provenance.

GICP++ requires that manifest by default and compares its datum with
the adapter's transient-local `/gps_p1/local_enu_origin` metadata within
`localization/enu_origin_tolerance_m`. Point clouds, initial poses, and GT
odometry are ignored until this live check passes. For legacy offline bags
without metadata, set `require_live_enu_origin=false` and provide
`expected_enu_origin` explicitly. Leave
`localization/utm_transform_path` empty: the ENU transform has already been
applied. [`scripts/merge_glim_submaps.py`](scripts/merge_glim_submaps.py) is
now only a compatibility wrapper around this canonical exporter.

## 5. small_gicp backend and warm-start

The old nanoGICP/nanoflann implementation has been replaced by the vendored,
header-only `small_gicp` backend. The adapter preserves the localizer's matcher
call surface while using small_gicp's parallel kd-tree, covariance estimation,
GICP factors, and OpenMP reduction. It also preserves the Hitch safety
contract: non-finite results fail closed, the reported Hessian is re-linearized
at the final pose, and a degeneracy-projected pose is rescored before use.
Registration still optimizes small_gicp's Mahalanobis energy, while the public
fitness gate preserves nanoGICP's calibrated contract: mean squared Euclidean
nearest-neighbor distance over every source point, in square metres. The
independent `gicp/minCorrespondenceRatio` floor also rejects solutions
supported by too little in-radius scan mass.

Large-map handling adopts the device-neutral part of
[`augcog/DLIO_plusplus#14`](https://github.com/augcog/DLIO_plusplus/pull/14):
the complete GLIM map is voxelized and assigned covariances once at startup,
then each registration uses a bounded XY target around the predicted pose.
Local targets copy each point together with its cached full-map covariance and
build only their KD-tree. An asynchronous rebuild starts before the active
target loses scan coverage; if it is late, the scan thread rebuilds
synchronously and continues LiDAR+IMU localization. It does not authorize a
GNSS reset. Snap-back still requires the repository's fresh RTK-fixed stream
and fail-closed covariance gate.

The PR's wrong-lock check is retained only as
`localization/gt_recovery/sanity_radius`: a fresh, time-matched RTK-fixed pose
may reject and immediately recover a GICP candidate outside that radius.
RTK-float, stale, unknown-covariance, and absent fixes cannot operate this gate.
The lookup uses the median point time represented by `T_prior`, rather than the
Robin W frame-start header stamp.

The shipped controls are `localization/local_map/enabled`, `radius`,
`min_points`, and `build_threads`. Disabling `enabled` restores full-map
registration. The local radius must exceed `sqrt(2) * cropBoxFilter/size +
maxCorrespondenceDistance`, which is validated at startup.

The **first real `gicp.align()`** would still pay first-touch overhead for
OpenMP, Eigen, source-tree allocation, and map pages.

The adapter exposes upstream's optional ground-vehicle LM constraints through
`gicp/dof/*` and `gicp/prior/*`. The shipped mode remains `6dof`, with prior
information disabled, to preserve the validated nanoGICP-era behavior during
the backend migration. Enable `4dof` or nonzero prior information only after a
Robin W replay has established constraint and correction baselines. The
Mahalanobis optimizer energy remains available as `final_error` diagnostics;
it is not compared with the Euclidean fitness thresholds.

GICP++ runs **one dummy align** at init against ~200 randomly-sampled map points to burn those costs at startup instead of on the first localization scan:

```cpp
auto dummy_src = std::make_shared<pcl::PointCloud<PointType>>();
pcl::RandomSample<PointType> rs;
rs.setInputCloud(this->map_cloud);
rs.setSample(std::min<unsigned int>(200, this->map_cloud->size()));
rs.filter(*dummy_src);

this->gicp.setInputSource(dummy_src);
pcl::PointCloud<PointType> aligned_scratch;
this->gicp.align(aligned_scratch, Eigen::Matrix4f::Identity());
```

Logs the wall time at INFO so operators see it land cleanly before the first scan arrives. Free to run; no configuration knob.

Files touched: [`include/gicp_plusplus/small_gicp_backend.hpp`](include/gicp_plusplus/small_gicp_backend.hpp),
[`include/gicp_localization/localization.h`](include/gicp_localization/localization.h),
[`src/localization.cc`](src/localization.cc), and
[`cfg/localization.yaml`](cfg/localization.yaml).

## 6. Yaw-rate-adaptive observer

Upstream DLIO's geometric observer uses fixed gains (`Kp`, `Kq`, `Kv`, `Kab`, `Kgb`) for position / orientation / velocity / bias corrections. At high yaw rate, two things happen simultaneously: (a) GICP is most likely to slide along an unconstrained axis, and (b) the IMU integration is at its most informative (gyro doing real work). Upstream applies the same Kp/Kq regardless — the scan corrections drag the optimizer along even when GICP isn't reliable.

GICP++ adds an attenuation block in `updateState()` that scales `Kp` and `Kq` (but NOT `Kv`/`Kab`/`Kgb` — bias estimation still benefits from the small corrections) based on `|state.v.ang.b[2]|` (body-frame yaw rate):

| `|ω_z|` | Effective Kp / Kq |
|---|---|
| ≤ `threshold_rad_s` (default 0.5 rad/s ≈ 29 °/s) | full gain (×1.0) |
| ≥ `saturation_rad_s` (default 1.5 rad/s ≈ 86 °/s) | minimum gain (×`min_gain_scale`, default 0.25) |
| in between | linear interpolation |

Logged via throttled INFO when the scale drops below 0.99 so operators see the gate firing in real time during testing.

New YAML knobs:
- `odom/geo/yawrate_attenuation/enable` (default `true` in race mode, `false` in safe mode)
- `odom/geo/yawrate_attenuation/threshold_rad_s` (default 0.5)
- `odom/geo/yawrate_attenuation/saturation_rad_s` (default 1.5)
- `odom/geo/yawrate_attenuation/min_gain_scale` (default 0.25)

Files touched: [`include/gicp_localization/localization.h`](include/gicp_localization/localization.h), [`src/localization.cc`](src/localization.cc), [`cfg/localization.yaml`](cfg/localization.yaml), [`cfg/localization_safe.yaml`](cfg/localization_safe.yaml).

## 7. Motion-variance gate

Upstream's stationary IMU calibration path (`STATIONARY_CALIBRATING`) sums IMU samples over `dlio/imu/calibTime` seconds and treats the mean as gravity/bias — without any motion check. If the bag starts in motion AND RTK is unavailable (so the `rtk_init` path doesn't fire), upstream silently produces a tilted gravity vector and wrong gyro/accel bias.

GICP++ tracks the running variance of `||linear_acceleration||` over the calibration window. At window close, if σ‖a‖ > `localization/calib/motion_sigma_max` (default 0.10 m/s² in race, 0.05 in safe), the calibration is **refused** — accumulators reset, window restarts, and a bold-yellow one-shot warning fires:

```
[WARN] Stationary IMU calibration REFUSED — motion detected
       (σ_||a||=0.342 m/s² > 0.100 m/s² over 0.5s / 100 samples).
       The vehicle appears to be moving. ...
       Resetting the window. Bring the vehicle to rest, or enable
       localization/rtk_init/enable to calibrate from RTK GT while moving.
```

Subsequent refusals get throttled INFO lines at 2 Hz. The calibration only completes when the vehicle actually comes to rest, OR the operator switches to RTK-driven calibration.

New YAML knob: `localization/calib/motion_sigma_max`.

Files touched: [`include/gicp_localization/localization.h`](include/gicp_localization/localization.h), [`src/localization.cc`](src/localization.cc), [`cfg/localization.yaml`](cfg/localization.yaml).

## 8. Operator-side health checks

Two new bold-yellow one-shot warnings to surface common operator-side misconfigurations:

**(a) `gt_odom` never arrived.** At 10 s after node start, if `gt_recovery/enable=true` AND `gt_odom/enable=true` AND zero messages have arrived on the `gt_odom` topic, the timer fires once with a diagnostic:

- **Zero publishers**: the adapter is not running or `gt_odom_topic` is wrong.
- **Publishers exist, no messages**: the adapter has not observed a genuine RTK_FIXED solution.

Healthy state prints a single confirmation INFO line.

**(b) IMU never arrived.** The check reports the adapter default
`/gps_p1/imu` and the resolved publisher count.

Files touched: [`src/localization.cc`](src/localization.cc) (gt_odom timer at ~line 490; IMU topic warn around line 450).

## 9. `base_link` / `imu_link` frame split

Upstream DLIO assumes a single body frame for both IMU integration and pose reporting. On a vehicle where the IMU is mounted in a specific spot on the dome but the downstream consumer expects pose in a vehicle-conventional body frame (rear axle, chassis center, etc.), this conflates two things that should be separate.

GICP++ splits them:

- **`imu_frame`** stays at `imu_link` (Atlas Duo Center of Navigation) — this is what GLIM++ anchors the map to, so the IMU integration and the map are in the same frame.
- **`base_frame`** defaults to `base_link`, which the URDF places relative to `imu_link` via the static `imu_link → base_link` TF in [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml). Default identity — `base_link` physically coincident with `imu_link`. Override per vehicle.

This means: the **map** is in `imu_link`; the **published pose** (`localized_pose`, `localized_odom`, `localized_path`) is in `base_link`. The localizer applies the URDF transform internally. **Changing `imu_link → base_link` does not invalidate the map** — the operator can move `base_link` per-vehicle without rebuilding.

Files touched: [`cfg/localization.yaml`](cfg/localization.yaml). Depends on the `imu_link → base_link` static TF entry in [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml).

## 10. Race-mode parameter tuning

Beyond the structural changes above, race mode also bundles latency-focused parameter defaults vs. upstream:

| Knob | Upstream-equivalent | GICP++ race default | Rationale |
|---|---|---|---|
| Robin W FPS expectation | 10 Hz typical | 20 Hz documented | LiDAR firmware setting; halves scan-to-correction interval. |
| `cropBoxFilter/size` | ~80 m | `40.0` m | At race speed anything beyond 40 m doesn't affect self-position. |
| `gicp/maxIterations` | `128` | `32` | Feature-rich pre-built map converges in 5–15 iterations. |
| All debug publishers | typically on | `false` | Per-scan publisher overhead eliminated for race mode. |
| `voxelFilter/res` | `0.3` | `0.5` | Sparse spatial density bounds GICP point count. |

Roll-back path is one YAML diff (`mode:=safe` overlay or per-key edits in `cfg/localization.yaml`).

Files touched: [`cfg/localization.yaml`](cfg/localization.yaml), [`../recording/sensor_config.yaml`](../recording/sensor_config.yaml) (FPS note).

## 11. What was removed from upstream

A small set of upstream additions that do not apply on the Hitch dome were
removed to keep the code base tight:

- **nanoGICP and its standalone nanoflann adapter.** Registration is provided
  by the reviewed, vendored small_gicp backend described in section 5.
- Unsupported vendor-specific naming from the deployed path. Generic
  compatibility decoders remain for the documented Ouster, Velodyne, Hesai,
  and Livox layouts, while the Robin W path accepts only
  `timestamp/FLOAT64/count=1` numeric Unix seconds and validates every raw
  cloud against the configured frame-period contract.
- **Earlier-deployment comment strings and topic name hints** throughout the code (`/gps_na/imu`, `novatel_a`, `gps_bottom`, `imu_bottom`, etc.).
- **VECTR copyright headers** are RETAINED on files that originated there — that's legal attribution, not project text.

## 12. What was NOT changed

Important — so adopters can rely on the existing DLIO body of work and literature:

- ROS topic names, ENU map/datum checks, RTK-fixed recovery, and the
  three-Robin merge/deskew interfaces are unchanged by the small_gicp
  migration.
- The geometric observer math (`updateState`, `propagateState`, error formulation) — unchanged structurally; only Kp/Kq scale at the application step has been added.
- The IMU integration (gravity compensation, bias estimation, deskewing) — unchanged.
- The Init-Phase state machine (`WAITING` / `RTK_CALIBRATING` / `STATIONARY_CALIBRATING` / `DONE`) — unchanged structurally; only the motion-variance gate was added inside `STATIONARY_CALIBRATING`.
- The vendored small_gicp license and source notices are preserved verbatim.
- Per-scan publisher schema and topic names — unchanged.

## 13. File-by-file diff summary

| File | Status | Change category |
|------|--------|-----------------|
| [`cfg/localization.yaml`](cfg/localization.yaml) | MODIFIED | Race-mode defaults; all sections re-commented for Hitch dome |
| [`cfg/localization_safe.yaml`](cfg/localization_safe.yaml) | NEW | Safe-mode overlay (§2) |
| [`launch/localization_with_tf.launch.py`](launch/localization_with_tf.launch.py) | MODIFIED | Topic defaults, URDF discovery, `mode` arg, `nav_sat_gated_odom` spawn |
| [`launch/gicp_localization-race.service`](launch/gicp_localization-race.service) | NEW | systemd unit, SCHED_FIFO + CPU affinity, `Conflicts=safe` |
| [`launch/gicp_localization-safe.service`](launch/gicp_localization-safe.service) | NEW | systemd unit, default scheduling, `Conflicts=race` |
| [`src/nav_sat_gated_odom.cc`](src/nav_sat_gated_odom.cc) | NEW | RTK-gated INS odometry republisher (§3) |
| [`src/localization.cc`](src/localization.cc) | MODIFIED | Seyond timestamp validation and endpoint matching + warm-start + yaw-rate attenuation + motion gate + gt_odom health + §14 P1–P5 improvements |
| [`include/dlio/dlio.h`](include/dlio/dlio.h) | MODIFIED | SensorType enum with explicit `SEYOND` absolute-seconds path |
| [`include/gicp_localization/localization.h`](include/gicp_localization/localization.h) | MODIFIED | New member fields for yaw-rate attenuation + motion gate + gt_odom timer |
| [`include/gicp_plusplus/small_gicp_backend.hpp`](include/gicp_plusplus/small_gicp_backend.hpp) | NEW | Reviewed small_gicp adapter, final-pose scoring/Hessian, and fail-closed result handling |
| [`thirdparty/small_gicp`](thirdparty/small_gicp) | NEW | Vendored header-only small_gicp implementation and MIT license |
| [`scripts/merge_glim_submaps.py`](scripts/merge_glim_submaps.py) | NEW | GLIM++ submap → single PCD bridge (§4) |
| [`scripts/convert_ply_to_pcd.py`](scripts/convert_ply_to_pcd.py) | UNCHANGED | Legacy single-file PLY→PCD; still shipped for fallback |
| [`scripts/visualize_lidar_topic.py`](scripts/visualize_lidar_topic.py) | MODIFIED | Defaults retargeted to Robin W |
| [`scripts/live_gps_localization.py`](scripts/live_gps_localization.py) | MODIFIED | Defaults retargeted to Atlas Duo |
| `scripts/debug_pose_inspector.py`, `scripts/plot_*.py`, `scripts/profile_*.py` | UNCHANGED | Generic diagnostic / plotting; not vehicle-specific |
| `include/nano_gicp/*`, `src/nano_gicp/*` | REMOVED | Replaced by small_gicp |
| [`CMakeLists.txt`](CMakeLists.txt) | MODIFIED | Header-only small_gicp target, localizer, compatibility bridge, and tests |
| [`package.xml`](package.xml) | MODIFIED | Adds `libxml2` (URDF parsing for offline extrinsics); package name stays `gicp_localization` for ROS / colcon compatibility |
| [`README.md`](README.md) | REWRITTEN | This file |


## 14. 2026-07 P1–P5 improvements

Added 2026-07-05 (the P1–P5 turn-error campaign, plus review fixes); every
dome adaptation above (warm-start, yaw-rate-adaptive gains, motion-variance
gate, race/safe modes, `base_link` split, Robin W sensor handling) is
preserved. The campaign diagnosed a family of turn-localization errors from
cross-run replays; `scripts/analyze_scan_debug_log.py` scores any replay log
against the locked pre-fix baseline.

**What came in (mechanisms):**

- **P1 — confidence-weighted GICP gating.** Per-map rolling-median fitness
  ratios (`gicp/fitnessBaseline/*`, `fitnessRatioRejectThreshold`) replace
  absolute thresholds that go stale across maps; hessian degeneracy now does
  **partial updates** (full-6D solution remapping on the vehicle-re-centered,
  unit-scaled hessian — coupled rot/trans null directions included) instead of
  binary rejects; a **yaw-consistency veto** keeps IMU yaw on low-confidence
  matches. New statuses `ok_partial` / `rejected_fitness_ratio`.
  `seedBaseline` ships **0.0 (off)** here — measure the Robin W per-map floor
  with `scripts/analyze_scan_debug_log.py` before setting it.
- **P2 — state continuity.** Rejected scans seed the next IMU prior from the
  *current* propagated velocity (stale-velocity corner-cutting bug); GT-snap
  recovery resolves linear/angular twist sources independently (gyro backfill +
  GT finite-differencing; never zeroes a moving vehicle);
  `gt_recovery/min_consecutive_failures` default 5.
- **P3 — delta-form observer + single bias point.** The observer applies GICP
  as the time-free delta `T_meas · T_prior⁻¹` to the current state (kills the
  yaw-rate-proportional lag from the 0.1–0.3 s measurement latency); IMU biases
  are subtracted once, at buffering. **Composes with** the Hitch yaw-rate gain
  attenuation (§6): the attenuation now scales the P2-bounded `dt_eff`
  corrections.
- **P4 — merge/scan evidence.** Per-frame debug topics (`fitness_ratio`,
  `degen_*`, `yaw_veto`, `merged_aux_count`, `aux<i>_merge_dt_s`,
  `aux<i>_points`, `scan_time_span_s`), the same fields in `SCAN DEBUG`,
  per-aux clock-offset stats (warns at |mean| > 20 ms), concat buffer 200,
  rejected-candidate endpoint-delta statistics for tuning the sweep gate,
  and the **turnkey scorecard** `scripts/analyze_scan_debug_log.py`
  (acceptance/streaks, gicp_ms percentiles, fitness floor + suggested ratio
  thresholds, gt_err yaw-rate buckets, concat coverage).
- **URDF-first extrinsics** (`urdf_transforms.hpp`, libxml2 dep) and the
  single-IMU allowlist guard (`/imu/data` and `/gps_p1/imu`; exactly one
  subscription is used, and frame match is required against `imu_link`).
- **Evidence-first defaults:** `debug/enable_pub` and `verbose_scan_log` are
  ON (and the `verbose:false` WARN clamp now spares the SCAN DEBUG lines).

**Robin W notes.** `localization/sensor_type: "seyond"` validates
`timestamp/FLOAT64/count=1` and interprets it as numeric Unix seconds. The
Seyond source offset has a 10 us quantum within one frame period; GICP++ does
not round or rescale the hydrated doubles. The frame period is configuration,
not a constant: `localization/seyond_frame_duration_s` defaults to `0.100`
(10 FPS, the slowest supported rate, so the default can only be permissive) and
**must be set to `0.05` when running the documented 20 FPS race configuration**
— otherwise the contract check accepts a fused double-frame and the
`lidar_concat` ambiguity ceiling widens to a whole frame period. The node
tracks the observed frame period and warns when it disagrees. The official
driver filters invalid returns before publishing and hydrates every retained
point, so a zero timestamp is invalid and rejects the whole cloud. The
deskewer uses the validated absolute
capture times directly. Aux scans merged by
`lidar_concat` remain on the shared PTP axis and are not rebased by the
inter-header delta (`lidar_concat` remains disabled by default here until
3x Robin W sweep alignment is validated; the strict-merge guard and per-frame
evidence are ready when it is enabled).

**LiDAR quality gate.** Before timestamp validation, crop, deskew, or
concatenation, GICP++ measures the first usable raw cloud from each configured
stream. It uses a near-extreme vertical elevation span after trimming 0.5% per
side. Robin W is nominally 30 degrees; the shipped profile rejects spans below
27 degrees or clouds with fewer than 100 valid sampled returns. A separate
occupancy gate divides the measured span into bins no wider than 2 degrees and
requires each bin to contain at least 0.1% of valid samples (minimum one), so a
disconnected steep-return cluster cannot hide a narrow stream at any cluster
fraction. Localization remains blocked until the primary and all configured
auxiliaries pass independently. The checks are then disabled for the
continuous run. Configure this with
`localization/lidar_quality/min_vertical_fov_deg` and
`localization/lidar_quality/min_valid_points` (accepted range 100-10000 from a
maximum 20000-point sample).

**Also included:** the Atlas `adapter` package at the repo root (optional
alternative ingestion, see
[`../adapter/README_HITCH_PORT.md`](../adapter/README_HITCH_PORT.md) —
including the P2 fix that populates `twist.angular` on INS odometry, which
the snap recovery wants from whatever GT source you use).

**Not yet re-validated on this platform:** the P1 ratio thresholds and
`hessianCondMax` (calibrated on an earlier vehicle's data), the dense GLIM
map profile, and everything needs a Robin W replay + scorecard pass. Run one
bag through, read the scorecard's suggested thresholds, then tune.

## Credits and license

Original Direct LiDAR-Inertial Odometry (DLIO):
- **Kenny J. Chen**, **Ryan Nemiroff**, **Brett T. Lopez** — UCLA Verifiable & Control-Theoretic Robotics (VECTR) Lab.
- Upstream repository: <https://github.com/vectr-ucla/direct_lidar_inertial_odometry>.

`small_gicp`:
- **Kenji Koide** — National Institute of Advanced Industrial Science and
  Technology (AIST).
- Source and license: <https://github.com/koide3/small_gicp>.
- Koide, K. *small_gicp: Efficient and parallel algorithms for point cloud
  registration.* Journal of Open Source Software, 9(100), 6948, 2024.

VECTR copyright notices are retained on every source / header file that originated upstream as legal attribution. The fork-specific changes in this folder are licensed under the same terms as the original work.

If you publish work that uses this localizer, please also cite the original DLIO paper:

> Chen, K., Nemiroff, R., & Lopez, B. T. *"Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction."* IEEE International Conference on Robotics and Automation (ICRA), 2023.

See [LICENSE](../LICENSE) at the repository root and
[`thirdparty/small_gicp/LICENSE`](thirdparty/small_gicp/LICENSE). Upstream
source notices remain intact.
