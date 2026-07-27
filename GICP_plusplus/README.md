# GICP++ — A heavily modified fork of DLIO for Hitch Sensor Dome

> **This is not stock DLIO.** The folder name `GICP_plusplus/` is intentional: this version diverges from upstream [`vectr-ucla/direct_lidar_inertial_odometry`](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) in ways that change algorithmic behavior — not just configuration. If you came here looking for the original DLIO, that lives at the link above and we strongly recommend starting there if you do not have the [Hitch Sensor Dome](../README.md) hardware. All algorithmic and implementation credit for DLIO + nano_gicp belongs to **Kenny J. Chen, Ryan Nemiroff, and Brett T. Lopez (UCLA's Verifiable & Control-Theoretic Robotics Lab)**. See [Credits](#credits-and-license).

This document is a **complete change log** between GICP++ and upstream `vectr-ucla/direct_lidar_inertial_odometry`. It is organized so an adopter can decide, mechanism by mechanism, which changes apply to their use case and which would need to be reverted.

## Index of changes

1. [Hardware retargeting — Robin W + Atlas Duo](#1-hardware-retargeting)
2. [Two-mode operation — race vs. safe](#2-two-mode-operation)
3. [RTK-gated INS odometry republisher (`nav_sat_gated_odom`)](#3-rtk-gated-ins-odometry-republisher)
4. [GLIM++ map bridge (`merge_glim_submaps.py`)](#4-glim-map-bridge)
5. [GICP warm-start at init](#5-gicp-warm-start)
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
| IMU topic | `/imu/data` (fusion_engine_driver, Atlas Duo) |
| GT odom topic | `/odom_rtk_only` (via §3 republisher) |
| `base_frame` | `base_link` (see §9) |
| `imu_frame` | `imu_link` (Atlas Duo CoN) |
| `lidar_frame` | `lidar_front_link` |
| Aux LiDAR frames | `lidar_rear_left_link`, `lidar_rear_right_link` |
| URDF auto-discovery | walks up for `GLIM_plusplus/config/sensor_dome.urdf` |
| `sensor_type` | `seyond` (`timestamp/FLOAT64`, numeric Unix seconds) |
| Absolute timestamp handling | Robin W uses numeric absolute seconds; the separate `luminar` path retains raw uint64 epoch-ns support |

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

Upstream DLIO assumes that whatever lands on the `gt_odom` topic is trustworthy — it consumes the message immediately for the bootstrap path and the GT-snap recovery path. That assumption holds when the upstream INS / RTK voter embeds quality gating; on the Hitch dome the Atlas Duo publishes `/odom` continuously regardless of `/gps/fix` status, so the assumption breaks.

GICP++ adds a small C++ executable, `nav_sat_gated_odom` ([`src/nav_sat_gated_odom.cc`](src/nav_sat_gated_odom.cc)), that subscribes to `/odom` + `/gps/fix` and republishes on `/odom_rtk_only` only when **all four gates pass**:

1. A NavSatFix has been received at least once.
2. The fix is fresh (`now − fix.stamp ≤ max_fix_age_s`, default 0.5 s).
3. `status.status >= STATUS_GBAS_FIX` when `require_rtk_fixed=true`.
4. Position σ from `position_covariance` diag is ≤ `max_position_stddev` (default 0.10 m — matches GLIM++'s factor-bridge default).

A 10 s periodic log reports `published=N rejected=M (no_fix=… stale=… status=… cov=…) last_reject="…"`. The localizer's `gt_odom_topic` defaults to `/odom_rtk_only` so the gate is transparent to the localization code — it still treats arrival as RTK-fixed, but now that's actually true.

The launch file auto-spawns `nav_sat_gated_odom` alongside the localizer via `run_rtk_gate:=true` (default). Disable with `run_rtk_gate:=false` if your bag already contains a pre-gated odometry topic.

Files added: [`src/nav_sat_gated_odom.cc`](src/nav_sat_gated_odom.cc).  
Files touched: [`CMakeLists.txt`](CMakeLists.txt), [`launch/localization_with_tf.launch.py`](launch/localization_with_tf.launch.py).

## 4. GLIM++ map bridge

Upstream DLIO loads a single PCD via `localization/map_path`. GLIM++ dumps **per-submap** subdirectories (`<dump_path>/NNNNNN/`), each containing the submap's local point cloud + `T_world_origin` metadata. The upstream workflow assumes the operator manually merges them via `ros2 run glim_ros offline_viewer` (GUI), exports a PLY, and converts with `convert_ply_to_pcd.py` — a three-step process with a GUI in the middle.

GICP++ adds [`scripts/merge_glim_submaps.py`](scripts/merge_glim_submaps.py) (~360 lines of Python with numpy + open3d) to close that gap. It:

1. Walks `<dump_dir>/NNNNNN/` submap directories in numeric order.
2. Reads `data.txt` and pulls out `T_world_origin` (Eigen-formatted 4×4 double).
3. Reads `points_compact.bin` with **auto-detected layout** — tries 32-byte `Vector4d`, 16-byte `Vector4f`, 24-byte `Vector3d`, 12-byte `Vector3f`, picks the one whose size divides cleanly into the file AND whose decoded points pass sanity checks.
4. Applies `T_world_origin` to bring each submap into the map frame, then concatenates.
5. Applies race-mode filters in order (each optional via CLI flag):
    - **Z-clip** (`--z-min` / `--z-max`) — drop sky reflections + below-ground returns.
    - **Centerline corridor** (`--track-csv` / `--mask-width`) — for race tracks, keeps only points within N m of the racing line. Uses `scipy.cKDTree` if installed; brute-force fallback otherwise.
    - **Statistical outlier removal** (`--outlier-k` / `--outlier-std`) — kills dust / rain phantoms.
    - **Voxel downsample** (`--voxel-res`) — uniform spatial density.
6. Writes binary PCD by default.
7. With `--copy-utm`, also mirrors `T_world_utm.txt` next to the output for easy `localization/utm_transform_path` pickup.

The legacy [`scripts/convert_ply_to_pcd.py`](scripts/convert_ply_to_pcd.py) is still shipped for operators who prefer the offline_viewer route or whose `gtsam_points` build uses an unsupported binary layout.

Files added: [`scripts/merge_glim_submaps.py`](scripts/merge_glim_submaps.py).

## 5. GICP warm-start

Upstream nano_gicp builds the target kd-tree at `setInputTarget()` and computes per-target covariances at `calculateTargetCovariances()` — both at constructor time. By the time the first scan arrives, the target side is ready. But the **first real `gicp.align()`** still pays ~5–20 ms of first-touch overhead: OpenMP thread-pool spin-up, Eigen kernel JIT warm-up, source-side kd-tree allocation, page-faults through the just-loaded map.

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

Files touched: [`src/localization.cc`](src/localization.cc) (lines ~389–421).

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

- **Zero publishers**: `nav_sat_gated_odom` isn't running OR `gt_odom_topic` arg points at the wrong topic.
- **Publishers exist, no messages**: `/gps/fix` has never reached STATUS_GBAS_FIX since startup (RTK convergence, NTRIP outage, blocked sky view).

Healthy state prints a single confirmation INFO line.

**(b) IMU never arrived.** The upstream-inherited check suggests `/imu/data` (the Atlas Duo default) in its typo example.

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

A small set of upstream additions that don't apply on the Hitch dome were removed to keep the code base tight:

- **`SensorType::LUMINAR`** enum value and its per-sensor branches were
  removed in the original retarget — and then RESTORED by the 2026-07 P1–P5
  improvements (§14), whose deskew/merge code paths reference them. The enum
  is now `{ OUSTER, VELODYNE, SEYOND, HESAI, LIVOX, LUMINAR, UNKNOWN }` and
  `sensor_type: "luminar"` parses (uint64 epoch-ns per-point timestamps);
  the value is unused on the dome (Robin W uses numeric FLOAT64 seconds via
  the explicit `seyond` branch).
- **`is_luminar_` dead member** in `localization.h` (declared but never used) — still removed.
- **`luminar_uint64` parameter** on `shiftCloudTimestamps` — removed in the
  retarget, re-introduced with §14 (it selects the absolute-epoch no-shift
  path).
- **Earlier-deployment comment strings and topic name hints** throughout the code (`/gps_na/imu`, `novatel_a`, `gps_bottom`, `imu_bottom`, etc.).
- **VECTR copyright headers** are RETAINED on files that originated there — that's legal attribution, not project text.

## 12. What was NOT changed

Important — so adopters can rely on the existing DLIO body of work and literature:

- The nano_gicp algorithm itself (kd-tree, correspondence search, Levenberg-Marquardt) — unchanged.
- The geometric observer math (`updateState`, `propagateState`, error formulation) — unchanged structurally; only Kp/Kq scale at the application step has been added.
- The IMU integration (gravity compensation, bias estimation, deskewing) — unchanged.
- The Init-Phase state machine (`WAITING` / `RTK_CALIBRATING` / `STATIONARY_CALIBRATING` / `DONE`) — unchanged structurally; only the motion-variance gate was added inside `STATIONARY_CALIBRATING`.
- nanoflann + nano_gicp vendored code — unchanged.
- License texts — every upstream LICENSE / NOTICE preserved verbatim.
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
| [`src/localization.cc`](src/localization.cc) | MODIFIED | Sensor scrub + warm-start + yaw-rate attenuation + motion gate + gt_odom health + §14 P1–P5 improvements (incl. LUMINAR branches) |
| [`include/dlio/dlio.h`](include/dlio/dlio.h) | MODIFIED | SensorType enum: explicit `SEYOND` absolute-seconds path; LUMINAR retained for §14 |
| [`include/gicp_localization/localization.h`](include/gicp_localization/localization.h) | MODIFIED | New member fields for yaw-rate attenuation + motion gate + gt_odom timer |
| [`scripts/merge_glim_submaps.py`](scripts/merge_glim_submaps.py) | NEW | GLIM++ submap → single PCD bridge (§4) |
| [`scripts/convert_ply_to_pcd.py`](scripts/convert_ply_to_pcd.py) | UNCHANGED | Legacy single-file PLY→PCD; still shipped for fallback |
| [`scripts/visualize_lidar_topic.py`](scripts/visualize_lidar_topic.py) | MODIFIED | Defaults retargeted to Robin W |
| [`scripts/live_gps_localization.py`](scripts/live_gps_localization.py) | MODIFIED | Defaults retargeted to Atlas Duo |
| `scripts/debug_pose_inspector.py`, `scripts/plot_*.py`, `scripts/profile_*.py` | UNCHANGED | Generic diagnostic / plotting; not vehicle-specific |
| `include/nano_gicp/*`, `src/nano_gicp/*` | UNCHANGED | Vendored nano_gicp / nanoflann |
| [`CMakeLists.txt`](CMakeLists.txt) | MODIFIED | Add `nav_sat_gated_odom` executable + install rule |
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
  and the **turnkey scorecard** `scripts/analyze_scan_debug_log.py`
  (acceptance/streaks, gicp_ms percentiles, fitness floor + suggested ratio
  thresholds, gt_err yaw-rate buckets, concat coverage).
- **URDF-first extrinsics** (`urdf_transforms.hpp`, libxml2 dep) and the
  single-IMU allowlist guard (Hitch default `/imu/data`; frame-match check
  left OFF until the Atlas Duo driver's `header.frame_id` is verified).
- **Evidence-first defaults:** `debug/enable_pub` and `verbose_scan_log` are
  ON (and the `verbose:false` WARN clamp now spares the SCAN DEBUG lines).

**Robin W notes.** `localization/sensor_type: "seyond"` validates
`timestamp/FLOAT64/count=1` and interprets it as numeric Unix seconds. The
deskewer uses those absolute capture times directly. Aux scans merged by
`lidar_concat` remain on the shared PTP axis and are not rebased by the
inter-header delta (`lidar_concat` remains disabled by default here until
3x Robin W sweep alignment is validated; the strict-merge guard and per-frame
evidence are ready when it is enabled).

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

Original Direct LiDAR-Inertial Odometry (DLIO) and `nano_gicp`:
- **Kenny J. Chen**, **Ryan Nemiroff**, **Brett T. Lopez** — UCLA Verifiable & Control-Theoretic Robotics (VECTR) Lab.
- Upstream repository: <https://github.com/vectr-ucla/direct_lidar_inertial_odometry>.

VECTR copyright notices are retained on every source / header file that originated upstream as legal attribution. The fork-specific changes in this folder are licensed under the same terms as the original work.

If you publish work that uses this localizer, please also cite the original DLIO paper:

> Chen, K., Nemiroff, R., & Lopez, B. T. *"Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction."* IEEE International Conference on Robotics and Automation (ICRA), 2023.

See [LICENSE](../LICENSE) at the repository root. The fork preserves upstream's MIT-style notices; nano_gicp and nanoflann carry their own permissive licenses inside the respective subdirectories.
