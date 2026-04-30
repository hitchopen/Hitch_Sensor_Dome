# Moving-Start Initialization — using the Atlas Duo INS as ground truth

## The problem upstream GLIM has

GLIM's upstream `NaiveInitialStateEstimation` (and its `LooseInitialStateEstimation` cousin) derives the world-frame orientation from the accelerometer mean over the first ~1 s of IMU data. Mathematically this finds the gravity direction in the IMU's body frame and rotates the world frame so gravity points in -Z. The implicit assumption is that the IMU is stationary during that window — only gravity is acting on the accelerometer.

That assumption breaks whenever a recording starts with the vehicle already in motion. Common cases:

- Mid-session restarts of `sensor_recorder.py` while the car is driving.
- Replaying a bag whose stationary head was trimmed.
- Starting on a banked surface or during a curving entry to a track.

The integrated linear acceleration leaks into the gravity estimate, producing a tilted world frame (`T_world_imu` rotated wrong by a couple of degrees in pitch/roll). Downstream:

- IMU preintegration accumulates against the wrong "up" — bias grows.
- The optimizer compensates with translation drift, especially in z.
- Loop closure between lap 1 and lap 2 (see [`multi_lap_loop_closure.md`](multi_lap_loop_closure.md)) becomes harder because the second lap's submaps start at the wrong altitude.
- Visual symptom: the `imu_link` trajectory drifts upward into the sky over multiple laps.

## What this fork changes

The Hitch Sensor Dome fork **removes the gravity-from-accelerometer pathway entirely** and requires an external INS pose to be injected at startup. The Atlas Duo (Point One Nav) already produces a fused, gravity-resolved orientation as part of its INS output, so we use that directly.

The change is implemented in C++ on both sides of the SLAM-core / ROS-wrapper boundary:

| Layer | File | Change |
|-------|------|--------|
| SLAM core | `glim/src/glim/odometry/initial_state_estimation.cpp` | `NaiveInitialStateEstimation::initial_pose()` returns `nullptr` until `force_init==true`. The `acc_dir` → `T_world_imu` derivation is gone. `insert_imu()` no longer accumulates `sum_acc`. |
| SLAM core | `glim/src/glim/odometry/odometry_estimation_imu.cpp` | Constructor always instantiates `NaiveInitialStateEstimation`. The `LooseInitialStateEstimation` branch and the JSON-driven `estimate_init_state` branching are gone. New public method `OdometryEstimationIMU::set_init_state(T, v)` forwards to the Naive instance via `dynamic_cast`. |
| SLAM core | `glim/include/glim/odometry/odometry_estimation_base.hpp` | New virtual `set_init_state(T, v)` on `OdometryEstimationBase` (default no-op) so the .so boundary is type-safe. |
| SLAM core | `glim/include/glim/odometry/async_odometry_estimation.hpp` and the `.cpp` | New `AsyncOdometryEstimation::set_init_state(T, v)` queues the values onto a mutex-protected slot; the worker thread drains it at the top of `run()` so no races against `insert_imu` / `insert_frame`. |
| ROS wrapper | `glim_ros2/include/glim_ros/glim_ros.hpp` and `.cpp` | Two new subscriptions: `geometry_msgs/PoseStamped` on `ins_pose_topic` (default `/pose`) and `nav_msgs/Odometry` on `ins_odom_topic` (default empty). The first valid message on either calls `odometry_estimation->set_init_state(T, v)`, then both subscriptions are released. |

The data flow on a normal session start:

```
  Atlas Duo (FusionEngine driver)
            │ /pose @ 10 Hz
            ▼
  GlimROS::ins_pose_callback     (one-shot)
            │ T_world_imu, v_world_imu
            ▼
  AsyncOdometryEstimation::set_init_state    (queued; thread-safe)
            │ drained on worker thread
            ▼
  OdometryEstimationIMU::set_init_state
            │ dynamic_cast<NaiveInitialStateEstimation*>
            ▼
  NaiveInitialStateEstimation::set_init_state
            │ force_init = true
            ▼
  next call to initial_pose() returns the EstimationFrame
  with T_world_imu = (Atlas Duo orientation), v_world_imu = (its velocity)
```

While `force_init==false`, `NaiveInitialStateEstimation::initial_pose()` returns `nullptr` and `OdometryEstimationIMU::insert_frame()` (which calls `init_estimation->initial_pose()` to seed the optimizer) silently buffers — IMU and LiDAR data are still consumed but the optimizer doesn't kick off. The window is short in practice: at 10 Hz `/pose` and a typical bag start, the first valid INS pose arrives within ~100 ms.

## RTK-fixed gating — verifying the INS pose is *reliable*

A naive "accept the first pose" implementation would happily latch onto an INS that's still cold-starting, dead-reckoning on IMU only, or in RTK-float mode (sub-meter, but not centimetre-grade). Since GLIM's *entire* map is anchored to this single pose, accepting a bad one means re-recording the session. The wrapper enforces a three-stage gate before calling `set_init_state`:

| Stage | Check | Default threshold | Why |
|-------|-------|-------------------|-----|
| **(1) Fix status** | `NavSatFix.status.status ≥ STATUS_GBAS_FIX` | `ins_require_rtk_fixed = true` | `STATUS_GBAS_FIX` (= 2) is the RTK-class indicator in REP-145. `STATUS_FIX` (= 0, single-point GNSS) and `STATUS_SBAS_FIX` (= 1) are explicitly rejected by default. |
| **(2) Covariance** | `position_covariance` diagonal max stddev ≤ `ins_max_position_stddev` | `0.10 m` | RTK-fixed reliably hits 1–5 cm. RTK-float typically sits at 10–50 cm. The 10 cm threshold catches the RTK→fixed transition cleanly. |
| **(3) Stability**  | Last `ins_min_pose_window_samples` PoseStamped/Odometry messages are mutually consistent: pairwise translation drift < `ins_max_pose_jitter_trans`, pairwise `|q1·q2| > ins_min_quat_dot` | 10 samples · 5 cm · 0.999 (≈2.5°) | Catches the case where the GNSS reports RTK-fixed but the INS hasn't yet aligned its IMU mechanization to that fix — orientation jitters before locking. |

All three must be true on the *same* incoming pose for it to be accepted.

While any of the three is failing, the wrapper's `ins_init_timeout_tick` timer prints a bold RED warning every 10 s naming the most-recent rejection reason and listing remediation steps. After `ins_init_timeout_s` (default 60 s), the warning escalates to "TIMEOUT" but **GLIM never auto-aborts** — the operator decides whether to wait, relax thresholds, or kill the process.

### Sample warning output (RTK-float, INS still settling)

```
============================================================
  ⚠  GLIM is waiting for a reliable INS pose. ⚠
============================================================
  Elapsed:  42.0 s  (timeout 60.0 s)
  Last NavSatFix: status=GBAS_FIX (RTK-class), pos σ=0.183 m,
                  lat=37.872103, lon=-122.258431, alt=68.92
  Last reject reason: position covariance stddev=0.183 m > 0.100 m (RTK not yet fixed)

  Why GLIM is blocked:
    The Hitch fork uses the Atlas Duo INS pose as the
    ground-truth orientation for SLAM initialization. To
    avoid anchoring the entire map to a bad pose, the
    wrapper enforces RTK-fixed status and covariance gating.

  What you can do:
    1. Wait — RTK convergence typically takes 30–120 s outdoors.
    2. Check sky visibility — RTK needs unobstructed L1+L5.
    3. Verify NTRIP corrections are flowing (check Atlas web UI).
    4. Relax the gate (degraded init) by re-launching with:
         ins_require_rtk_fixed:=false
         ins_max_position_stddev:=0.5    # 0.5 m for SBAS-class
       Note: relaxing INVALIDATES the fix to the moving-start
       pathology; the map will not be reliably gravity-aligned.
```

### Successful gate transition

```
[glim] Hitch fork: subscribed to NavSatFix topic '/gps/fix' (gate signal)
[glim] Hitch fork: subscribed to INS PoseStamped topic '/pose'
[glim] Hitch fork: RTK gate — require_rtk_fixed=true, max_pos_stddev=0.10 m,
                   window=10 samples, max_jitter=0.050 m, min_quat_dot=0.9990,
                   timeout=60 s
[glim] Hitch fork: INS init pose ACCEPTED — fix=GBAS_FIX (RTK-class),
                   pos σ=0.018 m, translation=[3.214, -1.072, 68.940],
                   qw=0.9988
[odom] initial IMU state pinned from external INS source: ...
```

### Beyond init — the RTK-gated GNSS factor bridge

The init gate above is one-shot: it pins the SLAM world frame at session start and then releases. But the project also wants RTK-quality GNSS to **continue feeding the global graph as soft factors** during the rest of the session — that's what prevents multi-lap z-drift (see [`multi_lap_loop_closure.md`](multi_lap_loop_closure.md)).

The bridge in `GlimROS::try_publish_gnss_factor` reuses the same NavSatFix subscription (kept alive post-init) and applies a per-message RTK gate to every incoming `/pose` (or `/odom`). When the gate passes, the pose is republished on `gnss_factor_topic` (default `/gnss/pose_rtk_only`) as `geometry_msgs/PoseWithCovarianceStamped`, which `libgnss_global.so` consumes natively. When the gate fails (RTK-float, no-fix, tunnel), the bridge silently drops the pose; `gnss_global` sees a quiet topic and adds no factor for that interval. The optimizer's LiDAR cost carries the trajectory through the gap, and factors resume automatically when RTK locks again.

The bridge is configured with a parallel set of parameters that default to the same thresholds as the init gate, but can be tuned independently:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `gnss_factor_topic` | `/gnss/pose_rtk_only` | Where the bridge publishes; `""` disables the bridge |
| `gnss_factor_require_rtk_fixed` | `true` | If true, only republish during RTK-class fixes |
| `gnss_factor_max_position_stddev` | `0.10` m | Reject poses whose NavSatFix covariance exceeds this |

A periodic log line every 10 s reports `N published, M rejected` so the operator can see whether the bridge is doing useful work mid-session — counts dropping to zero for an extended period typically means RTK degraded (tunnel, urban canyon) and the operator can read NavSatFix status to confirm.

This is the *only* GNSS-aware mechanism in GLIM that runs throughout the session. All other GLIM extensions (gravity_estimator, flat_earther, deskewing, imu_validator, imu_prediction, velocity_suppressor, orb_slam) consume IMU and/or LiDAR only — they have no built-in GNSS dependency, and consequently no RTK gating to consider.

## Operating without RTK

If your setup has no RTK base station, you have two reasonable choices:

1. **Float-grade init** — re-launch with `ins_max_position_stddev:=0.5`, keep `ins_require_rtk_fixed:=true`. Gates on RTK-float (typically 10–50 cm σ) instead of fixed. Map metric scale is still trustworthy, but absolute world-frame altitude has 0.5 m of slop.

2. **SBAS / single-point init** — set `ins_require_rtk_fixed:=false` and `ins_max_position_stddev:=2.0`. Allows any GNSS-aware INS pose. The fix to the moving-start pathology is preserved (gravity is still resolved by the INS), but the map's world-frame anchor is loose at the metre scale.

Both are explicit, logged, and configurable per-run. The default (`true / 0.10`) targets the Hitch Sensor Dome's RTK reference setup — see [`../../PTP_sync/README.md`](../../PTP_sync/README.md) §3.1 for the full GNSS configuration.

## What the user has to do

Nothing, at the launch level. The default launch invocation:

```bash
ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py
```

is already wired to subscribe to `/pose` and `/gps/fix` with the strict-RTK gate enabled. If your INS pose is on a different topic:

```bash
ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py ins_pose_topic:=/atlas/pose
```

If your INS publishes `nav_msgs/Odometry` instead of `PoseStamped` (which lets GLIM also consume linear velocity, not just orientation):

```bash
ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py \
    ins_pose_topic:=  ins_odom_topic:=/odom
```

For offline rosbag replay, the bag must contain `/pose` (or whichever topic you choose). Recordings made by `recording/sensor_recorder.py` already include it — see [`recording/sensor_config.yaml`](../../recording/sensor_config.yaml) → `point_one_nav.topics.pose`.

## What the obsolete Python helper used to do

A previous design (visible in this repo's git history but no longer in the working tree) had a Python helper that:

1. Subscribed to `/pose` for ~2 s.
2. Wrote `init_T_world_imu` / `init_v_world_imu` into a runtime copy of `config_odometry_gpu.json`.
3. Pointed `glim_rosnode` at the runtime copy.

That helper has been removed. The C++ subscription replaces it because:

- The C++ runs inside the same process as GLIM, so there's no Python ↔ C++ launch-ordering race.
- Replays of MCAP bags work the same way as live runs (the bag carries `/pose`, the C++ subscription consumes it).
- The launch file is shorter and has fewer moving parts.
- A pose-quality gate (RTK-fixed status + covariance + multi-sample stability) is enforced inline before the pose is forwarded — see "RTK-fixed gating" above.

## Diagnostics — `check_init_stationarity.py`

`scripts/check_init_stationarity.py` is preserved as a **diagnostic** tool. It scans the first 3 s of `/imu/data` (live or from a bag) and prints a bold RED warning if the vehicle was non-stationary during that window. With the C++-side fix above, GLIM no longer fails on a moving-start bag — but the warning is still informative because it tells you whether you should expect the optimizer to take longer than usual to settle (more bias to estimate from a non-zero starting state).

```bash
# Live, while the recording rig is running:
python3 GLIM_plusplus/scripts/check_init_stationarity.py --live

# Against a recorded bag:
python3 GLIM_plusplus/scripts/check_init_stationarity.py \
    --bag recording/data/session_20260429_0942/rosbag2

# In CI / launch-gating mode (exit 2 on non-stationary):
python3 GLIM_plusplus/scripts/check_init_stationarity.py --bag <path> --strict
```

## What changed numerically — gravity calibration vs INS init

| Quantity | Upstream NAIVE/LOOSE | This fork |
|----------|----------------------|-----------|
| Where T_world_imu's rotation comes from | accelerometer mean over first 1–3 s | Atlas Duo INS quaternion (already gravity-resolved) |
| Where v_world_imu comes from | always `0` | Atlas Duo INS twist (or `0` if PoseStamped only) |
| Behaviour if the vehicle is stationary | works | works (INS has settled to a stable orientation) |
| Behaviour if the vehicle is moving | tilted world frame, downstream z-drift | correct world frame, no init artifact |
| Cold-start latency | ~1–3 s of stationary IMU | ~100 ms of INS messages (10 Hz `/pose`) |
| Required dependency | clean stationary IMU window | a working INS publishing `/pose` |

The fork's tradeoff: GLIM now requires an INS to start. If you're running on hardware *without* an INS (e.g. a pure-LiDAR box), you'd need to revert the four C++ changes listed in the table above. For the Hitch Sensor Dome — which always has an Atlas Duo — this is the correct tradeoff: the failure mode that the upstream code was sensitive to never happens here.

## Related documents

- [`multi_lap_loop_closure.md`](multi_lap_loop_closure.md) — the downstream symptom (z-drift on lap 2+) that motivated this fix, and the GNSS / VGICP tuning that complements it.
- [`../README.md`](../README.md) §"Tuning provenance" — the broader history of GLIM tuning in this fork.
- [`../../recording/sensor_config.yaml`](../../recording/sensor_config.yaml) — confirms the `/pose` topic is published by `fusion_engine_driver`.
