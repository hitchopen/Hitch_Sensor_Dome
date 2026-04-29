# Multi-Lap Loop Closure — Debugging Z-Drift

A focused guide for the failure mode where GLIM's estimated trajectory tilts upward into the sky during the second (or later) lap of repeated data over the same track.

## Symptom

When mapping multi-lap data:

- Lap 1 looks reasonable.
- The second time the trajectory passes through a previously-mapped region, the estimated `imu_link` z-coordinate climbs steadily.
- The map develops "ghost" floor surfaces: lap-2 ground returns sit several metres above lap-1 ground returns.
- `T_world_utm.txt` is written but the recovered SE(3) doesn't pin the trajectory back to GNSS altitude visibly.

## Root cause

GLIM's `config_global_mapping_gpu` module performs **implicit** loop closure: rather than detecting and validating loops, it adds a VGICP factor between *every pair of submaps within `max_implicit_loop_distance` whose estimated overlap exceeds `min_implicit_loop_overlap`*. This is computationally lighter than explicit loop detection but introduces a chicken-and-egg dependency:

1. Lap 1 accumulates a small amount of z-drift. LiDAR scan-matching is structurally weaker in z (the only z-anchor is ground returns; vertical features are sparser than horizontal). IMU bias, particularly on the accelerometer z-axis, contributes too.
2. Lap 2 odometry initializes from end-of-lap-1, inheriting the drift.
3. When global mapping evaluates a candidate factor between lap-1 submap *j* and lap-2 submap *k*, the **initial pose for VGICP comes from the current odometry estimate** — the two submaps are already several metres apart in z.
4. VGICP iterates from that initial pose. Its convergence basin is roughly 1–2 voxel widths. With residual z-offset exceeding `submap_voxel_resolution_max` × ~2, VGICP either:
    - Fails to find correspondences (estimated overlap drops below `min_implicit_loop_overlap` → no factor is created), or
    - Locks onto false correspondences (a factor is created but it pulls in the wrong direction).
5. Without a correct closure factor, lap 2's z-drift is uncorrected and grows.

The GNSS prior factor *should* fix this, but with `prior_inf_scale = [1e4, 1e4, 1e4]` and `min_baseline = 1.0` m, the GNSS contribution is dwarfed by the LiDAR factor count. LiDAR drift wins by sheer factor density.

## The fix in this fork

Three concurrent changes break the cycle (already applied to this fork's configs):

| Change | Where | Effect |
|--------|-------|--------|
| `prior_inf_scale[2]: 1e4 → 5e4` | `glim_ext/config/config_gnss_global.json` | z is now 5× more strongly anchored than horizontal. Prevents lap-1 z-drift from accumulating in the first place. |
| `min_baseline: 1.0 → 0.5` | `glim_ext/config/config_gnss_global.json` | 2× the GNSS factor density along the trajectory. Each individual factor still has the same per-measurement weight, but more of them carry equal weight in the optimization. |
| `submap_voxel_resolution_max: 1.0 → 2.0` | `glim/config/config_global_mapping_gpu.json` | Top of the multi-resolution VGICP ladder doubles. Submap-to-submap registration retains a usable convergence basin when initial poses are off by ~1 m in z. |
| `max_implicit_loop_distance: 100 → 200` | `glim/config/config_global_mapping_gpu.json` | Factor candidates are evaluated up to 200 m apart in the estimated frame, comfortably covering a single race-track lap. |
| `min_implicit_loop_overlap: 0.2 → 0.1` | `glim/config/config_global_mapping_gpu.json` | Submaps with even partial overlap (drifted in z) still create a factor, letting the optimizer pull them together. |

Layer 1 (stronger GNSS) is the primary fix — it stops the drift before VGICP has to clean it up. Layers 2 and 3 (wider voxel ceiling, looser proximity thresholds) are insurance for when residual drift survives the GNSS pull.

## Verification — what to look for during the next run

Run the launch helper and `tee` the output so you can grep the logs after:

```bash
ros2 launch GLIM/launch/hitch_sensor_dome.launch.py 2>&1 | tee glim_run.log
```

### 1. GNSS factors are actually being inserted

Watch for these messages in the log:

```
[gnss_global] initializing GNSS global constraints
[gnss_global] T_world_utm=…
[gnss_global] insert N GNSS prior factors
```

`N` should grow steadily — at the new `min_baseline = 0.5` m and a typical lap length of ~1 km, you should see N rise into the thousands by the end of one lap. If N stays low, GNSS is being filtered out (often: covariance reported by the driver is too pessimistic; the factor is rejected on a sanity check).

```bash
grep "insert.*GNSS" glim_run.log | tail -20
```

### 2. Implicit loop-closure factors fire when lap 2 begins

`config_global_mapping_gpu`'s factor creation isn't logged by default. The proxies are:

- The GLIM viewer should show pose-graph edges between non-consecutive submaps once lap 2 starts overlapping lap 1. In `librviz_viewer.so`, look at the pose-graph topic (typically `/glim_ros/factors`).
- If you don't see lap-1↔lap-2 edges appear, the implicit threshold is still being missed. Lower `min_implicit_loop_overlap` further (try 0.05) and/or raise `submap_voxel_resolution_max` (try 4.0).

### 3. T_world_utm written and stable

```bash
cat glim_maps/session_<ts>/T_world_utm.txt
```

The 4×4 matrix should be written once and remain stable across the run. If the run completes without writing this file, GNSS was never aligned — check `min_baseline` against the actual distance traveled before the first GNSS lock.

### 4. Trajectory altitude vs GNSS altitude

After the run, plot `traj_imu.txt` z-coordinate against the GNSS altitude over time. The two curves should overlay within a metre. If `traj_imu.txt` z drifts up while GNSS is steady, the GNSS prior is being underweighted — raise `prior_inf_scale[2]` further (try 1e5).

## If the fix isn't sufficient

Try in this order, only one change at a time so you can attribute the improvement:

1. **Raise `prior_inf_scale[2]` to `1e5`.** Use only if GNSS is RTK-fixed (sub-cm vertical). With float-only fix, a stronger weight on a noisy measurement causes horizontal jitter to "leak" into the trajectory.

2. **Drop `min_baseline` to `0.25`.** Useful for very fast vehicles where 0.5 m of motion only takes 5–10 ms.

3. **Enable `libflat_earther.so`** in `config_ros.json`'s `extension_modules` list. This adds a soft prior that the trajectory's z-axis stays near a flat-earth assumption. **Only correct for tracks that are actually flat** — using it on a track with elevation changes will fight the real terrain and produce worse maps. Configured in `glim_ext/config/config_flat_earther.json`.

4. **Switch global mapper to the pose-graph variant.** Edit `glim/config/config.json` and replace
    ```json
    "config_global_mapping": "config_global_mapping_gpu.json"
    ```
    with
    ```json
    "config_global_mapping": "config_global_mapping_pose_graph.json"
    ```
    The pose-graph mode does explicit candidate generation (`min_travel_dist`, `max_neighbor_dist`) followed by VGICP validation (`min_inliear_fraction`). This is more robust to drifted initialization at the cost of being slower. Tune `max_neighbor_dist` upward if first-lap drift was several metres.

5. **Inspect for IMU bias drift directly.** `libimu_validator.so` is enabled by default in this fork. Its output appears as `[imu_validator]` lines. If it reports growing acc-z bias estimates, the issue is upstream of GLIM — check the Atlas Duo's accelerometer factory calibration or whether the host clock vs PPS sync is solid (a sub-microsecond timing slip on the 200 Hz IMU stream would manifest as bias-like drift).

## Why this fork's defaults are now permissive rather than tight

Multi-lap mapping is a fundamentally harder regime than single-lap odometry. The default GLIM tunings on the upstream repository are calibrated for single-pass mapping where every submap is novel; loop closure is largely a post-hoc convenience. For repeated-trajectory data — race tracks, fleet vehicles operating on fixed routes, surveying a parking lot — loop closure must work on the second pass to be useful at all. The widened thresholds (200 m candidate distance, 10 % overlap, 2 m voxel ceiling) trade some compute for that robustness. The GNSS z-prior is what makes the loosened thresholds safe: false closures are caught by the global anchor before they corrupt the map.
