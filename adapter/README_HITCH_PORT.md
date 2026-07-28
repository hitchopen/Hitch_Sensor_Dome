# adapter — status on the Hitch Sensor Dome

Point One Atlas normalization boundary. Converts raw Atlas FusionEngine WGS84
pose/IMU into local-ENU `/gps_p1/*` streams (`filtered_odom`,
`filtered_odom_rtk_fixed`, `fix`, `imu`, optional `/gnss*`), with
P1-clock-consistent monotonic stamps and an optional P1 PCAP IMU replay node.
It also publishes transient-local `/gps_p1/local_enu_origin` metadata so
GICP++ can compare the live adapter datum with the GLIM map manifest.
Frames default to the dome TF (`imu_link`, the Atlas Duo Center of Navigation
— see `config/sensor_dome_tf.yaml` at the repo root).

## Production GLIM++ GNSS boundary

The strict GLIM++ mapping profile consumes both
`/gps_p1/filtered_odom_rtk_fixed` and `/gps_p1/fix` from this adapter. They
share the same timestamp and FusionEngine sample. The odometry stream exists
only when the native pose reports `solution_type == kRtkFixed` and passes
finite covariance limits; `/gps_p1/fix` supplies the synchronized WGS84
position, freshness, and covariance companion. This is the repository path that reliably
distinguishes Fixed from Float.

The legacy GLIM compatibility path remains `/pose` plus `/gps/fix`. It cannot
distinguish Float from Fixed through REP-145 status and is not the production
adapter path. GICP++ now consumes `/gps_p1/filtered_odom_rtk_fixed` directly;
its `/odom_rtk_only` helper output is legacy compatibility only.

The adapter is also useful when:

- replaying a raw Atlas PCAP without the live driver (`p1_imu_pcap_replay_node.py`),
- you want the single-datum **local-ENU contract** and monotonic-stamp
  guarantees at the ingestion edge,
- you need `twist.angular` populated on the INS odometry: the Atlas Pose
  message carries no body rates, and GICP_plusplus's GT-snap recovery uses
  the odometry's angular twist for rate continuity (P2#2 fix — this adapter
  fills it from the latest gyro, freshness-gated, with rate covariance).

Set `local_enu_origin` to the deployment datum; the adapter now refuses to
start when neither origin source is configured. The retired
`39.58227391,-86.74232215,260.4` placeholder is also rejected unless
`allow_legacy_local_enu_origin:=true` explicitly acknowledges that exact
site. GICP++ may use the adapter
streams directly or retain its separate compatibility gate. In either case,
make sure **whatever publishes the GT odometry populates
`twist.twist.angular`** — otherwise every GT snap resets the angular-rate
state (see GICP_plusplus README, "GT-driven pose recovery").

Not Robin W-specific: this package touches no LiDAR data.

## Upstream sync — merge record, 2026-07-27

Re-synced against [`augcog/DLIO_plusplus`](https://github.com/augcog/DLIO_plusplus)
branch `ucb-roar` (fork point `edfede3` → tip `69eb574`) via 3-way merge.
Upstream's delta in this package was **pure hardening** of the Point One Atlas
normalization node — no change of sensor vendor, topic contract, or output
semantics — so all of it was taken. Upstream PR #15 did not touch this package.

### Hitch customizations preserved

| Customization | Where |
|---|---|
| `imu_frame_id` / `body_frame_id` default to `imu_link` (Atlas Duo Center of Navigation, per `config/sensor_dome_tf.yaml`) instead of upstream's `gps_antenna_top` | `src/adapter_node.cpp`, `config/adapter.yaml` |
| `adapter_utils` installed to `lib/` as a library (ARCHIVE/LIBRARY/RUNTIME) rather than `lib/adapter/` — required for the target to link in this workspace | `CMakeLists.txt` |
| README reframed for the dome; GLIM++ and GICP++ production paths consume the adapter's `/gps_p1/*` outputs directly | `README.md`, this file |
| `local_enu_origin` documented as an earlier-deployment **placeholder** that must be replaced with the deployment datum | `config/adapter.yaml`, `README.md` |

The only merge conflict was the README's opening paragraph, where upstream
re-described a `prep_bag.py` pipeline that does not exist on this platform. The
Hitch framing was kept and upstream's genuinely new sections (run summary /
audit counters, origin resolution rules) were taken.

### What came in from upstream

**P1 → ROS clock mapping** (`adapter_utils.{hpp,cpp}`)

- **Slew-limited applied offset.** Online bin refinement used to step the
  output stamp domain mid-stream, producing anomalous `dt` for consumers. The
  applied offset now moves toward its target at a bounded rate.
- **Forced re-anchor on epoch change.** `reset()` clears bins, slew, and
  streaks, so a new epoch never inherits a stale minimum lag.
- **Forward-glitch streak counter.** A *persistent* forward jump (device epoch
  change) now resets the mapper after N rejects instead of freezing it forever.

**Pose path** (`adapter_node.cpp`)

- **Forward-spike quarantine**, placed *before* any stateful update
  (`last_p1_time_`, `addPosePair`, `toRos`). A single in-range forward glitch
  would otherwise become the clock bin's minimum lag and publish every later
  normal sample seconds early. The bound is `pose_max_forward_jump_sec`.
- **Startup parameter validation.** Non-finite or nonpositive timing
  parameters now throw at startup instead of silently degrading retiming — a
  zero flush timeout stranded the arrival-retime queue, and a nonpositive
  nominal period broke synthesized spacing.
- **`imu_arrival_retime_lookahead` clamp.** A negative value wrapped to
  `SIZE_MAX` and the arrival-retime queue never published.
- **`local_enu_origin` both/neither fix.** The inline default is now empty, so
  setting *only* `local_enu_origin_ttl_path` no longer trips the both-set guard
  with a misleading error.

**Run audit.** Four drop counters (`pose_dropped_invalid`,
`imu_dropped_invalid_stamp`, `imu_sidecar_miss_drop`,
`imu_dropped_clock_not_ready`) plus `p1_clock_ready`, `p1_clock_drift_ms`, and
`p1_clock_reset_count` are always reported in the run-summary line, so a run
report can **prove** zero loss and detect a mid-run clock re-anchor rather than
infer it from matching in/out totals. Together with GLIM++'s
`gnss_global summary:` line this closes the RTK timing audit chain from PCAP to
map factors. All four are expected to be `0` on a healthy run.

### Post-merge additions on this platform

- `config/adapter.yaml` now carries `pose_max_forward_jump_sec: 5.0`
  explicitly. The node declares and startup-validates it, but neither
  upstream's YAML nor its launch file exposed it, which left the quarantine
  bound undiscoverable. This is an upstream gap rather than a merge regression,
  and is worth reporting upstream.
- `README.md`'s "Startup validation" list named four validated parameters where
  the code validates five; `pose_max_forward_jump_sec` was added.

### Verification performed

- Upstream hunk coverage: `adapter_utils.{hpp,cpp}`,
  `launch/adapter.launch.py`, and `scripts/p1_imu_pcap_replay_node.py` are
  byte-identical to upstream tip. `src/adapter_node.cpp` differs only by the
  two Hitch `imu_link` defaults and their explanatory comment. No upstream hunk
  was dropped.
- Syntax-only compile of both `.cpp` files against stubbed ROS headers: clean
  under `-Wall -Wextra -Wformat=2`.
- All 24 declared node parameters are unique, and every key in
  `config/adapter.yaml` plus every launch-injected parameter resolves to a
  declared parameter — no silently-ignored typos.
- `local_enu_origin` both/neither logic re-checked end to end: a normal
  `ros2 launch` with the checked-in YAML resolves to exactly one source and
  does not trip the startup guard.

Note that no build or runtime test was executed for this merge; the checks
above are static.

### Known upstream design gap (unchanged by this merge)

On the **non-launch** path (`ros2 run adapter adapter --params-file
config/adapter.yaml -p local_enu_origin_ttl_path:=…`) the YAML's non-empty
inline origin still trips the both-set startup error, because only the launch
file blanks it. Use the launch file, or clear `local_enu_origin` in the YAML
when passing a TTL path.

## Build note

`adapter/` lives at the repo root as a sibling of `GLIM_plusplus/` and
`GICP_plusplus/`, so a normal workspace build discovers it like any other
package:

```bash
colcon build --symlink-install
```

(Do not nest it inside another package directory: colcon does not descend
into a recognized package to find nested ones.)
