# adapter — status on the Hitch Sensor Dome

Point One Atlas normalization boundary. Converts raw Atlas FusionEngine WGS84
pose/IMU into local-ENU `/gps_p1/*` streams (`filtered_odom`,
`filtered_odom_rtk_fixed`, `fix`, `imu`, optional `/gnss*`), with
P1-clock-consistent monotonic stamps and an optional P1 PCAP IMU replay node.
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
adapter path. `/odom_rtk_only` belongs to GICP++ and is not a GLIM input.

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

## Upstream sync

Re-synced against upstream `ucb-roar` on 2026-07-27 (P1 clock-mapper
hardening, forward-spike quarantine, startup parameter validation, run-audit
counters). Dome frame defaults (`imu_link`) and the build fix are preserved.
See [MERGE_NOTES_2026-07-27.md](MERGE_NOTES_2026-07-27.md).

## Build note

`adapter/` lives at the repo root as a sibling of `GLIM_plusplus/` and
`GICP_plusplus/`, so a normal workspace build discovers it like any other
package:

```bash
colcon build --symlink-install
```

(Do not nest it inside another package directory: colcon does not descend
into a recognized package to find nested ones.)
