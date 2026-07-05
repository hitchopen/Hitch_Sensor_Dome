# adapter — status on the Hitch Sensor Dome

Point One Atlas normalization boundary. Converts raw Atlas FusionEngine WGS84
pose/IMU into local-ENU `/gps_p1/*` streams (`filtered_odom`,
`filtered_odom_rtk_fixed`, `imu`, optional `/gnss*`), with P1-clock-consistent
monotonic stamps and an optional P1 PCAP IMU replay node. Frames default to
the dome TF (`imu_link`, the Atlas Duo Center of Navigation — see
`config/sensor_dome_tf.yaml` at the repo root).

## OPTIONAL / ALTERNATIVE ingestion path

The dome pipeline ingests the Atlas Duo through its own driver chain
(`/imu/data`, `/odom_rtk_only`, `/gnss/pose_rtk_only` — see the repo root
README and `recording/`). This adapter is an **alternative ingestion path**,
valuable when:

- replaying a raw Atlas PCAP without the live driver (`p1_imu_pcap_replay_node.py`),
- you want the single-datum **local-ENU contract** and monotonic-stamp
  guarantees at the ingestion edge,
- you need `twist.angular` populated on the INS odometry: the Atlas Pose
  message carries no body rates, and GICP_plusplus's GT-snap recovery uses
  the odometry's angular twist for rate continuity (P2#2 fix — this adapter
  fills it from the latest gyro, freshness-gated, with rate covariance).

To use it, remap its outputs onto the dome conventions (or point
GICP_plusplus/GLIM_plusplus at `/gps_p1/*` directly) and set
`local_enu_origin` to the deployment datum. If you keep the existing driver
chain instead, make sure **whatever publishes the GT odometry populates
`twist.twist.angular`** — otherwise every GT snap resets the angular-rate
state (see GICP_plusplus README, "GT-driven pose recovery").

Not Robin W-specific: this package touches no LiDAR data.

## Build note

`adapter/` lives at the repo root as a sibling of `GLIM_plusplus/` and
`GICP_plusplus/`, so a normal workspace build discovers it like any other
package:

```bash
colcon build --symlink-install
```

(Do not nest it inside another package directory: colcon does not descend
into a recognized package to find nested ones.)
