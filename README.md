# Hitch Sensor Dome

A 3D-printable modular sensor dome for mounting a multi-sensor mapping rig on a vehicle roof via a suction cup camera mount.

## Sensors

- 3× Seyond Robin W LiDAR — 120° HFOV each, arranged at 120° intervals for full 360° surround coverage
- 1× Point-One Nav Atlas Duo INS — center-mounted, CoN at geometric origin
- 1× **Point One SP1** multi-frequency (L1/L2/L5) GNSS antenna — on ArduSimple survey stand, centered above CoN
- 4× e-con RouteCAM_P_CU25_CXLC_IP67 cameras — front stereo pair (110 mm baseline) + rear symmetric pair

> **GNSS antenna note.** The Atlas Duo's GNSS port supplies **3.3 V DC bias** for a powered LNA (per the [Point One Atlas User Guide](https://pointonenav.com/wp-content/uploads/2024/06/Atlas-User-Guide.pdf)). The [Point One SP1](https://store.pointonenav.com/products/sp1-high-precision-gnss-antenna) ships voltage-matched to this rail and is the recommended default. The ArduSimple "Magnetic Stand for Survey GNSS Antenna" used in the BOM has a **5/8"-11 UNC** thread (the surveying-pole standard); thread compatibility of each candidate antenna is summarized below.
>
> | Antenna | Bands | Thread fit on the ArduSimple stand |
> |---------|-------|-------------------------------------|
> | [Point One SP1](https://store.pointonenav.com/products/sp1-high-precision-gnss-antenna) | L1 / L2 / L5 | **Drop-in.** SP1 kit ships with a magnetic mount + 75 mm riser carrying a 5/8"-11 UNC thread. |
> | [Tallysman TW3972](https://www.tallysman.com/product/tw3972-triple-band-gnss-antenna-with-l-band/) | L1 / L2 / L5 + L-band | **Adapter required.** Native mount is through-hole / flat-bottom. Add a Calian / Tallysman [Pipe Mount Adapter PN 23-0065-0](https://www.calian.com/advanced-technologies/gnss_product/pipe-mount-adapter-screw-compression-pn-23-0065-0/) (or PN 23-0040-0 L-bracket) to expose a 5/8"-11 UNC thread. |
> | [Harxon HX-CSX601A](https://en.harxon.com/product/detail/99) | GPS/GLONASS/Galileo/BeiDou multi-band | **Verify on current datasheet.** Survey-grade, TNC-F. Public sources are inconsistent on whether the thread is 5/8"-11 UNC (the surveying standard) or 5/8"-12; confirm with Harxon's brochure for the unit you actually order before committing. |
>
> Verify each candidate's LNA voltage / current spec against 3.3 V before connecting, and add a DC block if the antenna is already powered from another source. Do **not** repurpose the GNSS pigtail off a combo LTE + GNSS antenna (Peplink, etc.) — even the better combo antennas are a meaningful regression from the SP1 in band coverage and noise figure, and the resulting RTK fix quality drops accordingly.

> ### ⭐ Dual GNSS antenna — strongly recommended
>
> A second SP1-class GNSS antenna mounted at a known offset from the first turns RTK-fixed into a **drift-free heading reference** (typically 0.1°–1° accurate, depending on baseline length). Without it, vehicle heading is integrated from the IMU gyroscope and drifts over time — the canonical multi-lap z-drift / yaw-drift failure modes that GLIM++ has to compensate for.
>
> > **⚠ IMPORTANT — the 3D reference design ships with only ONE GNSS antenna mount.** The SCAD model in [`3D files/`](3D%20files/) and the BOM in [`3D files/README.md`](3D%20files/README.md) cover a single magnetic survey stand centered above the Atlas Duo's CoN. The geometry of a second antenna stand is **deployment-specific** because the optimal baseline depends on the vehicle (a sedan roof, a race-car shell, and a survey truck cab all want different placements). Every real deployment is expected to add a secondary GNSS antenna — the 3D files are a starting point, not the final mechanical design. Mount the second antenna rigidly enough that the baseline does not flex during driving (rigid metal plate or 3D-printed extension bolted to the dome, not adhesive-mounted), measure the resulting `imu_link → gnss_antenna_secondary_link` offset to the nearest centimetre, and enter that offset in [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) per the steps below.
>
> **To enable dual-antenna in this project:**
>
> 1. Mount a second SP1 (or compatible multi-band antenna) at a fixed offset from the primary. **Recommended baseline 1.0–1.5 m** for vehicle-roof installations; minimum 0.5 m for usable accuracy.
> 2. Wire it into the Atlas Duo's secondary GNSS RF input (with an antenna splitter or the second port if your Atlas Duo unit exposes it). Configure the Atlas Duo's web UI for dual-antenna heading mode.
> 3. **Edit [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml)** to record the secondary antenna's actual mounting position. The relevant block looks like this:
>
>    ```yaml
>    # Excerpted from config/sensor_dome_tf.yaml — secondary antenna entry.
>    # Replace translation (0, 0, 0) with the measured offset, in metres,
>    # from imu_link (Atlas Duo CoN) to the secondary antenna phase center.
>    # Rotation stays identity — the antenna baseline is a translation.
>    - frame_id: imu_link
>      child_frame_id: gnss_antenna_secondary_link
>      translation:
>        x: 0.0      # ← measured fore/aft offset, +X forward, metres
>        y: 1.200    # ← measured left/right offset, +Y left, metres (example: 1.2 m laterally to the left)
>        z: 0.273    # ← measured up offset, +Z up, metres (typically matches the primary antenna's height)
>      rotation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
>    ```
>
>    The translation `(0, 0, 0)` that ships in the file is a **sentinel** that explicitly disables dual-antenna mode — any real installation will have a non-zero offset. Measure with a tape to ±1 cm; the launch file converts your offset into the expected RTK-fixed heading σ via `σ ≈ atan2(0.01 m, baseline_m)`, so a 1.2 m baseline yields σ ≈ 0.48° (well below the runtime sanity-check threshold described later in the orientation-prior section). Whatever offset you enter here must **also** be programmed into the Atlas Duo firmware as `gnss_lever_arm_secondary` (see the lever-arm callout below) — the two representations must agree.
> 4. Regenerate the URDF: `cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py`. The script prints `GNSS antenna mode: DUAL` along with the baseline length and the expected RTK-fixed heading σ. If you see `GNSS antenna mode: SINGLE`, the translation in step 3 is still at the sentinel and step 4 will refuse to enable dual-antenna mode.
>
> **Fallback if you have only one antenna:** leave `gnss_antenna_secondary_link` at its default `(0, 0, 0)`. GLIM++ detects the sentinel at launch time and runs in single-antenna mode — the system still works, the primary antenna alone provides RTK position for both GLIM++'s init pose and the session-long GNSS factor stream. The only loss is the dual-antenna heading benefit (init gates stay at single-antenna defaults and yaw drift remains an IMU concern). Keep `enable_orientation_prior` set to `false` in [`GLIM_plusplus/glim_ext/config/config_gnss_global.json`](GLIM_plusplus/glim_ext/config/config_gnss_global.json) — see the GLIM++ GNSS yaw prior callout below. See [`GLIM_plusplus/README.md`](GLIM_plusplus/README.md) for the side-by-side comparison of what changes between the two modes.

## Sensor Layout

The diagrams below (generated from the v17e SCAD model) label every sensor position relative to the Atlas Duo Center of Navigation (origin) in the ROS REP 103 vehicle frame (+X forward, +Y left, +Z up).

![Top-down sensor layout](3D%20files/sensor_dome_layout_top.jpg)

*Top-down view. LiDARs 1–3 face outward at 0° / 120° / 240°; cameras 1–2 form the 110 mm front stereo baseline flanking LiDAR 1; cameras 3–4 sit on the rear-left and rear-right hex faces.*

![Isometric sensor layout](3D%20files/sensor_dome_layout_iso.jpg)

*Isometric view showing the two-level dome: LiDARs hang from the underside of L2, cameras sit on top of L2, and the GNSS antenna rises above the plate center on its magnetic stand.*

## Repository Structure

```
3D files/                  OpenSCAD model, READMEs, and exported STL files
  sensor_dome.scad         Parametric OpenSCAD source (v17e) — TWO-PIECE
                           bolted assembly (L1 + L2 joined by 12× M6 BHCS)
  sensor_dome_unibody.scad UNIBODY variant — wraps sensor_dome.scad and
                           fuses L1 + brackets + L2 into ONE printable part
  README.md                Detailed design specifications (English)
  README_zh.md             Detailed design specifications (Chinese)
  *.stl                    Exported print-ready meshes (L1, L2)

Documents/           Component datasheets
  Pointonenav-assembly-atlas-duo.pdf
  Seyond-Robin-W1G-Manual.pdf
  e-con_RouteCAM_CU25_IP67_Datasheet.pdf
  e-con_RouteCAM_CU25_IP67_Lens_Datasheet.pdf
  Datasheet_Magnetic_Stand_for_Survey_GNSS_Antenna.pdf

config/              Project-wide configuration (single source of truth)
  sensor_dome_tf.yaml     Static TF transforms (all sensors → imu_link)
  network_config.yaml     NIC, host IP, sensor IPs, DHCP pool
  load_network_config.sh  Sourced by setup_*.sh to export NETCFG_*

PTP_sync/                    One-time host + sensor time-sync setup
  1_install_packages.sh      apt prereqs, RT permissions, ROS 2 Humble/Jazzy
  2_configure_host_network.sh Host NIC static IP, HW-timestamping detect
  3_setup_ins_to_pc_sync.sh   gpsd + chrony + ptp4l GM + phc2sys + p1 driver
  4_setup_lidar_ptp.sh       Robin W PTP slave + Seyond ROS 2 driver
  5_setup_camera_ptp.sh      RouteCAM PTP slave + Aravis (Tier 2 only)
  provision_robin_w_multiunit.sh One-time per-LiDAR IP renumber + UDP port
  README.md                  Architecture, verification, troubleshooting

recording/           Run-time data recording + Foxglove dashboard
  sensor_recorder.py      Detect sensors → verify clock sync → record .mcap
  rate_monitor.py         Per-topic Hz on /sensor_dome/rates
  sensor_config.yaml      IPs, frame_ids, sync tolerances, driver commands
  foxglove/               Pre-built Foxglove Studio layout
  launch/                 Static-TF launch helper
  data/                   Default output root for recorded sessions
  README.md               Architecture and running procedure

GLIM_plusplus/                LiDAR-Inertial mapping (fork of koide3/glim)
  config/                 sensor_dome.urdf + URDF generator
  launch/                 hitch_sensor_dome.launch.py
  scripts/                init-pose helpers, mapping-profile generator, PCD export
  glim/                   Upstream GLIM core (with project tuning)
  glim_ext/               Upstream extensions (GNSS prior re-enabled)
  glim_ros2/              ROS 2 wrapper (multi-LiDAR concat, INS/GNSS bag feed)
  README.md               Fork notice, integration, multi-lap fix

GICP_plusplus/                LiDAR-only localization (fork of vectr-ucla/DLIO)
  cfg/                    localization.yaml (race) + localization_safe.yaml
  launch/                 localization_with_tf.launch.py + systemd units
  scripts/                merge_glim_submaps.py + diagnostic tooling
  include/, src/          small_gicp adapter + localizer + nav_sat_gated_odom
  thirdparty/small_gicp/  vendored header-only registration backend
  README.md               Fork notice, two-mode design, race vs safe
```

## Quick Start

Install [OpenSCAD](https://openscad.org/) and pick one of the two print options below. Both produce the same final geometry; the difference is whether the dome is two bolted plates or one fused part.

### Option A — Two-piece bolted assembly (default, recommended)

The original design. L1 (base + 6 brackets, 139 mm tall) and L2 (top plate, 12 mm thick) print separately and bolt together with 12× M6×20 mm BHCS at the bracket tops. Faster prints, no support material, easy to disassemble for service, easy to swap one plate without reprinting the other.

1. Open `3D files/sensor_dome.scad` in OpenSCAD.
2. Set `RENDER_MODE = 1`, render (F6), export STL (F7) → that's the **L1 print**.
3. Set `RENDER_MODE = 2`, render and export → that's the **L2 print**.
4. Print both on a 305 × 305 mm bed (PETG or ABS, 50–60% infill). No support needed on either part — the brackets are vertical walls and L2 is a flat slab.
5. Assemble with the BOM in [`3D files/README.md`](3D%20files/README.md).

### Option B — Unibody single-piece print

For deployments where you want zero bolted joints between L1 and L2 — slightly stiffer in torsion, no chance of bolt loosening over time, and one file to slice instead of two. Trade-off: a long print with significant support material under the L2 cantilever.

1. Open `3D files/sensor_dome_unibody.scad` in OpenSCAD. This file `include`s `sensor_dome.scad`, so every parameter (pillar height, plate dimensions, sensor mount patterns) automatically tracks the two-piece version.
2. Render (F6) and export STL (F7) → that's the **whole-dome print** (≈ 280 × 300 × 151 mm bounding box, fits a 305 × 305 × 300 mm bed in default L1-down orientation).
3. **Support material is required.** The L2 plate cantilevers between bracket tops over a mostly-empty 280 × 300 mm volume. Use **tree / organic supports** (PrusaSlicer 2.6+, Bambu Studio, Cura "Tree") under L2's underside only — not between bracket walls. Expect ~1.5–2 kg of support PETG and ~28–36 h print time at typical settings; soluble support is impractical at this height. The 12 bracket bolts in the BOM are not needed for this variant.

See [`3D files/README.md`](3D%20files/README.md) for full design specifications, BOM, and assembly instructions.

## 2026-07 localization & mapping improvements (P1–P5)

`GLIM_plusplus/` and `GICP_plusplus/` picked up the 2026-07 turn-error
campaign fixes (2026-07-05), while keeping every dome-specific behavior
(Robin W handling, race/safe modes, yaw-rate-adaptive gains, INS-driven GLIM
init, dual-antenna yaw prior). Headlines — full details in each package
README's "2026-07 P1–P5 improvements" section:

- **GICP++**: confidence-weighted gating (per-map fitness ratios, full-6D
  degeneracy *partial updates* instead of binary rejects, yaw-consistency
  veto), delta-form observer correction (removes the yaw-rate-proportional
  turn lag), state-continuity fixes on rejected scans and GT snaps, per-frame
  merge/scan evidence topics, and a turnkey replay scorecard
  (`GICP_plusplus/scripts/analyze_scan_debug_log.py`).
- **GLIM++**: hardened multi-LiDAR concat (schema gate, strict merge guard,
  per-frame `CONCAT DEBUG` evidence + per-aux clock-offset watch), GNSS
  factor backfill + save-drain fixes, and a per-sample **yaw-quality gate**
  on the dual-antenna heading prior.
- **adapter** (new, `adapter/` at the repo root): the Atlas local-ENU
  normalization package — optional alternative to the existing driver chain;
  notably populates `twist.angular` on INS odometry, which GICP++'s snap
  recovery uses for rate continuity.

### 2026-07-27 GLIM++ upstream re-merge

`GLIM_plusplus/` was re-synced against upstream `augcog/DLIO_plusplus`
(`ucb-roar`) plus its open PR #15, keeping **P1 as the GNSS source and
Robin W as the LiDAR** — only algorithm hardening was taken, not upstream's
different GNSS topic contract. Headlines (full list and merge record in
[`GLIM_plusplus/README.md` §14](GLIM_plusplus/README.md#14-2026-07-27-upstream-re-merge)
and [§10 "Design notes"](GLIM_plusplus/README.md#10-design-notes); upstream
sources are [`koide3/glim`](https://github.com/koide3/glim) and
[`augcog/DLIO_plusplus`](https://github.com/augcog/DLIO_plusplus)):

- **Multi-LiDAR sweep matching no longer trusts header time.** Where an
  absolute per-point time axis exists, aux sweeps are selected by point-time
  endpoint agreement and the offline reader plans every merge in a
  deterministic two-pass join; otherwise it holds each primary until every
  aux header passes it, fixing the "always merge the *previous* side sweep"
  bias. See the timestamp-encoding note above for which path is active.
- **GNSS anchoring safety**: submaps bracketing an RTK dropout are left
  un-anchored instead of chord-interpolated; the one-shot world/GNSS fit has
  RMS and both-sides-baseline gates; an optional rolling anchor-divergence
  gate can abort a run rather than write a quietly-misanchored map; and a
  machine-parseable `gnss_global summary:` line reports delivered-vs-emitted
  factors and anchor coverage (gate map acceptance on that, not exit code).
- **Timestamp integrity**: IMU/LiDAR epoch resets (bag loop, power-cycle) are
  detected and reported loudly instead of silently mixing epochs.
- Online mapping combined with multi-LiDAR concat is now refused at startup —
  no operational change here, since this platform already maps offline.
- New: unit tests for per-point time decoding, a high-quality mapping-profile
  generator, and a PCD exporter under `GLIM_plusplus/scripts/`.

The dome's P1 and Robin W extrinsics were re-verified against the 3D design
files after the merge, tracing the chain from `3D files/sensor_dome.scad`
through [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) and the
generated URDF into the GLIM configs. All three LiDAR mounts measured within
**0.05 mm** of nominal in the unibody STL (80 mm ring radius at 0° / 120° /
240°, z = 114.5 mm above the Atlas Center of Navigation), and the four Atlas
M4 mount features within **0.1 mm** of their SCAD positions. The URDF is
byte-identical to a fresh regeneration from the YAML, so the two cannot drift
apart. Full table, derivations, and the two standing caveats are in
[`GLIM_plusplus/README.md` §10 "TF verification"](GLIM_plusplus/README.md#10-design-notes).

### LiDAR per-point timestamp standard

The repository targets ROS 2 Humble on Ubuntu 22.04 and ROS 2 Jazzy on Ubuntu
24.04. Humble compatibility is mandatory for the official Seyond deployment
path, so the LiDAR contract and its consumers use only APIs common to both
distributions.

The following `sensor_msgs/PointCloud2` layouts are the **canonical ingestion
contract for this repository**. Field name, ROS datatype, unit, and origin are
all part of the contract; datatype alone is not enough to infer the time axis.

| Vendor / profile | Field | ROS datatype | Stored unit | Time origin |
|---|---|---:|---|---|
| Ouster | `t` | `UINT32` | nanoseconds | start of sweep |
| Velodyne | `time` | `FLOAT32` | seconds | start of sweep |
| Hesai | `timestamp` | `FLOAT64` | seconds | Unix epoch (absolute) |
| Livox | `timestamp` | `FLOAT64` | numeric nanoseconds, multiplied by `1e-9` to obtain seconds | Unix epoch (absolute) |
| Seyond Robin W (ROS 2, `coordinate_mode:=3`) | `timestamp` | `FLOAT64` | seconds | Unix epoch (absolute) |

#### Seyond raw payload vs ROS 2

The two layers must not be conflated:

- **Raw packet layer:** each point carries `ts_10us`, a compact offset from its
  packet/frame time origin in 10 microsecond ticks. Robin W frames are 100 ms,
  so a frame contains at most about 10,000 timestamp quanta. This raw offset is
  not an absolute epoch value.
- **Hydrated SDK/PCL/ROS 2 layer:** the driver publishes `double timestamp`, as
  confirmed by Seyond support. PCL converts it to the `sensor_msgs/PointCloud2` field
  `timestamp/FLOAT64/count=1`, in absolute Unix seconds.

The reviewed driver computes:

```text
T_packet_start = packet.common.ts_start_us * 1e-6
delta_t_point  = point.ts_10us * 1e-5
T_point        = T_packet_start + delta_t_point
```

This is source-verified at pinned Seyond commit
[`18c5c936`](https://github.com/Seyond-Inc/seyond_ros_driver/tree/18c5c9362d41cd0766ee1b430f4b431bb14b1ccf):
[`driver_lidar.cc`](https://github.com/Seyond-Inc/seyond_ros_driver/blob/18c5c9362d41cd0766ee1b430f4b431bb14b1ccf/src/seyond_lidar_ros/src/driver/driver_lidar.cc#L558-L604)
performs the reconstruction and
[`point_types.h`](https://github.com/Seyond-Inc/seyond_ros_driver/blob/18c5c9362d41cd0766ee1b430f4b431bb14b1ccf/src/seyond_lidar_ros/src/driver/point_types.h#L35-L52)
registers `timestamp` as `double`.

Equivalently, when the packet offset within the frame is included in
`delta_t_point_from_frame`,
`T_point = T_frame_start + delta_t_point_from_frame`. The cloud
`header.stamp` is `frame_start_ts_us * 1000` nanoseconds. PTP supplies the
shared Unix timebase; `coordinate_mode:=3` changes point axes to REP-103 and
does not change timestamp representation. Never cast an absolute Unix
timestamp to `FLOAT32`, because that destroys sub-second deskew precision.

GLIM uses `perpoint_relative_time=false`, scale `1.0`, and normalizes the
absolute values to offsets from the first point for deskew. GICP++ uses its
explicit `seyond` path and deskews directly on the same absolute values.
The official driver filters invalid returns before publication and hydrates
every published point timestamp; therefore a zero timestamp is not a valid
ROS-layer sentinel. GLIM++ and GICP++ reject a Robin W cloud containing one.
During three-LiDAR concatenation, absolute point timestamps are **not**
shifted by `aux_header_stamp - primary_header_stamp`; they already identify
capture time on the shared PTP axis. `float64_time_is_epoch_ns` remains
`false` because Robin W publishes numeric IEEE-754 seconds, not raw uint64
nanosecond bits.

[`PTP_sync/4_setup_lidar_ptp.sh`](PTP_sync/4_setup_lidar_ptp.sh) pins the
reviewed Seyond commit, removes the repository's obsolete relative-time
override from existing installations, and verifies the source formula and
PCL registration before building. GLIM also rejects a Robin W cloud unless
the field schema and absolute axis match this contract.
Verify a driver or bag before mapping:

```bash
ros2 topic echo --once --field fields /robin_w_front/points
# PointField datatype 8 is FLOAT64; expect: name=timestamp, datatype=8, count=1.
```

The generic GLIM converter retains support for Hesai absolute seconds, Livox
numeric epoch nanoseconds, relative-time LiDARs, and explicit
raw-epoch-nanosecond carriers, but those compatibility paths do not redefine
the Robin W contract above.

### LiDAR vertical-FOV startup gate

Each Robin W must provide approximately its nominal **30 degree vertical field
of view**. GLIM++ and GICP++ announce this gate at startup and validate the
first usable raw PointCloud2 from each configured LiDAR before localization,
mapping, transforms, or concatenation can begin. Each of the three streams is
checked independently, so the wide angular union of a merged cloud cannot hide
one degraded LiDAR. After all configured streams pass, FOV measurement is
disabled for the continuous run; this is a launch-time hardware/data-contract
check, not a per-frame scene-content gate.

The measured span trims only the outer 0.5% of finite, nonzero XYZ return
elevations, preserving the production threshold's intended margin: **27
degrees** for a nominal 30-degree sensor, with at least **100 valid sampled
returns**. Independently, the gate divides that span into bins no wider than
2 degrees and requires every bin to contain at least 0.1% of valid samples
(minimum one). This occupancy check prevents a disconnected high-angle cluster
of any size from making a narrow cloud pass. A cloud below the threshold, with
an occupancy gap, too few valid returns, or an invalid XYZ layout is logged and
rejected before SLAM starts; a pass message is logged for each sensor and once
again when the full startup set is ready. The valid-return setting accepts
100-10000 while the measurement
samples at most 20000 points. The safety knobs are
`lidar_quality.min_vertical_fov_deg` / `min_valid_points` in GLIM and
`localization/lidar_quality/min_vertical_fov_deg` / `min_valid_points` in
GICP.

The fitness-ratio thresholds, `hessianCondMax`, and the
dense-map profile were calibrated on earlier datasets from a different
vehicle and still need a Robin W replay + scorecard pass before tuning —
see the package READMEs.

## Recording & Visualization

Once the dome is built and the sensors are connected, two folders take it from hardware to a usable dataset:

1. **One-time setup** — run the scripts in [`PTP_sync/`](PTP_sync/) to bring up the GPS-disciplined PTP grandmaster on the host and enable IEEE 1588 PTP on every LiDAR and camera. After this, all sensors share a sub-microsecond GPS time base.

2. **Per-session recording** — run the Atlas [`adapter/`](adapter/) alongside [`recording/sensor_recorder.py`](recording/sensor_recorder.py). The recorder auto-detects connected sensors, verifies the clock-sync chain and the adapter's authoritative Fixed-only odometry, then records GNSS / IMU / LiDAR / camera streams into a Foxglove-native MCAP rosbag. A bundled Foxglove Studio layout shows the three Robin W point clouds superimposed in the IMU frame, all four camera views, a GNSS map, an IMU plot, and live per-topic frame rates while the data is being captured.

```bash
# After PTP_sync/ has been run once:
sudo python3 recording/sensor_recorder.py
# Then in Foxglove: Open Connection → ws://localhost:8765
#                   Layouts → Import → recording/foxglove/sensor_dome_layout.json
```

See [`recording/README.md`](recording/README.md) for the architecture diagram and running procedure.

## Mapping (GLIM++)

For SLAM and 3D mapping the project ships **GLIM++**, a heavily modified fork of **GLIM** — *Graph-based LiDAR-Inertial Mapping* by Kenji Koide et al. (AIST), upstream at <https://github.com/koide3/glim>. The fork lives at [`GLIM_plusplus/`](GLIM_plusplus/) (the double-plus signals it is *not* stock GLIM). At a high level, GLIM++ differs from upstream in eight categories:

1. **Sensor adaptation** — topics, frames, and field names are set for the Hitch Sensor Dome (3× Robin W + Atlas Duo + 4× RouteCAM) out of the box: `/imu/data`, `/robin_w_*/points`, `imu_link`/`lidar_front_link` per [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml).
2. **Vehicle-agnostic body frame** — GLIM++ builds the map anchored at `imu_link` (Atlas Duo Center of Navigation), so maps are portable across vehicles. The downstream localizer reports pose in `base_link` (ROS standard body frame), with a static `imu_link → base_link` transform in [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) — identity by default, override per vehicle (rear axle, chassis center, etc) without rebuilding the map.
3. **Outdoor / vehicle-scale tuning** — 24 parameter changes (loosened IMU noise, larger voxels, longer init window, sub-mapping density) calibrated for highway / track / vehicle motion.
4. **Multi-lap loop-closure fix** — wider VGICP convergence basin, looser implicit-loop thresholds, and a stronger GNSS z-prior to prevent the canonical second-lap-tilts-to-the-sky failure mode.
5. **Initialization rewrite (C++)** — gravity-from-accelerometer is removed; the optimizer now requires an external INS pose to start. Allows recordings that begin in motion (mid-session restarts, race-track replays, bag trims).
6. **RTK-fixed gating for the initial pose** — authoritative adapter `kRtkFixed` input plus fail-closed NavSatFix status/covariance and multi-sample stability checks, with a bold-RED CLI warning if the gate is not met within the timeout.
7. **RTK-gated GNSS factor bridge** — soft GNSS prior factors are added to the global graph throughout the session, but only when RTK is fixed. Suspends silently in tunnels and resumes on re-fix.
8. **Optional GNSS yaw prior (dual-antenna only)** — when a dual-antenna RTK heading is available, an optional `PoseRotationPrior` factor pulls each submap's yaw toward the RTK heading. OFF by default; must be turned on manually for dual-antenna installations.

A URDF generator and a `ros2 launch` helper round out the integration. See [`GLIM_plusplus/README.md`](GLIM_plusplus/README.md) for the full file-by-file change log, upstream credits, license preservation, citation, and build instructions.

> ### ⚠ Operational requirement — RTK-fixed GNSS to start a session
>
> **GLIM++ uses the Atlas Duo's RTK-fixed GNSS pose and velocity as the ground truth for initialization.** This replaces the upstream "stationary IMU calibration" requirement with a much sharper one: **a mapping session cannot begin until the Atlas adapter emits a genuine `kRtkFixed` solution with centimetre-grade covariance.** GLIM++ also checks fresh NavSatFix status/covariance and multi-sample stability, then prints a periodic bold-RED warning while waiting. It will not auto-abort, but it will not start collecting map factors until the gate passes.
>
> What this means in the field:
>
> - **Plan for RTK convergence.** Park with clear sky view and wait for RTK-fixed lock before launching GLIM++. Outdoor convergence is typically 30–120 s; longer in marginal conditions. Verify in the Atlas Duo web UI before starting.
> - **NTRIP corrections must be flowing.** The Atlas Duo's Ethernet path to the cellular router (see [`PTP_sync/README.md`](PTP_sync/README.md) §3.1) must reach an NTRIP caster. RTK-fixed without NTRIP is not achievable.
> - **Tunnels and urban canyons during the session are fine** — the per-message RTK gate suspends factor publishing during outages and resumes on re-fix. The session is *not* re-started; only the *initial* pose requires RTK-fixed.
> - **Without RTK** (no base station, no NTRIP) — this is a **diagnostics-only** configuration, not a supported mapping mode. Point the run at a continuous INS source and set `"ins_require_rtk_fixed": false` with `"ins_max_position_stddev": 0.5` in `config_ros.json`. These are configuration keys, not launch arguments. The map still builds, but the world-frame anchor degrades from centimetre-grade to metre-grade, and the run is reported as `rtk_origin_only` rather than `rtk_anchored`. The production profile stays Fixed-only. Note that loosening the NavSatFix covariance threshold alone does **not** open the gate: the gate reads the Atlas solution type, so a Float fix is refused regardless of how good its reported covariance looks.

> ### ⚙ GLIM++ GNSS antenna lever-arm compensation — OFF by default (Atlas Duo only)
>
> **GLIM++ ships with GNSS antenna-to-IMU lever-arm compensation disabled.** The Atlas Duo is a **tightly-coupled GNSS+INS**, not a raw GNSS receiver: its onboard fusion engine already resolves each antenna's RTK observation into the device IMU frame and publishes `/pose` and `/odom` **at the IMU origin**. Adding a second lever-arm correction inside GLIM++ would double-compensate and silently bias the map.
>
> **This is only correct if the Atlas Duo's antenna offsets are configured to match the physical dome geometry.** During Atlas Duo commissioning, the antenna offsets must be entered into the unit's config (Atlas Duo web UI → device configuration), expressed in the device IMU frame in metres:
>
> - **`gnss_lever_arm_primary`** must match the vector from `imu_link` to `gnss_antenna_primary_link` in [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) — currently `(0, 0, 0.273)`.
> - **`gnss_lever_arm_secondary`** (only if a secondary antenna is wired in) must match the vector from `imu_link` to `gnss_antenna_secondary_link`.
>
> If those values are wrong inside the Atlas firmware, the Atlas's own RTK-fixed pose solution will be biased by the rotated lever arm — and **no external compensation in GLIM++ can recover it**. The fix has to be re-flashed into the Atlas config and the session re-recorded.
>
> **If the Atlas Duo is replaced by a non-tightly-coupled GNSS** (a raw RTK receiver such as SwiftNav / u-blox / NovAtel without onboard INS fusion, or any loosely-coupled stack that publishes positions at the **antenna** reference rather than the IMU origin), then lever-arm compensation must be **turned ON inside GLIM++** — the antenna-to-IMU correction is no longer being applied upstream. The recommended implementation reads `imu_link → gnss_antenna_primary_link` from [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) inside the wrapper's GNSS factor bridge (`try_publish_gnss_factor` in `glim_ros.cpp`) and applies `p_imu_utm = p_antenna_utm − R_world_imu · t_imu_gnss` before the message reaches `libgnss_global.so`.

> ### 🧭 GLIM++ GNSS yaw prior — ON by default (dual-antenna RTK)
>
> **GLIM++ ships with a GNSS yaw prior (`PoseRotationPrior`) enabled in [`GLIM_plusplus/glim_ext/config/config_gnss_global.json`](GLIM_plusplus/glim_ext/config/config_gnss_global.json).** Each submap is constrained toward the RTK-derived heading, eliminating the slow yaw drift that LiDAR + IMU alone leave open over a long session. This is the **default configuration** for the Hitch Sensor Dome because the dome ships as a dual-antenna RTK platform.
>
> **How it works.** A dual-antenna RTK receiver measures heading directly from the baseline between the two antennas — drift-free, accurate to roughly 0.1°–4° depending on baseline length. The adapter's `/gps_p1/filtered_odom_rtk_fixed` topic carries that heading only while FusionEngine reports `kRtkFixed`; the wrapper republishes it on `/gnss/pose_rtk_only` with a tight yaw covariance, and `libgnss_global.so` consumes it as a `PoseRotationPrior` factor on every submap. The shipped weighting is `[1e-6, 1e-6, 1e2]` — yaw only, σ_yaw ≈ 0.1 rad (~5.7°), appropriate for a 0.3–1 m baseline. With a longer baseline (e.g. 2 m) the Atlas Duo's heading covariance shrinks and the weight can be raised (try `5e2` for ~2.6° σ). Roll and pitch are kept at near-zero weight because GNSS does not observe them — the IMU + gravity already does.
>
> ### ⚠ GLIM++ GNSS yaw prior — MUST be turned OFF for single-antenna installations
>
> **If the dome runs with only one GNSS antenna, the yaw prior must be disabled.** With a single antenna, the heading on `/pose` is gyro-integrated from the INS — drifting over the session. Tying the optimizer to a drifting reference is worse than leaving yaw under LiDAR + IMU control alone, and the multi-lap z-drift / yaw-drift failure modes that GLIM++ was forked to fix re-appear.
>
> **To switch to single-antenna mode, two changes are required, both before launching GLIM++:**
>
> 1. **TF YAML.** Leave (or set back) `gnss_antenna_secondary_link` to its sentinel `(0, 0, 0)` in [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml). The launch file will then print `single-antenna mode` instead of `dual-antenna mode ENABLED`.
> 2. **GNSS factor config.** Open [`GLIM_plusplus/glim_ext/config/config_gnss_global.json`](GLIM_plusplus/glim_ext/config/config_gnss_global.json) and flip:
>
>    ```jsonc
>      "enable_orientation_prior": false,
>    ```
>
> Step 2 is the load-bearing one: if you change the TF YAML to single-antenna but leave the orientation prior on, the wrapper publishes a loose-covariance quaternion (gyro-integrated INS yaw) and the rotation prior factor will still fire and drag the optimizer toward that drifting heading. The flag must be flipped explicitly.
>
> ### 🛡 Three-layered defense against orientation-prior misconfiguration
>
> The yaw prior is the most consequential dual-antenna feature and the most subtly wrong if any of the three configurations involved (physical mounting, our config files, Atlas firmware) drifts out of sync. GLIM++ runs three independent checks:
>
> 1. **Atlas firmware prerequisite (documented).** The Atlas Duo's `gnss_lever_arm_secondary` and dual-antenna heading mode must be configured in the Atlas web UI to match the physical mounting. This cannot be verified from our code — it has to be set up correctly during Atlas commissioning. See the GNSS antenna lever-arm callout above for the matching parameters.
> 2. **Launch-time consistency check (automatic).** `hitch_sensor_dome.launch.py` reads [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) and [`GLIM_plusplus/glim_ext/config/config_gnss_global.json`](GLIM_plusplus/glim_ext/config/config_gnss_global.json) at startup, and emits a bold-yellow warning if the TF YAML's antenna count disagrees with `enable_orientation_prior`. This catches operator-side mistakes in our own config files.
> 3. **Runtime yaw σ sanity check (automatic).** Once GLIM++ is running, the C++ wrapper compares the Atlas-reported yaw σ on each `/odom` message (the value the Atlas itself publishes in its pose covariance) against the σ we expect from the dual-antenna baseline. After the first ~20 samples, if the Atlas is consistently reporting a much wider σ than expected, a bold-yellow one-shot warning fires explaining that the Atlas firmware is probably *not* in dual-antenna heading mode despite our config saying so. The check requires `ins_odom_topic` to be wired (Odometry carries covariance; PoseStamped does not). A healthy check prints a single info-level confirmation that the dual-antenna heading is live.

```bash
# (one-time) generate sensor_dome.urdf from sensor_dome_tf.yaml
cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py

# Validate the dome mapping configuration and live support nodes:
#   1. Park with clear sky and wait for Atlas Duo RTK-fixed lock.
#   2. Run the launch-time TF/GNSS consistency checks:
ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py

# Build offline against a recorded MCAP bag. The adapter must run during
# capture; the bag must include its Fixed-only odometry plus /gps_p1/fix:
ros2 run glim_ros glim_rosbag recording/data/session_<ts>/rosbag2 \
    --ros-args -p config_path:=GLIM_plusplus/glim/config \
                -p dump_path:=glim_maps/session_<ts>
```

## Localization (GICP++)

For online scan-to-map localization against a pre-built PCD, the project ships **GICP++**, a heavily modified fork of **DLIO** (*Direct LiDAR-Inertial Odometry* by Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez at UCLA's VECTR Lab, upstream at <https://github.com/vectr-ucla/direct_lidar_inertial_odometry>). The fork lives at [`GICP_plusplus/`](GICP_plusplus/) (the double-plus signals it is *not* stock DLIO). GICP++ ships in **two operating modes** selectable at launch time:

- **🏁 Race mode (default)** — front Robin W only, 40 m crop, 32 GICP iterations, yaw-rate-adaptive Kp/Kq attenuation, all debug topics off. Targets ~3–5 ms IMU-to-pose latency for a downstream 200 Hz controller on a pre-built race-track map.
- **🛡 Safe mode** — all three Robin W LiDARs concatenated, 100 m crop, upstream-strict 128 GICP iterations, tighter convergence epsilons, all debug topics on. Targets maximum sensor coverage + diagnostic visibility when CPU headroom is plentiful and latency isn't the bottleneck.

Key project upgrades vs. the upstream VECTR DLIO:

1. **Robin W + Atlas Duo targeting** — defaults use low-latency `/imu/data`, adapter `/gps_p1/filtered_odom_rtk_fixed`, and `/robin_w_*/points`; adapter `/gps_p1/imu` is also allowed for normalized replay. Frames come from `config/sensor_dome_tf.yaml`.
2. **Fixed-only RTK policy** — the adapter checks FusionEngine `solution_type == RTK_FIXED` and covariance before publishing `/gps_p1/filtered_odom_rtk_fixed`. Float/no-fix periods use LiDAR+IMU only. The optional [`nav_sat_gated_odom`](GICP_plusplus/src/nav_sat_gated_odom.cc) compatibility bridge is fail-closed for covariance and freshness, but REP-145 `STATUS_GBAS_FIX` is only RTK-class and cannot itself distinguish float from fixed.
3. **Two-mode design** — `cfg/localization.yaml` (race base) + `cfg/localization_safe.yaml` (overlay) layered by the launch file's `mode:=race|safe|custom` arg, with paired mutually-exclusive systemd units for production deployment.
4. **Verified GLIM++ map bridge** — `GLIM_plusplus/scripts/export_glim_dump_to_pcd.py` decodes the exact compact layout, applies each `T_world_origin` and `inverse(T_world_utm)`, and writes an ENU PCD plus datum manifest. The adapter publishes its resolved datum on transient-local `/gps_p1/local_enu_origin`; GICP refuses point clouds and RTK odometry until that live datum matches the manifest.
5. **GICP warm-start** — eager kd-tree build at init + one dummy align to burn OpenMP thread-pool spin-up, Eigen JIT, and source-side kd-tree allocation before the first real scan.
6. **Yaw-rate adaptive observer** — `Kp` and `Kq` in the geometric observer auto-attenuate at high body-frame yaw rate so the IMU prediction takes precedence through corner entries where GICP is most likely to slide.
7. **Motion-variance gate on IMU calibration** — refuses the stationary-bias calibration window if the vehicle is moving (σ‖a‖ > 0.10 m/s²); falls back to RTK-driven calibration via the gt_odom path.
8. **Operator-side health checks** — one-shot bold-yellow warning at 10 s if `gt_odom` never arrives; throttled diagnostic logs during dead-reckoning streaks.
9. **base_link / imu_link split** — localizer reports pose in `base_link` (configurable per vehicle via [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml)) even though GLIM++ builds the map in `imu_link`. Same map works across vehicles with different body frames.

```bash
# The surveyed datum must be identical in the adapter, map export, and GICP.
ENU_ORIGIN="<lat_deg>,<lon_deg>,<alt_m>"

# 1. Build a map offline with GLIM++ (see Mapping above), then export GLIM
#    WORLD into the surveyed local-ENU frame. This also writes
#    /tmp/race_map.pcd.manifest.yaml.
python3 GLIM_plusplus/scripts/export_glim_dump_to_pcd.py \
    /tmp/dump /tmp/race_map.pcd \
    --frame enu --enu-origin "${ENU_ORIGIN}" --voxel-size 0.4

# 2. Run the localizer in race mode (default):
ros2 launch gicp_localization localization_with_tf.launch.py \
    map_path:=/tmp/race_map.pcd \
    local_enu_origin:="${ENU_ORIGIN}"

# Or safe mode (3× LiDARs, full coverage, all debug on):
ros2 launch gicp_localization localization_with_tf.launch.py \
    mode:=safe \
    map_path:=/tmp/race_map.pcd \
    local_enu_origin:="${ENU_ORIGIN}"
```

See [`GICP_plusplus/README.md`](GICP_plusplus/README.md) for the complete file-by-file changelog vs. upstream, side-by-side race vs. safe knob table, systemd unit installation, and latency budget breakdown.

## Coordinate System (ROS REP 103)

- **+X** = Forward, **+Y** = Left, **+Z** = Up
- **Origin** = Atlas Duo Center of Navigation (CoN)

## Credits

This project was designed and is maintained by **Dr. Allen Y. Yang** (Hitch Interactive · University of California, Berkeley).

Implementation testing and field validation were carried out by the **Berkeley AI Racing Tech** team: from UC Berkeley (alphabetical by last name) — Bryan Chang, Logan Kinajil-Moran, Moises Lopez Mendoza, Gary Passon, Tanishaa Viral Shah, Joshua Sun, Jovan Yap; from UC San Diego — Kevin Shin.

Please cite or credit this repository when reusing any of the mechanical design, the ROS 2 TF configuration, the PTP synchronization pipeline, or the recording / visualization tooling in derivative work:

> Yang, A. Y. *Hitch Sensor Dome: a 3D-printable modular multi-sensor mount for vehicle-roof mapping.* GitHub repository, 2026.

Thanks to the OpenSCAD, ROS 2, linuxptp, chrony, Aravis, Foxglove, and MCAP communities whose open-source tooling this project builds on. The mapping pipeline is built on **GLIM** by Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno (AIST) — see [`GLIM_plusplus/README.md`](GLIM_plusplus/README.md) for the full upstream attribution, license preservation, and citation.

## License

See [LICENSE](LICENSE) for details.
