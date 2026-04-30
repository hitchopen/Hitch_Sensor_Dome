# Hitch Sensor Dome

A 3D-printable modular sensor dome for mounting a multi-sensor mapping rig on a vehicle roof via a suction cup camera mount.

## Sensors

- 3× Seyond Robin W LiDAR — 120° HFOV each, arranged at 120° intervals for full 360° surround coverage
- 1× Point-One Nav Atlas Duo INS — center-mounted, CoN at geometric origin
- 1× **Point One SP1** multi-frequency (L1/L2/L5) GNSS antenna — on ArduSimple survey stand, centered above CoN
- 4× e-con RouteCAM_P_CU25_CXLC_IP67 cameras — front stereo pair (104 mm baseline) + rear symmetric pair

> **GNSS antenna note.** The Atlas Duo's GNSS port supplies **3.3 V DC bias** for a powered LNA (per the [Point One Atlas User Guide](https://pointonenav.com/wp-content/uploads/2024/06/Atlas-User-Guide.pdf)). The [Point One SP1](https://store.pointonenav.com/products/sp1-high-precision-gnss-antenna) ships voltage-matched to this rail and is the recommended default. The ArduSimple "Magnetic Stand for Survey GNSS Antenna" used in the BOM has a **5/8"-11 UNC** thread (the surveying-pole standard); thread compatibility of each candidate antenna is summarized below.
>
> | Antenna | Bands | Thread fit on the ArduSimple stand |
> |---------|-------|-------------------------------------|
> | [Point One SP1](https://store.pointonenav.com/products/sp1-high-precision-gnss-antenna) | L1 / L2 / L5 | **Drop-in.** SP1 kit ships with a magnetic mount + 75 mm riser carrying a 5/8"-11 UNC thread. |
> | [Tallysman TW3972](https://www.tallysman.com/product/tw3972-triple-band-gnss-antenna-with-l-band/) | L1 / L2 / L5 + L-band | **Adapter required.** Native mount is through-hole / flat-bottom. Add a Calian / Tallysman [Pipe Mount Adapter PN 23-0065-0](https://www.calian.com/advanced-technologies/gnss_product/pipe-mount-adapter-screw-compression-pn-23-0065-0/) (or PN 23-0040-0 L-bracket) to expose a 5/8"-11 UNC thread. |
> | [Harxon HX-CSX601A](https://en.harxon.com/product/detail/99) | GPS/GLONASS/Galileo/BeiDou multi-band | **Verify on current datasheet.** Survey-grade, TNC-F. Public sources are inconsistent on whether the thread is 5/8"-11 UNC (the surveying standard) or 5/8"-12; confirm with Harxon's brochure for the unit you actually order before committing. |
>
> Verify each candidate's LNA voltage / current spec against 3.3 V before connecting, and add a DC block if the antenna is already powered from another source. Do **not** repurpose the GNSS pigtail off a combo LTE + GNSS antenna (Peplink, etc.) — even the better combo antennas are a meaningful regression from the SP1 in band coverage and noise figure, and the resulting RTK fix quality drops accordingly.

## Sensor Layout

The diagrams below (generated from the v17c SCAD model) label every sensor position relative to the Atlas Duo Center of Navigation (origin) in the ROS REP 103 vehicle frame (+X forward, +Y left, +Z up).

![Top-down sensor layout](3D%20files/sensor_dome_layout_top.jpg)

*Top-down view. LiDARs 1–3 face outward at 0° / 120° / 240°; cameras 1–2 form the 104 mm front stereo baseline flanking LiDAR 1; cameras 3–4 sit on the rear-left and rear-right hex faces.*

![Isometric sensor layout](3D%20files/sensor_dome_layout_iso.jpg)

*Isometric view showing the two-level dome: LiDARs hang from the underside of L2, cameras sit on top of L2, and the GNSS antenna rises above the plate center on its magnetic stand.*

## Repository Structure

```
3D files/           OpenSCAD model, READMEs, and exported STL files
  sensor_dome.scad   Parametric OpenSCAD source (v17c)
  README.md          Detailed design specifications (English)
  README_zh.md       Detailed design specifications (Chinese)
  *.stl              Exported print-ready meshes (L1, L2)

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

PTP_sync/            One-time host + sensor time-sync setup
  setup_ubuntu_sync.sh    GPS-disciplined PTP grandmaster (gpsd → chrony → ptp4l)
  setup_robin_w_sync.sh   Enable PTP on Seyond Robin W LiDARs
  setup_camera_sync.sh    Enable IEEE 1588 PTP on RouteCAM cameras
  README.md               Architecture, verification, troubleshooting

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
  docs/                   Multi-lap loop-closure debugging guide
  glim/                   Upstream GLIM core (with project tuning)
  glim_ext/               Upstream extensions (GNSS prior re-enabled)
  glim_ros2/              Upstream ROS 2 wrapper (unmodified)
  README.md               Fork notice, integration, multi-lap fix
```

## Quick Start

1. Install [OpenSCAD](https://openscad.org/)
2. Open `3D files/sensor_dome.scad`
3. Set `RENDER_MODE = 1` for Level 1, `RENDER_MODE = 2` for Level 2
4. Render (F6) and export STL (F7)
5. Print both parts on a 305 × 305 mm bed (PETG or ABS, 50–60% infill)

See [`3D files/README.md`](3D%20files/README.md) for full design specifications, BOM, and assembly instructions.

## Recording & Visualization

Once the dome is built and the sensors are connected, two folders take it from hardware to a usable dataset:

1. **One-time setup** — run the scripts in [`PTP_sync/`](PTP_sync/) to bring up the GPS-disciplined PTP grandmaster on the host and enable IEEE 1588 PTP on every LiDAR and camera. After this, all sensors share a sub-microsecond GPS time base.

2. **Per-session recording** — run [`recording/sensor_recorder.py`](recording/sensor_recorder.py) to auto-detect connected sensors, verify the clock-sync chain, and record GNSS / IMU / LiDAR / camera streams into a Foxglove-native MCAP rosbag. A bundled Foxglove Studio layout shows the three Robin W point clouds superimposed in the IMU frame, all four camera views, a GNSS map, an IMU plot, and live per-topic frame rates while the data is being captured.

```bash
# After PTP_sync/ has been run once:
sudo python3 recording/sensor_recorder.py
# Then in Foxglove: Open Connection → ws://localhost:8765
#                   Layouts → Import → recording/foxglove/sensor_dome_layout.json
```

See [`recording/README.md`](recording/README.md) for the architecture diagram and running procedure.

## Mapping (GLIM++)

For SLAM and 3D mapping the project ships **GLIM++**, a heavily modified fork of **GLIM** — *Graph-based LiDAR-Inertial Mapping* by Kenji Koide et al. (AIST), upstream at <https://github.com/koide3/glim>. The fork lives at [`GLIM_plusplus/`](GLIM_plusplus/) (the double-plus signals it is *not* stock GLIM). At a high level, GLIM++ differs from upstream in seven categories:

1. **Sensor adaptation** — topics, frames, and field names switched from the previous AV-24 / Luminar deployment to the Hitch Sensor Dome (3× Robin W + Atlas Duo + 4× RouteCAM).
2. **Vehicle-agnostic body frame** — `base_frame_id = imu_link`, so maps anchor to the Atlas Duo Center of Navigation and are portable across vehicles.
3. **Outdoor / vehicle-scale tuning** — 24 parameter changes (loosened IMU noise, larger voxels, longer init window, sub-mapping density) calibrated for highway / track / vehicle motion.
4. **Multi-lap loop-closure fix** — wider VGICP convergence basin, looser implicit-loop thresholds, and a stronger GNSS z-prior to prevent the canonical second-lap-tilts-to-the-sky failure mode.
5. **Initialization rewrite (C++)** — gravity-from-accelerometer is removed; the optimizer now requires an external INS pose to start. Allows recordings that begin in motion (mid-session restarts, race-track replays, bag trims).
6. **RTK-fixed gating for the initial pose** — three-stage gate on NavSatFix status, covariance, and multi-sample stability, with a bold-RED CLI warning if the gate is not met within the timeout.
7. **RTK-gated GNSS factor bridge** — soft GNSS prior factors are added to the global graph throughout the session, but only when RTK is fixed. Suspends silently in tunnels and resumes on re-fix.

A URDF generator and a `ros2 launch` helper round out the integration. See [`GLIM_plusplus/README.md`](GLIM_plusplus/README.md) for the full file-by-file change log, upstream credits, license preservation, citation, and build instructions.

> ### ⚠ Operational requirement — RTK-fixed GNSS to start a session
>
> **GLIM++ uses the Atlas Duo's RTK-fixed GNSS pose and velocity as the ground truth for initialization.** This replaces the upstream "stationary IMU calibration" requirement with a much sharper one: **a mapping session cannot begin until the Atlas Duo reports an RTK-fixed solution with centimetre-grade covariance.** GLIM++ enforces this in C++ via a three-stage gate (status, covariance, multi-sample stability) and prints a periodic bold-RED warning while waiting. It will not auto-abort — but it will not start collecting map factors either, until the gate passes.
>
> What this means in the field:
>
> - **Plan for RTK convergence.** Park with clear sky view and wait for RTK-fixed lock before launching GLIM++. Outdoor convergence is typically 30–120 s; longer in marginal conditions. Verify in the Atlas Duo web UI before starting.
> - **NTRIP corrections must be flowing.** The Atlas Duo's Ethernet path to the cellular router (see [`PTP_sync/README.md`](PTP_sync/README.md) §3.1) must reach an NTRIP caster. RTK-fixed without NTRIP is not achievable.
> - **Tunnels and urban canyons during the session are fine** — the per-message RTK gate suspends factor publishing during outages and resumes on re-fix. The session is *not* re-started; only the *initial* pose requires RTK-fixed.
> - **Without RTK** (no base station, no NTRIP) — the gate can be relaxed via `ins_require_rtk_fixed:=false ins_max_position_stddev:=0.5`, accepting RTK-float or SBAS for init. The map is still useful but the world-frame anchor is loose at the metre scale rather than the centimetre scale. See [`GLIM_plusplus/docs/moving_start_initialization.md`](GLIM_plusplus/docs/moving_start_initialization.md) §"Operating without RTK".

```bash
# (one-time) generate sensor_dome.urdf from sensor_dome_tf.yaml
cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py

# Live mapping against the recording stack:
#   1. Park with clear sky and wait for Atlas Duo RTK-fixed lock.
#   2. Launch:
ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py

# Or offline against a recorded MCAP bag (the bag must include /pose + /gps/fix):
ros2 run glim_ros glim_rosbag recording/data/session_<ts>/rosbag2 \
    --ros-args -p config_path:=GLIM_plusplus/glim/config \
                -p dump_path:=glim_maps/session_<ts>
```

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
