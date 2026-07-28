# Hitch Sensor Dome

A 3D-printable modular sensor dome that mounts a multi-sensor mapping and
localization rig on a vehicle roof via a suction-cup camera mount, together with
the ROS 2 software that turns it into a working pipeline: an offline
LiDAR–inertial mapper, a real-time localizer, and the clock synchronization that
makes both of them trustworthy.

This README is the map of the project. Every package keeps its own technical
documentation, and this file links to it rather than repeating it:

| Package | What it is | Read |
|---|---|---|
| [`GLIM_plusplus/`](GLIM_plusplus/) | Offline LiDAR–inertial mapping, RTK-anchored | [README](GLIM_plusplus/README.md) |
| [`GICP_plusplus/`](GICP_plusplus/) | Real-time localization against a prebuilt map | [README](GICP_plusplus/README.md) |
| [`adapter/`](adapter/) | Point One Atlas GNSS/INS normalization node | [README](adapter/README.md) |
| [`PTP_sync/`](PTP_sync/) | One-time clock synchronization setup | [README](PTP_sync/README.md) |
| [`recording/`](recording/) | Session recorder and Foxglove visualization | [README](recording/README.md) |
| [`3D files/`](3D%20files/) | SCAD source, STLs, BOM, assembly | [README](3D%20files/README.md) |

---

## 1. Sensors, and which of them the software actually uses

The reference build carries:

- **3× Seyond Robin W LiDAR** — 120° horizontal FOV each, mounted at 0° / 120° / 240° for 360° surround coverage
- **1× Point One Atlas Duo INS** — center-mounted, its Center of Navigation at the geometric origin
- **1× Point One SP1 GNSS antenna** (L1/L2/L5) on a survey stand, centered above the CoN
- **4× e-con RouteCAM cameras** — front stereo pair plus a rear pair

**The cameras are an optional mount, and no mapping or localization algorithm
in this repository consumes them.** The default GLIM++ build disables camera
support at compile time: it does not discover or link OpenCV, `cv_bridge`, or
`image_transport`, and its binaries contain no camera subscription or rosbag
image-deserialization path. The dome still carries camera extrinsics in
[`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml), and the recorder can
capture camera topics when cameras are present, but neither the map nor the pose
estimate is a function of camera data. Camera intake inherited from upstream
GLIM is an explicit opt-in build described in the
[GLIM++ README](GLIM_plusplus/README.md#build-dependencies).

### Enabling the optional cameras

Camera recording and GLIM image intake are separate opt-ins:

1. Provision the RouteCAM hardware, PTP, Aravis driver, and ROS camera packages.
   Script 5 is intentionally excluded from the default P1 + LiDAR setup:

   ```bash
   cd PTP_sync
   ./5_setup_camera_ptp.sh --eth <sensor-interface>
   cd ..
   ```

2. To record cameras, keep their real IP addresses, frames, and topics in
   [`recording/sensor_config.yaml`](recording/sensor_config.yaml), then run the
   normal recorder. It detects all reachable cameras and lets the operator
   select them; no GLIM rebuild is required just to record images:

   ```bash
   python3 recording/sensor_recorder.py --profile glim
   ```

3. To compile upstream-compatible GLIM image intake, rebuild all three GLIM
   packages with both camera layers enabled. A clean CMake cache is required
   when changing from the camera-free build:

   ```bash
   sudo apt install -y libopencv-dev
   colcon build --packages-select glim glim_ext glim_ros --symlink-install \
     --cmake-clean-cache \
     --cmake-args -DCMAKE_BUILD_TYPE=Release \
                  -DBUILD_WITH_OPENCV=ON \
                  -DBUILD_WITH_CV_BRIDGE=ON
   source install/setup.bash
   ```

4. Set `glim_ros.image_topic` in
   [`GLIM_plusplus/glim/config/config_ros.json`](GLIM_plusplus/glim/config/config_ros.json)
   to one published `sensor_msgs/msg/Image` topic, for example
   `/cam_front_left/image_raw`, and enable the image-consuming GLIM extension
   required by the application. When using a generated run profile, edit that
   profile's active `config_ros.json` instead. The shipped mapping modules do
   not consume images, so enabling the bridge alone does not change map
   estimation.

GLIM's inherited interface currently accepts one configured image topic.
The recorder can capture all four cameras. GICP++ has no camera intake and
remains LiDAR-only at every build setting.

Supported camera-free sensor sets are:

| Pipeline | Required sensors | Optional sensors |
|---|---|---|
| GICP++ | P1/Atlas plus the front Robin W | either or both rear Robin W units |
| GLIM++ | P1/Atlas plus all three Robin W units | cameras only |

For GICP++, a two-LiDAR installation still runs `front_only`; select
`three_lidar` only when both rear units are present. GLIM always uses the full
three-LiDAR set. The RTK-fixed and dual-heading policies described below remain
quality requirements for production GLIM mapping, not additional camera
dependencies.

### Sensor layout

![Top-down sensor layout](3D%20files/sensor_dome_layout_top.jpg)

*Top-down view. LiDARs 1–3 face outward at 0° / 120° / 240°; cameras 1–2 form
the 110 mm front stereo baseline flanking LiDAR 1; cameras 3–4 sit on the rear
hex faces.*

![Isometric sensor layout](3D%20files/sensor_dome_layout_iso.jpg)

*Isometric view of the two-level dome: LiDARs hang from the underside of L2,
cameras sit on top of L2, and the GNSS antenna rises above the plate center.*

---

## 2. Dual GNSS antennas are mandatory for production mapping

**A second GNSS antenna is required for a production GLIM++ map.** It is not a
requirement for front-only GICP++ localization or its recording profile. With
two antennas at a known fixed baseline, the Atlas Duo solves heading *directly*
from the RTK carrier-phase difference between them: an absolute, drift-free
measurement, typically accurate to 0.1°–1° depending on baseline length,
available the moment RTK goes fixed and stable for the entire session.

> [!CAUTION]
> **DUAL-ANTENNA MODE REQUIRES A MANUAL TF UPDATE.** You must measure the real
> installed position of the secondary antenna and replace the
> `imu_link → gnss_antenna_secondary_link` translation in
> [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml). The checked-in
> default translation `(0, 0, 0)` is an explicit sentinel and is treated as
> **NO SECONDARY ANTENNA**, even when a second antenna is physically connected.

With one antenna there is no heading measurement at all. Heading has to be
integrated from the IMU gyroscope, so it is only ever as good as the bias
estimate, and the error grows without bound until something else corrects it. On
a ground vehicle that correction has to come from LiDAR scan matching against
structure, which is exactly what fails on a race track: long, low-feature
straights and repeated laps give the matcher little to bite on in the direction
that matters. This is the origin of the canonical multi-lap yaw-drift and
z-drift failures, and it is a problem that is far cheaper to *not have* than to
compensate for downstream.

Recommended baseline is **1.0–1.5 m** on a vehicle roof; below about 0.5 m the
angular accuracy degrades faster than it is worth. Mount the second antenna
rigidly — a metal plate or a printed extension bolted to the dome, never
adhesive or magnetic alone — because any flex in the baseline goes straight into
the heading solution.

Two things are then required to be consistent, and the system does not check
them for you: the measured `imu_link → gnss_antenna_secondary_link` offset in
[`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml), and the same lever
arm programmed into the Atlas Duo's own firmware configuration. If they
disagree, the heading is wrong in a way that looks plausible.

**What ships in this repository is single-antenna.** The 3D reference design
provides one antenna mount, and the checked-in configuration has the
orientation prior disabled — not because one antenna is sufficient, but because
the correct second-antenna geometry depends on the vehicle and cannot be
supplied as a default. A deployment is expected to add the antenna, measure the
offset, and enable the prior. The mechanical, wiring, and configuration
walkthrough is in
[`GLIM_plusplus/README.md` §1.1](GLIM_plusplus/README.md#11-single-antenna-vs-dual-antenna-mode);
the single-antenna path remains supported for GICP++ and for bringing the rig up
before the second antenna is fitted.

---

## 3. Clock synchronization is a prerequisite, not a nicety

Every sensor on the dome is disciplined to one clock: the LiDARs over PTP, the
host over the Atlas Duo's GNSS-derived time, and the Atlas Duo's own stream
mapped into ROS time by the adapter. This is not an optimization. Both pipelines
in this repository reason about *when a laser return happened*, not about when a
message arrived, and neither is meaningful if the sensors disagree about the
time.

Two consequences follow directly. Sweep merging across three LiDARs is done on
absolute per-point capture time, which only exists if the LiDARs share a clock
(§4). And GNSS factors are attached to poses by message time, so an unsynchronized
receiver silently anchors the map to the wrong place along the trajectory.

Run [`PTP_sync/`](PTP_sync/README.md) once per host before recording anything.
It installs and configures the time chain and verifies the LiDAR driver's wire
contract. A run captured without it may look normal and still produce a map that
is quietly wrong.

---

## 4. Per-point timestamp matching across three 120° LiDARs

The three Robin W units are not frame-synchronized with each other. PTP
disciplines their *clocks*, not their *scan phase*, so at any instant the three
sensors are at different points in their sweeps. Merging them on message header
time therefore pairs the wrong sweeps together and smears the merged cloud.

Both pipelines instead match sweeps on the **absolute per-point capture time**
the Robin W driver publishes, comparing the first and last point times of each
candidate sweep and accepting a merge only when both endpoints agree within a
configured tolerance. The offline mapper additionally plans every merge in a
deterministic two-pass join over the whole bag, so the same recording produces
the same map on every replay. Each raw sensor frame is also checked for vertical
FOV coverage before it can enter the pipeline, so a degraded or occluded unit is
caught at startup rather than being averaged into the other two.

The encoding contract, the matching algorithm, the tolerance, and the FOV gate
are documented in [`GLIM_plusplus/README.md`](GLIM_plusplus/README.md) and
[`GICP_plusplus/README.md`](GICP_plusplus/README.md).

### Canonical per-point timestamp standard

The repository targets ROS 2 Humble on Ubuntu 22.04 and ROS 2 Jazzy on Ubuntu
24.04. Humble compatibility is mandatory for the official Seyond deployment
path, so the LiDAR contract and its consumers use only APIs common to both
distributions.

Field name, ROS datatype, stored unit, and origin are all part of the ingestion
contract:

| Vendor / profile | Field | ROS datatype | Stored unit | Time origin |
|---|---|---:|---|---|
| Ouster | `t` | `UINT32` | nanoseconds | start of sweep |
| Velodyne | `time` | `FLOAT32` | seconds | start of sweep |
| Hesai | `timestamp` | `FLOAT64` | seconds | Unix epoch (absolute) |
| Livox | `timestamp` | `FLOAT64` | numeric nanoseconds, multiplied by `1e-9` to obtain seconds | Unix epoch (absolute) |
| Seyond Robin W (ROS 2, `coordinate_mode:=3`) | `timestamp` | `FLOAT64` | seconds | Unix epoch (absolute) |

For Robin W, do not conflate the raw packet representation with the hydrated
ROS representation:

- A raw point carries `ts_10us`, an offset in 10 microsecond ticks from its
  packet/frame time origin. A 100 ms frame contains at most about 10,000 such
  quanta.
- The Seyond SDK exposes `double timestamp`. The ROS 2 driver reconstructs the
  point time and PCL publishes `timestamp/FLOAT64/count=1` as absolute Unix
  seconds.

The pinned driver computes:

```text
T_packet_start = packet.common.ts_start_us * 1e-6
delta_t_point  = point.ts_10us * 1e-5
T_point        = T_packet_start + delta_t_point
```

Equivalently, `T_point = T_frame_start + delta_t_point_from_frame`. The cloud
`header.stamp` is `frame_start_ts_us * 1000` nanoseconds. PTP supplies the
shared Unix timebase; `coordinate_mode:=3` changes point axes to REP 103 and
does not change time representation. Never cast an absolute Unix timestamp to
`FLOAT32`, because doing so destroys sub-second deskew precision.

This implementation is source-verified at pinned Seyond commit
[`18c5c936`](https://github.com/Seyond-Inc/seyond_ros_driver/tree/18c5c9362d41cd0766ee1b430f4b431bb14b1ccf):
[`driver_lidar.cc`](https://github.com/Seyond-Inc/seyond_ros_driver/blob/18c5c9362d41cd0766ee1b430f4b431bb14b1ccf/src/seyond_lidar_ros/src/driver/driver_lidar.cc#L558-L604)
performs the reconstruction and
[`point_types.h`](https://github.com/Seyond-Inc/seyond_ros_driver/blob/18c5c9362d41cd0766ee1b430f4b431bb14b1ccf/src/seyond_lidar_ros/src/driver/point_types.h#L35-L52)
registers `timestamp` as `double`.

GLIM uses `perpoint_relative_time=false`, scale `1.0`, and normalizes absolute
values to offsets from the first point for deskew. GICP++ uses its explicit
`seyond` path and deskews on the same absolute values. A zero timestamp is not
a valid hydrated Robin W value and is rejected. Three-LiDAR concatenation must
not add `aux_header_stamp - primary_header_stamp` to these point times because
they already identify capture time on the shared PTP axis.
`float64_time_is_epoch_ns` remains `false`: Robin W publishes numeric IEEE-754
seconds, not raw `uint64` nanosecond bits.

Verify a live driver or recorded bag before mapping:

```bash
ros2 topic echo --once --field fields /robin_w_front/points
# PointField datatype 8 is FLOAT64; expect timestamp, datatype=8, count=1.
```

---

## 5. Mapping — GLIM++, offline and RTK-anchored

[`GLIM_plusplus/`](GLIM_plusplus/) builds the map. It is a heavily modified fork
of [koide3/glim](https://github.com/koide3/glim) that runs **offline against a
recorded MCAP bag**, not live. Offline is a deliberate choice: it removes the
real-time budget, so the mapper can afford dense downsampling, a deterministic
multi-LiDAR join, and a global factor graph over the whole session.

Its defining property here is that it is **tightly integrated with GNSS RTK**. A
session does not begin until the Atlas Duo reports a validated RTK-**fixed**
solution, so the map origin is globally referenced from the first factor rather
than being an arbitrary local frame that has to be aligned afterwards. Through
the run, RTK-fixed positions enter the graph as global constraints that bound
the drift a pure LiDAR–inertial solution accumulates. RTK-float is never
accepted: a float solution carries a *consistent* decimetre-scale bias, which
neither a robust loss nor the reported covariance protects against. When RTK
drops out, LiDAR–inertial odometry simply carries the trajectory unanchored
until a fixed solution returns.

The map that comes out is exported into a surveyed local-ENU frame, which is
what makes it directly usable by the localizer.

---

## 6. Localization — GICP++, real-time, one or three LiDARs

[`GICP_plusplus/`](GICP_plusplus/) localizes in real time against a map built by
GLIM++. Its registration backend derives from
[**small_gicp**](https://github.com/koide3/small_gicp), which replaced the
original nano_gicp backend and matches what upstream now uses.

It runs in **two LiDAR modes**, selected explicitly at launch:

- **`front_only`** — the single front Robin W. Lowest compute load and lowest
  latency, which is what a racing deployment usually wants. This is the default.
- **`three_lidar`** — front plus both rear units, merged on per-point time as in
  §4. Full 360° geometry for the matcher, at a higher cost per frame.

The mode is one knob, not two: the older `lidar_concat/enabled` boolean is
derived from it, so the sensor topology cannot be configured inconsistently. The
shipped systemd units pin the pairing they were tuned for — race with
`front_only`, safe with `three_lidar`.

---

## 7. Why these forks diverge from upstream

Upstream GLIM and DLIO/GICP are both tuned, reasonably, for the platforms their
authors targeted: **spinning LiDARs on drones and handheld or slow indoor
rigs**. Those assumptions run through the defaults — modest voxel resolutions
suited to indoor scale, short smoother windows, stationary-start initialization,
convergence basins sized for walking speed, and per-point time handling built
around a rotating sensor's scan model.

This project retunes both for a different problem: a **high-speed ground vehicle
on a race track**. In practice that has meant outdoor-scale voxel and
correspondence distances; smoother and convergence settings that survive hard
acceleration and aggressive yaw; initialization from a moving start, because a
recording rarely begins with the vehicle parked; semi-solid-state LiDAR geometry
instead of a rotating scan model; and gating that treats a bad global anchor as
a failure to report rather than a cost to absorb, because a race map that is
quietly a few degrees rotated is worse than one that refuses to build.

Every algorithmic change is enumerated, with its rationale, in each package's
own README. Upstream credit and license text are preserved unchanged.

---

## 8. Repository layout

```
3D files/        SCAD source, STLs, BOM, print and assembly guide
config/          sensor_dome_tf.yaml — the single TF source of truth
PTP_sync/        one-time clock synchronization setup and verification
recording/       session recorder, sensor config, Foxglove layout
adapter/         Point One Atlas -> ROS normalization node
GLIM_plusplus/   offline LiDAR-inertial mapping (fork of koide3/glim)
GICP_plusplus/   real-time localization (small_gicp backend)
```

`config/sensor_dome_tf.yaml` is the one file every other component reads its
geometry from. The URDF that GLIM++ and GICP++ consume is generated from it, and
the launch files publish their static transforms from it, so there is no second
copy to drift.

---

## 9. Quick start

Print and assemble the dome from [`3D files/README.md`](3D%20files/README.md),
then, in order:

1. **Synchronize clocks** — run [`PTP_sync/`](PTP_sync/README.md) once per host.
   Nothing downstream is trustworthy before this.
2. **Record a session** — see [`recording/README.md`](recording/README.md). The
   deployment's native-message Atlas driver and the adapter must already be
   publishing live data. Before opening the bag, the recorder independently
   compares every selected LiDAR/camera `header.stamp` with host
   `CLOCK_REALTIME`. This is verification only: all clock configuration and
   synchronization belong to `PTP_sync/`, and the recorder never changes them.
   Use `--profile glim` for a map-building capture.
3. **Build a map offline** — see [`GLIM_plusplus/README.md`](GLIM_plusplus/README.md).
   Park with clear sky and wait for RTK-fixed before starting the session.
4. **Localize in real time** — see [`GICP_plusplus/README.md`](GICP_plusplus/README.md).
   The surveyed ENU datum must match between the map export and the localizer.

## Coordinate system (ROS REP 103)

**+X** forward, **+Y** left, **+Z** up. The origin is the Atlas Duo Center of
Navigation.

## Contact

Maintained by **Dr. Allen Y. Yang** — Hitch Interactive · University of
California, Berkeley. Please open a GitHub issue for questions, corrections, or
reuse enquiries.

## Credits

This project was designed and is maintained by **Dr. Allen Y. Yang** (Hitch Interactive · University of California, Berkeley).

Implementation testing and field validation were carried out by the **Berkeley AI Racing Tech** team: from UC Berkeley (alphabetical by last name) — Bryan Chang, Logan Kinajil-Moran, Moises Lopez Mendoza, Gary Passon, Tanishaa Viral Shah, Joshua Sun, Di Tian, Jovan Yap; from UC San Diego — Kevin Shin.

Please cite or credit this repository when reusing any of the mechanical design, the ROS 2 TF configuration, the PTP synchronization pipeline, or the recording / visualization tooling in derivative work:

> Yang, A. Y. *Hitch Sensor Dome: a 3D-printable modular multi-sensor mount for vehicle-roof mapping.* GitHub repository, 2026.

Thanks to the OpenSCAD, ROS 2, linuxptp, chrony, Aravis, Foxglove, and MCAP communities whose open-source tooling this project builds on. The mapping pipeline is built on **GLIM** by Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno (AIST), and the localization backend on **small_gicp** by the same author — see [`GLIM_plusplus/README.md`](GLIM_plusplus/README.md) and [`GICP_plusplus/README.md`](GICP_plusplus/README.md) for full upstream attribution, license preservation, and citation.

## License

See [LICENSE](LICENSE) for details.
