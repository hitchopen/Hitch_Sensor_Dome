# GLIM — Hitch Sensor Dome integration

> **This is a fork.** GLIM (Graph-based LiDAR-Inertial Mapping) is an open-source SLAM framework authored by **Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno** at AIST (National Institute of Advanced Industrial Science and Technology, Japan). All credit for the algorithmic and implementation work belongs to the upstream authors. This fork adapts the framework to the [Hitch Sensor Dome](../README.md) hardware and its accompanying time-sync, recording, and visualization tooling. See [Credits](#credits), [License](#license), and [Citation](#citation) below.
>
> Upstream repository: <https://github.com/koide3/glim>
> Paper: [Koide et al., *GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors*, RA-L 2024](https://staff.aist.go.jp/k.koide/assets/pdf/koide2024ral.pdf)
>
> This fork preserves the upstream `glim`, `glim_ext`, and `glim_ros2` package source largely untouched. The integration sits in two new top-level folders (`config/`, `launch/`) and in tuning-only edits to the JSON files under `glim/config/` and `glim_ext/config/`.

## What's different in this fork

Three categories of change, in order of significance:

1. **Hitch Sensor Dome wiring.** All sensor topics, frame IDs, and extrinsics now match the rest of the project. Topics are `/imu/data`, `/robin_w_*/points`, `/cam_*/image_raw`, `/gps/fix` (matching [`recording/sensor_config.yaml`](../recording/sensor_config.yaml)). Frames are `imu_link`, `lidar_*_link`, `cam_*_link` (matching [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml)). Lidar–IMU extrinsics are derived from a generated URDF rather than hard-coded matrices.

2. **Vehicle-agnostic global mapping.** `base_frame_id` is pinned to `imu_link` (the Atlas Duo Center of Navigation) instead of being left blank or set to a vehicle-specific `base_link`. Maps built by this fork are anchored to the sensor rig, not to the host vehicle, so a single map can be reused across vehicles by adding only one downstream `imu_link → base_link` static transform per platform.

3. **Tuning pass for outdoor perception/localization data collection.** IMU noise loosened, deskewing re-enabled (`global_shutter_lidar=false`), GNSS prior reactivated with `prior_inf_scale=[1e4, 1e4, 1e4]` (was `[0,0,0]` upstream — silently disabled GNSS), `acc_scale=1.0` for the Atlas Duo, larger downsample target (20 000) and `k_correspondences=20` for the 3-sensor stitched cloud, voxel resolutions set for vehicle-scale outdoor mapping, IMU validator enabled by default for bring-up. The 24 individual changes are itemized inline as comments in each `config_*.json` file.

A monorepo layout (one repo, three packages — `glim`, `glim_ext`, `glim_ros2`) is also retained from the prior `airacingtech` fork lineage.

## Quick start

Edit [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) once for your installation, then:

```bash
# (one-time) generate sensor_dome.urdf from sensor_dome_tf.yaml.
cd GLIM/config && python3 generate_sensor_dome_urdf.py

# build glim, glim_ext, glim_ros2 (see Building below).
# then launch GLIM with the project's static TFs + foxglove_bridge:
ros2 launch GLIM/launch/hitch_sensor_dome.launch.py
```

Live mode subscribes directly to the recorder's topics; offline mode replays an `.mcap` bag captured by [`recording/sensor_recorder.py`](../recording/sensor_recorder.py):

```bash
# offline against an MCAP bag from recording/data/session_*/
ros2 run glim_ros glim_rosbag recording/data/session_<ts>/rosbag2 \
    --ros-args -p config_path:=GLIM/glim/config \
                -p dump_path:=glim_maps/session_<ts>
```

## Hitch Sensor Dome integration details

### Single source of truth for extrinsics

Every static `imu_link → sensor_link` transform lives in [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) (3 lidars + 4 cameras = 7 fixed transforms, all yaw-only on a horizontal mounting plate). [`GLIM/config/generate_sensor_dome_urdf.py`](config/generate_sensor_dome_urdf.py) converts that YAML into a one-shot `sensor_dome.urdf` that GLIM consumes via the `urdf_path` field in `config_sensors.json` (both for `T_lidar_imu` and for the `lidar_concat` multi-LiDAR stitching). Re-run the generator any time the TF YAML changes.

The launch helper publishes the same YAML through `tf2_ros::static_transform_publisher` so the live TF tree, the URDF GLIM uses, and the rest of the project all reference the same numbers.

### Vehicle-agnostic global mapping

`base_frame_id = "imu_link"` in `config_ros.json`. The map is built body-relative to the Atlas Duo Center of Navigation. Each downstream vehicle integrator publishes its own static transform from `imu_link` to its preferred `base_link` (axle midpoint, IMU-on-vehicle origin, kinematic center, etc.) without forcing the map to be rebuilt. This lets the same recorded session and the same GLIM map be reused on different vehicle platforms.

### Topic conventions match the recording stack

| GLIM input | Topic | Source |
|------------|-------|--------|
| IMU | `/imu/data` | `fusion_engine_driver` (Atlas Duo, 200 Hz) |
| LiDAR primary | `/robin_w_front/points` | `seyond_ros_driver`, `coordinate_mode:=3` |
| LiDAR aux | `/robin_w_rear_left/points`, `/robin_w_rear_right/points` | merged by `lidar_concat` |
| Camera | `/cam_front_left/image_raw` | `camera_aravis2` (RouteCAM_P_CU25) |
| GNSS | `/gps/fix` | `fusion_engine_driver` (NavSatFix, 10 Hz) |

These are the same names [`recording/sensor_config.yaml`](../recording/sensor_config.yaml) advertises, so GLIM works against either live drivers or replayed `.mcap` bags without remapping.

### Tuning provenance

The 24 parameter changes in this fork's JSON configs are documented inline (every non-default value carries a comment explaining the rationale). The tuning was derived from a Claude-assisted review and is summarized in [`uploads/GLIM_config.yaml`](../recording/data/) (kept with the project for reference). Areas that received attention:

- IMU noise model and bias random walk — loosened to weight LiDAR more
- GNSS prior_inf_scale — re-enabled to fix multi-lap z-drift
- Deskewing re-enabled now that the upstream multi-LiDAR timestamp rebasing bug is fixed
- Voxel resolutions raised for vehicle-scale outdoor mapping
- Initialization window tripled for cleaner gravity estimation

### Multi-lap mapping & loop closure

Repeated-trajectory data — race tracks, fleet vehicles on fixed routes, surveying a parking lot — exposes a chicken-and-egg failure mode in GLIM's *implicit* loop closure: lap 1 accumulates a small amount of z-drift (LiDAR is structurally weak in z and IMU acc-z bias contributes), and by the time lap 2 starts overlapping lap 1 in the global graph, the submap pairs are already several metres apart in the estimated frame — outside VGICP's convergence basin. The would-be closure factor either never fires (overlap drops below threshold) or locks onto false correspondences. Lap 2 then continues to drift upward.

This fork applies a three-layer fix targeting that exact pathology:

1. **Stronger GNSS z-anchor** — `prior_inf_scale = [1e4, 1e4, 5e4]` (z 5× stronger than horizontal) and `min_baseline = 0.5` m (twice the GNSS factor density). Atlas Duo + RTK delivers ~1 cm vertical, so the elevated z weight is justified and prevents lap 1 from drifting in the first place.
2. **Wider VGICP convergence basin** — `submap_voxel_resolution_max = 2.0` m (was 1.0). The top of the multi-resolution voxel ladder grows so submap-to-submap registration tolerates ~1 m of initial pose error in z. The 0.3 m base voxel is unchanged, so map fidelity isn't sacrificed.
3. **Looser proximity thresholds for implicit factor creation** — `max_implicit_loop_distance = 200` m (covers a typical race-track lap) and `min_implicit_loop_overlap = 0.1` (down from 0.2) so partial overlaps still create factors. The strengthened GNSS anchor is what makes these looser thresholds safe; false closures are caught by the global anchor before they corrupt the map.

See [`docs/multi_lap_loop_closure.md`](docs/multi_lap_loop_closure.md) for the full diagnosis, the four post-run verification checks (GNSS factor count, pose-graph edges visible in the viewer, `T_world_utm.txt` written and stable, trajectory altitude vs GNSS altitude), and a five-step escalation if the default fix isn't sufficient.

## Repository structure

```
GLIM/
├── config/                              # Hitch Sensor Dome integration only
│   ├── generate_sensor_dome_urdf.py     # YAML → URDF converter
│   └── sensor_dome.urdf                 # generated, do not edit by hand
├── launch/
│   └── hitch_sensor_dome.launch.py      # static TFs + glim_rosnode + foxglove
├── docs/
│   └── multi_lap_loop_closure.md        # multi-lap z-drift debugging guide
├── glim/                                # upstream package — minor JSON tuning only
│   ├── config/                          # JSON configs (all tuned for this rig)
│   ├── include/
│   └── src/
├── glim_ext/                            # upstream extensions package
│   ├── config/
│   │   └── config_gnss_global.json      # /gps/fix, prior_inf_scale enabled
│   └── modules/
└── glim_ros2/                           # upstream ROS 2 wrapper, unmodified C++
    ├── launch/
    └── src/
```

## Dependencies

### System

- Ubuntu 22.04 (recommended) or 24.04
- ROS 2 Humble or Jazzy
- CUDA 11.8+ (optional, for GPU VGICP)

### apt packages

```bash
sudo apt update
sudo apt install -y \
  libeigen3-dev \
  libboost-all-dev \
  libfmt-dev \
  libomp-dev \
  libmetis-dev \
  ros-${ROS_DISTRO}-tf2-eigen \
  ros-${ROS_DISTRO}-pcl-ros \
  ros-${ROS_DISTRO}-foxglove-bridge
```

### GTSAM

```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam && mkdir build && cd build
cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc) && sudo make install
```

### gtsam_points (Koide)

```bash
git clone https://github.com/koide3/gtsam_points.git
cd gtsam_points && mkdir build && cd build
cmake .. -DBUILD_WITH_CUDA=ON          # OFF if no GPU
make -j$(nproc) && sudo make install
```

### Iridescence (optional, native viewer)

```bash
git clone https://github.com/koide3/iridescence.git
cd iridescence && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

## Building

```bash
cd ~/ros2_ws/src
git clone <this fork URL> .          # or symlink Hitch_Sensor_Dome/GLIM here
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

CPU-only build: drop `-DBUILD_WITH_CUDA=ON` from the gtsam_points step. GLIM will use the CPU VGICP path automatically.

## Map output

`glim_rosbag` writes its output directory to the `dump_path` parameter (default `/tmp/dump`):

```
glim_maps/session_<ts>/
├── graph.txt / graph.bin     # pose graph
├── 000000/, 000001/ ...      # submap directories with point clouds
├── odom_lidar.txt            # raw LiDAR odometry
├── odom_imu.txt              # IMU-rate odometry
├── traj_lidar.txt            # globally-optimized lidar trajectory
├── traj_imu.txt              # globally-optimized IMU trajectory
├── T_world_utm.txt           # 4×4 SE(3): map → UTM (only if GNSS aligned)
└── config/                   # snapshot of the configs used for this map
```

`T_world_utm.txt` is the SE(3) handle that pins the map to WGS-84/UTM. With `prior_inf_scale = [1e4, 1e4, 1e4]` enabled in this fork, the GNSS factor stays active throughout the run and `T_world_utm` is written reliably as long as GNSS lock is held longer than `min_baseline = 1.0 m` of motion.

## Troubleshooting

**GNSS not aligning.** Confirm `/gps/fix` is publishing (`ros2 topic echo /gps/fix`). Verify the Atlas Duo has a 3D fix (or RTK fix if you've configured NTRIP). Confirm the vehicle has moved further than `min_baseline` (default 1 m). If `T_world_utm.txt` is never written, check the log for `[gnss_global] insert N GNSS prior factors` — N should grow over time.

**Three lidar clouds visible but rotated 90° / not aligned.** The seyond driver is not running with `coordinate_mode:=3`. See [`recording/sensor_config.yaml`](../recording/sensor_config.yaml) — that flag is what produces the REP-103 axes (X-fwd, Y-left, Z-up) the static TFs assume.

**`urdf_path: ../../config/sensor_dome.urdf` not found.** GLIM resolves config-relative paths from the location of `config_ros.json`. Use the launch file (`hitch_sensor_dome.launch.py`) which sets `config_path` explicitly, or pass an absolute path on the command line.

**Map empty / not saving.** Pipe through `tee` rather than `grep` so SIGINT reaches GLIM during shutdown: `ros2 launch ... | tee output.log`. Check write permission on `dump_path`.

**Performance / dropped frames.** Lower `random_downsample_target` (default 20 000 in this fork) and `num_threads` in `config_preprocess.json`. Disable the native `librviz_viewer.so` if running headless.

**CUDA errors.** Rebuild gtsam_points with `-DBUILD_WITH_CUDA=OFF`. The CPU VGICP path will be used automatically.

## Credits

This fork is built on the work of others. Please credit them when citing or building on it.

- **GLIM** — Kenji Koide, Masashi Yokozuka, Shuji Oishi, Atsuhiko Banno (AIST). Repository: <https://github.com/koide3/glim>. Paper: [RA-L 2024](https://staff.aist.go.jp/k.koide/assets/pdf/koide2024ral.pdf).
- **gtsam_points** — Kenji Koide. Repository: <https://github.com/koide3/gtsam_points>.
- **GTSAM** — Frank Dellaert and the Georgia Tech Borg Lab. Repository: <https://github.com/borglab/gtsam>.
- **Iridescence** — Kenji Koide (visualization library). Repository: <https://github.com/koide3/iridescence>.

The integration work in this repository (the `config/` URDF generator, the `launch/` helper, and the JSON tuning under `glim/config/` and `glim_ext/config/`) is part of the **Hitch Sensor Dome** project, designed and maintained by Dr. Allen Y. Yang (Hitch Interactive · University of California, Berkeley).

## License

This fork inherits the license of every constituent package. **No license text from the upstream packages has been altered.**

- **GLIM** — MIT License (Kenji Koide / AIST)
- **gtsam_points** — MIT License
- **GTSAM** — BSD License
- **Iridescence** — MIT License
- **Hitch Sensor Dome integration code** (`config/`, `launch/`, JSON edits) — see [`../LICENSE`](../LICENSE) at the project root.

The full upstream license texts are preserved inside each package directory (`glim/`, `glim_ext/`, `glim_ros2/`).

## Citation

If you use this fork in academic work, please cite the upstream GLIM paper:

```bibtex
@article{koide2024glim,
  title   = {GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors},
  author  = {Koide, Kenji and Yokozuka, Masashi and Oishi, Shuji and Banno, Atsuhiko},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2024}
}
```

If the integration tooling in this repository (the URDF generator, the launch helper, the project-level tuning) was specifically useful, you may additionally credit the surrounding project:

```bibtex
@misc{yang2026hitchsensordome,
  title  = {Hitch Sensor Dome: a 3D-printable modular multi-sensor mount for vehicle-roof mapping},
  author = {Yang, Allen Y.},
  year   = {2026},
  note   = {GitHub repository}
}
```
