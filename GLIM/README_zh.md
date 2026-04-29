# GLIM —— Hitch Sensor Dome 集成

> **本仓库为 fork 派生版本。** GLIM (Graph-based LiDAR-Inertial Mapping) 是由日本国立先进工业科学技术研究所 (AIST) 的 **Kenji Koide、Masashi Yokozuka、Shuji Oishi、Atsuhiko Banno** 等人开发的开源 SLAM 框架。算法和实现的全部功劳归原作者所有。本 fork 将该框架适配到 [Hitch Sensor Dome](../README.md) 硬件及其配套的时间同步、数据采集与可视化工具链。详见下方 [Credits](#credits)、[License](#license)、[Citation](#citation)。
>
> 上游仓库：<https://github.com/koide3/glim>
> 论文：[Koide et al., *GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors*, RA-L 2024](https://staff.aist.go.jp/k.koide/assets/pdf/koide2024ral.pdf)
>
> 本 fork 几乎完整保留了上游 `glim`、`glim_ext`、`glim_ros2` 的源码。集成相关的内容集中在两个新增的顶层目录 (`config/`、`launch/`) 以及 `glim/config/` 与 `glim_ext/config/` 中针对调参的 JSON 修改。

## 本 fork 与上游的差异

按重要性排列的三类变更：

1. **Hitch Sensor Dome 接线整合。** 所有传感器 topic、frame ID 和外参都已与项目其它部分对齐。Topic 为 `/imu/data`、`/robin_w_*/points`、`/cam_*/image_raw`、`/gps/fix`（与 [`recording/sensor_config.yaml`](../recording/sensor_config.yaml) 一致）。Frame 为 `imu_link`、`lidar_*_link`、`cam_*_link`（与 [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) 一致）。LiDAR–IMU 外参由生成的 URDF 提供，不再使用硬编码矩阵。

2. **车辆无关的全局建图 (vehicle-agnostic global mapping)。** `base_frame_id` 固定为 `imu_link` (Atlas Duo 的导航中心 CoN)，而不是留空或设为某个特定车辆的 `base_link`。本 fork 构建的地图相对于传感器穹顶 (sensor rig)，而非具体的载具，因此同一张地图可以通过下游再发布一个 `imu_link → base_link` 静态变换的方式在多个车辆平台间复用。

3. **面向户外感知与定位数据采集的整体调参。** 放宽 IMU 噪声、重新启用畸变补偿 (`global_shutter_lidar=false`)、用 `prior_inf_scale=[1e4, 1e4, 5e4]` 重新激活 GNSS 先验（上游默认 `[0,0,0]`，相当于禁用 GNSS）、Atlas Duo 的 `acc_scale=1.0`、面向 3 传感器拼接点云的更大下采样目标 (20 000) 与 `k_correspondences=20`、面向车辆尺度户外建图的 voxel 分辨率，默认启用 IMU validator 用于装机调试。所有 24 项变更都在各 `config_*.json` 中以注释形式逐项说明。

继承自此前 `airacingtech` fork 沿袭的 monorepo 布局（一个仓库，三个包 — `glim`、`glim_ext`、`glim_ros2`）。

## 快速开始

为您的安装编辑一次 [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml)，然后：

```bash
# (一次性) 从 sensor_dome_tf.yaml 生成 sensor_dome.urdf。
cd GLIM/config && python3 generate_sensor_dome_urdf.py

# 编译 glim、glim_ext、glim_ros2 (见下文 Building 一节)。
# 然后用项目的静态 TF + foxglove_bridge 启动 GLIM：
ros2 launch GLIM/launch/hitch_sensor_dome.launch.py
```

实时模式直接订阅 recorder 的 topic；离线模式重放由 [`recording/sensor_recorder.py`](../recording/sensor_recorder.py) 采集的 `.mcap` 包：

```bash
# 离线重放 recording/data/session_*/ 中的 MCAP 包
ros2 run glim_ros glim_rosbag recording/data/session_<ts>/rosbag2 \
    --ros-args -p config_path:=GLIM/glim/config \
                -p dump_path:=glim_maps/session_<ts>
```

## Hitch Sensor Dome 集成细节

### 外参的单一信息源

每一个 `imu_link → sensor_link` 静态变换都存放在 [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) 中（3 个 LiDAR + 4 个相机 = 7 个固定变换，因为水平安装平台的关系，全部都是 yaw 旋转）。[`GLIM/config/generate_sensor_dome_urdf.py`](config/generate_sensor_dome_urdf.py) 把该 YAML 一次性转换成 `sensor_dome.urdf`，GLIM 通过 `config_sensors.json` 中的 `urdf_path` 字段读取它（既用于 `T_lidar_imu`，也用于 `lidar_concat` 多 LiDAR 拼接）。每次修改 TF YAML 后重新运行此生成器。

启动文件还会通过 `tf2_ros::static_transform_publisher` 发布同一份 YAML，使运行期 TF 树、GLIM 使用的 URDF 以及项目其它部分都引用同一组数值。

### 车辆无关的全局建图

`config_ros.json` 中 `base_frame_id = "imu_link"`。地图相对于 Atlas Duo 导航中心 (CoN) 在传感器主体坐标系中构建。每个下游车辆集成商再发布自己的静态变换，把 `imu_link` 映射到所偏好的 `base_link`（车轴中点、车载 IMU 原点、运动学中心等）——而无需重建地图。这样同一份采集数据和同一张 GLIM 地图就可以在不同车辆平台上重用。

### Topic 命名与采集栈一致

| GLIM 输入 | Topic | 来源 |
|------------|-------|--------|
| IMU | `/imu/data` | `fusion_engine_driver` (Atlas Duo, 200 Hz) |
| LiDAR primary | `/robin_w_front/points` | `seyond_ros_driver`, `coordinate_mode:=3` |
| LiDAR aux | `/robin_w_rear_left/points`、`/robin_w_rear_right/points` | 由 `lidar_concat` 合并 |
| 相机 | `/cam_front_left/image_raw` | `camera_aravis2` (RouteCAM_P_CU25) |
| GNSS | `/gps/fix` | `fusion_engine_driver` (NavSatFix, 10 Hz) |

这些是 [`recording/sensor_config.yaml`](../recording/sensor_config.yaml) 实际发布的同一组 topic，因此 GLIM 既可以在线订阅实时驱动，也可以离线回放 `.mcap` 包，全部不需要 remap。

### 调参出处

本 fork JSON 配置中的 24 项参数变更在文件内逐项注释（每个非默认值都附有解释其原因的注释）。调参由一次基于 Claude 的回顾整理而成，汇总文件 [`uploads/GLIM_config.yaml`](../recording/data/) 已随项目保存以备查询。涉及的层面：

- IMU 噪声模型与 bias 随机游走 —— 放宽以更倚重 LiDAR
- GNSS prior_inf_scale —— 重新启用以修正多圈 z 漂移
- 重新启用畸变补偿 (deskewing)，因为上游多 LiDAR 时间戳重基 (timestamp rebasing) 的 bug 已修复
- 提高 voxel 分辨率以适配车辆尺度的户外建图
- 初始化窗口扩大三倍以获得更干净的重力估计

### 多圈建图与回环

重复轨迹数据 —— 赛道、固定路线的车队、停车场测绘 —— 会暴露出 GLIM *隐式*回环 (implicit loop closure) 的 chicken-and-egg 失败模式：第 1 圈累积少量 z 方向漂移（LiDAR 在 z 方向结构性偏弱，IMU 加速度 z 轴 bias 也会贡献），等到第 2 圈在全局图中开始与第 1 圈重叠时，对应的 submap 在估计坐标系下已经相距好几米 —— 已经超出 VGICP 的收敛域。本应起作用的回环因子要么根本不被创建（重叠率低于阈值），要么吸附到错误的对应关系。第 2 圈于是继续向上漂移。

本 fork 针对该病灶应用三层修复：

1. **更强的 GNSS z 锚点** —— `prior_inf_scale = [1e4, 1e4, 5e4]` (z 比水平方向强 5 倍)，`min_baseline = 0.5` m (GNSS 因子密度翻倍)。Atlas Duo + RTK 提供约 1 cm 垂直精度，因此提升 z 权重是合理的，且能从源头阻止第 1 圈漂移的累积。
2. **更宽的 VGICP 收敛域** —— `submap_voxel_resolution_max = 2.0` m (原 1.0)。多分辨率 voxel 阶梯顶层翻倍，使 submap 之间的配准在初始位姿 z 方向有 ~1 m 误差时仍能收敛。基础 voxel 仍为 0.3 m，地图精度不变。
3. **更宽松的隐式因子触发阈值** —— `max_implicit_loop_distance = 200` m (覆盖典型赛道一圈) 和 `min_implicit_loop_overlap = 0.1` (从 0.2 降下来)，使部分重叠也能创建因子。强化后的 GNSS 锚点正是让这些更宽松阈值变得安全的关键 —— 错误回环会被全局锚点截断，不至于污染地图。

完整诊断、四项运行后验证 (GNSS 因子数、可视化器中可见的位姿图边、`T_world_utm.txt` 是否写入并稳定、轨迹高度 vs GNSS 高度) 以及五步升级方案 (在默认修复仍不足时使用) 详见 [`docs/multi_lap_loop_closure.md`](docs/multi_lap_loop_closure.md)。

## 仓库结构

```
GLIM/
├── config/                              # 仅 Hitch Sensor Dome 集成
│   ├── generate_sensor_dome_urdf.py     # YAML → URDF 转换器
│   └── sensor_dome.urdf                 # 自动生成，请勿手工编辑
├── launch/
│   └── hitch_sensor_dome.launch.py      # 静态 TF + glim_rosnode + foxglove
├── docs/
│   └── multi_lap_loop_closure.md        # 多圈 z 漂移调试指南
├── glim/                                # 上游核心包 —— 仅做 JSON 调参
│   ├── config/                          # JSON 配置文件 (全部针对本套硬件调过)
│   ├── include/
│   └── src/
├── glim_ext/                            # 上游扩展包
│   ├── config/
│   │   └── config_gnss_global.json      # /gps/fix, prior_inf_scale 已激活
│   └── modules/
└── glim_ros2/                           # 上游 ROS 2 封装，C++ 未改动
    ├── launch/
    └── src/
```

## 依赖

### 系统

- Ubuntu 22.04（推荐）或 24.04
- ROS 2 Humble 或 Jazzy
- CUDA 11.8+（可选，用于 GPU VGICP）

### apt 包

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
cmake .. -DBUILD_WITH_CUDA=ON          # 无 GPU 时设为 OFF
make -j$(nproc) && sudo make install
```

### Iridescence (可选，原生可视化器)

```bash
git clone https://github.com/koide3/iridescence.git
cd iridescence && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

## 编译

```bash
cd ~/ros2_ws/src
git clone <本 fork URL> .             # 也可以把 Hitch_Sensor_Dome/GLIM 软链到这里
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

仅 CPU 编译：在 gtsam_points 步骤里去掉 `-DBUILD_WITH_CUDA=ON`。GLIM 会自动改用 CPU VGICP 路径。

## 地图输出

`glim_rosbag` 把输出目录写到 `dump_path` 参数 (默认 `/tmp/dump`)：

```
glim_maps/session_<ts>/
├── graph.txt / graph.bin     # 位姿图
├── 000000/, 000001/ ...      # submap 目录（含点云）
├── odom_lidar.txt            # 原始 LiDAR 里程计
├── odom_imu.txt              # IMU 速率里程计
├── traj_lidar.txt            # 全局优化后的 LiDAR 轨迹
├── traj_imu.txt              # 全局优化后的 IMU 轨迹
├── T_world_utm.txt           # 4×4 SE(3): map → UTM (仅在 GNSS 对齐时写入)
└── config/                   # 本次建图所用配置的快照
```

`T_world_utm.txt` 是把地图与 WGS-84/UTM 关联起来的 SE(3) 句柄。本 fork 启用了 `prior_inf_scale = [1e4, 1e4, 5e4]`，GNSS 因子在整段运行中持续作用，只要 GNSS 锁定时间长于 `min_baseline = 0.5 m` 的运动距离，`T_world_utm` 就会被可靠写入。

## 故障排除

**GNSS 没有对齐。** 确认 `/gps/fix` 在发布 (`ros2 topic echo /gps/fix`)。检查 Atlas Duo 是否有 3D 锁定 (或者您配置了 NTRIP 时的 RTK 锁定)。确认车辆移动距离已超过 `min_baseline` (默认 0.5 m)。如果 `T_world_utm.txt` 始终没被写入，查 `[gnss_global] insert N GNSS prior factors` 这一行 —— N 应该随时间增长。

**三个 LiDAR 的点云都看得到，但旋转了 90° 或对不齐。** seyond 驱动没有以 `coordinate_mode:=3` 运行。见 [`recording/sensor_config.yaml`](../recording/sensor_config.yaml) —— 这个标志才能产生静态 TF 所假设的 REP-103 坐标轴 (X-fwd, Y-left, Z-up)。

**`urdf_path: ../../config/sensor_dome.urdf` 找不到。** GLIM 解析 config 相关路径时以 `config_ros.json` 所在位置为基准。请使用启动文件 (`hitch_sensor_dome.launch.py`)，它会显式设置 `config_path`；或者直接传一个绝对路径。

**地图为空 / 没保存。** 用 `tee` 替代 `grep` 接管输出，让 SIGINT 在退出过程中能传到 GLIM：`ros2 launch ... | tee output.log`。检查 `dump_path` 的写权限。

**性能问题 / 丢帧。** 调低 `random_downsample_target` (本 fork 默认 20 000) 和 `config_preprocess.json` 中的 `num_threads`。无头运行时禁用原生 `librviz_viewer.so`。

**CUDA 错误。** 用 `-DBUILD_WITH_CUDA=OFF` 重新编译 gtsam_points。系统会自动改用 CPU VGICP 路径。

## Credits

本 fork 站在他人工作之上构建。引用或基于本仓库构建时请同时致谢：

- **GLIM** —— Kenji Koide、Masashi Yokozuka、Shuji Oishi、Atsuhiko Banno (AIST)。仓库：<https://github.com/koide3/glim>。论文：[RA-L 2024](https://staff.aist.go.jp/k.koide/assets/pdf/koide2024ral.pdf)。
- **gtsam_points** —— Kenji Koide。仓库：<https://github.com/koide3/gtsam_points>。
- **GTSAM** —— Frank Dellaert 与 Georgia Tech Borg Lab。仓库：<https://github.com/borglab/gtsam>。
- **Iridescence** —— Kenji Koide (可视化库)。仓库：<https://github.com/koide3/iridescence>。

本仓库中的集成工作 (`config/` URDF 生成器、`launch/` 启动助手、`glim/config/` 与 `glim_ext/config/` 中的 JSON 调参) 是 **Hitch Sensor Dome** 项目的一部分，由 Dr. Allen Y. Yang (Hitch Interactive · 加州大学伯克利分校) 设计和维护。

## License

本 fork 继承每一个组成包的许可证。**任何上游许可证文本都没有被修改。**

- **GLIM** —— MIT License (Kenji Koide / AIST)
- **gtsam_points** —— MIT License
- **GTSAM** —— BSD License
- **Iridescence** —— MIT License
- **Hitch Sensor Dome 集成代码** (`config/`、`launch/`、JSON 调参) —— 见项目根目录 [`../LICENSE`](../LICENSE)。

每个包目录 (`glim/`、`glim_ext/`、`glim_ros2/`) 内部完整保留了上游许可证原文。

## Citation

如果在学术工作中使用本 fork，请引用上游 GLIM 论文：

```bibtex
@article{koide2024glim,
  title   = {GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors},
  author  = {Koide, Kenji and Yokozuka, Masashi and Oishi, Shuji and Banno, Atsuhiko},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2024}
}
```

如果本仓库的集成工具 (URDF 生成器、启动助手、项目级调参) 对您特别有用，可附加致谢相应项目：

```bibtex
@misc{yang2026hitchsensordome,
  title  = {Hitch Sensor Dome: a 3D-printable modular multi-sensor mount for vehicle-roof mapping},
  author = {Yang, Allen Y.},
  year   = {2026},
  note   = {GitHub repository}
}
```
