# GLIM++ —— 面向 Hitch Sensor Dome 的深度改造 GLIM fork

> **这不是原版 GLIM。** 文件夹名称特意采用 `GLIM_plusplus/`：本版本在算法行为层面（不仅是配置）已与上游 [`koide3/glim`](https://github.com/koide3/glim) 出现差异。如果您是来寻找原版 GLIM 的，原版仓库地址为 <https://github.com/koide3/glim>，并强烈建议从那里开始 —— 除非您拥有 [Hitch Sensor Dome](../README.md) 硬件。GLIM 算法与实现的全部功劳归 **Kenji Koide、Masashi Yokozuka、Shuji Oishi、Atsuhiko Banno（AIST）** 所有。详见 [Credits](#credits)、[License](#license)、[Citation](#citation)。

本文档是 GLIM++ 与上游 `koide3/glim` 之间的**完整变更日志**。结构按机制分节，方便采用者逐项判断哪些改动适合自己的使用场景，哪些需要回退。

## 变更索引

1. [面向 Hitch Sensor Dome 的传感器适配](#1-面向-hitch-sensor-dome-的传感器适配)
2. [车辆无关主体坐标系（`base_frame_id = imu_link`）](#2-车辆无关主体坐标系)
3. [面向户外 / 车辆尺度的整体调参（24 项参数变更）](#3-面向户外--车辆尺度的整体调参)
4. [多圈回环修复](#4-多圈回环修复)
5. [初始化重写 —— INS 驱动，移除"由加速度计估计重力"](#5-初始化重写)
6. [初始位姿的 RTK-fixed 准入门控](#6-初始位姿的-rtk-fixed-准入门控)
7. [RTK 门控的 GNSS 因子桥（init 之后）](#7-rtk-门控的-gnss-因子桥)
8. [项目集成 —— URDF 生成器、启动助手、诊断脚本](#8-项目集成)
9. [新增文档](#9-新增文档)
10. [未改动的部分](#10-未改动的部分)
11. [按文件汇总的差异](#11-按文件汇总的差异)

## 1. 面向 Hitch Sensor Dome 的传感器适配

将 topic、frame、字段名从此前的 AV-24 部署（Luminar Iris + 自定义 IMU）切换到 Hitch Sensor Dome 参考配置（3× Seyond Robin W + Point One Atlas Duo + 4× e-con RouteCAM）。

| 项目 | 原 | 现 |
|------|-----|-----|
| IMU topic | `/gps_bot/imu` | `/imu/data` |
| 主 LiDAR | `/luminar_front/points` | `/robin_w_front/points` |
| 辅 LiDAR | `/luminar_left/points`、`/luminar_right/points` | `/robin_w_rear_left/points`、`/robin_w_rear_right/points` |
| GNSS | `/gps_nav/odom` | `/gps/fix`（NavSatFix 门控信号） |
| 摄像头 | `/cam_front_left/image_raw` |（同名保留） |
| `intensity_field` | `reflectance` (Luminar) | `intensity` (Robin W 默认) |
| `ring_field` | `line_index` (Luminar) | `ring` (Robin W 默认) |
| `flip_points_y` | `true` (Luminar SAE Y-right) | `false` (Robin W 设 `coordinate_mode:=3` 后已是 REP-103) |
| LiDAR–IMU 外参来源 | 手工编辑的 7 维 TUM 数组 | 由 [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) 生成的 URDF —— 见 §8 |

涉及文件：`glim/config/config_sensors.json`、`glim/config/config_ros.json`。

## 2. 车辆无关主体坐标系

[`glim/config/config_ros.json`](glim/config/config_ros.json) 中：

```json
"imu_frame_id":  "imu_link",
"lidar_frame_id":"lidar_front_link",
"base_frame_id": "imu_link"      // 原值: "" (auto-detect)
```

将 `base_frame_id` 固定为 `imu_link`，使 GLIM 围绕 Atlas Duo 导航中心（CoN）建图，而不是某个具体车辆的 `base_link`。各下游车辆集成商再发布自己的静态 `imu_link → base_link` —— 这样建好的地图无需重跑 SLAM 即可在不同车辆平台上复用。

这是相对上游的有意分歧：上游默认 `base_frame_id` 为空，会沿用 IMU frame 的名字（在 AV-24 上是 `gps_bot`，与车辆耦合）。

## 3. 面向户外 / 车辆尺度的整体调参

七个 JSON 配置中共 24 项参数变更，每项在文件内附注释。按主题分组：

| 维度 | 文件 | 方向 |
|------|-------|-----------|
| **IMU 噪声 / bias** | `config_sensors.json` | 放宽：`acc 0.05 → 0.2`、`gyro 0.02 → 0.05`、`bias 1e-5 → 1e-4`。降低 IMU 信任、提高 LiDAR 权重。 |
| **加速度计标度** | `config_ros.json` | `acc_scale: 0.0 → 1.0`。Atlas Duo 原生输出 m/s²，无需自动检测。 |
| **初始化** | `config_odometry_gpu.json` | 窗口 `1.0 → 3.0` s（后已被完全替换 —— 见 §5）。 |
| **GNSS 先验** | `config_gnss_global.json` | `prior_inf_scale: [0,0,0] → [1e4, 1e4, 5e4]`（上游默认值相当于禁用 GNSS）。`min_baseline: 1.0 → 0.5`。 |
| **运动畸变补偿** | `config_sensors.json` | `global_shutter_lidar: true → false`。重新启用畸变补偿（上游多 LiDAR 时间戳重基 bug 已被修复，无需再绕开）。 |
| **每点时间戳** | `config_sensors.json` | `autoconf_perpoint_times: false → true`、`perpoint_relative_time: false → true`。Robin W 驱动输出相对帧时间戳。 |
| **下采样** | `config_preprocess.json` | `random_downsample_target: 10000 → 20000`、`k_correspondences: 10 → 20`。3 路传感器拼成的 360° 点云需更高密度。 |
| **VGICP voxel** | `config_odometry_gpu.json`、`config_sub_mapping_gpu.json` | 户外尺度提高基础分辨率：`voxel_resolution 0.25 → 0.5`、`voxel_resolution_max 0.5 → 1.0`。submap 内 `keyframe_voxel_resolution 0.25 → 0.15` 更紧。 |
| **smoother 窗口** | `config_odometry_gpu.json` | `full_connection_window_size: 2 → 4`。适应车辆剧烈机动。 |
| **submap 管理** | `config_sub_mapping_gpu.json` | `max_num_keyframes: 15 → 20`。submap 覆盖更全。 |
| **扩展模块** | `config_ros.json` | 启用 `libimu_validator.so` 用于装机调试。 |

涉及文件：上表中列出的全部 `glim/config/*.json` 与 `glim_ext/config/*.json`。

## 4. 多圈回环修复

针对上游 GLIM 经典的"第二圈翘向天空"失败模式，在 `glim/config/config_global_mapping_gpu.json` 与 `glim_ext/config/config_gnss_global.json` 中做了三层修复：

| 参数 | 原值 | 现值 |
|------|-----|-----|
| `submap_voxel_resolution_max` | `1.0` m | `2.0` m（拓宽 VGICP 收敛域） |
| `max_implicit_loop_distance` | `100` m | `200` m（覆盖典型赛道一圈） |
| `min_implicit_loop_overlap` | `0.2` | `0.1`（部分重叠也能创建因子） |
| GNSS `prior_inf_scale[2]` (z) | `1e4` | `5e4`（z 比水平方向强 5×） |
| GNSS `min_baseline` | `1.0` m | `0.5` m（GNSS 因子密度翻倍） |

里程计初始化与 VGICP 收敛域之间的 chicken-and-egg 详见 [`docs/multi_lap_loop_closure.md`](docs/multi_lap_loop_closure.md)。前三项拓宽了收敛域，使在残余漂移下回环因子仍能触发；GNSS 两项则从源头阻止漂移累积。两层修复缺一不可。

## 5. 初始化重写

这是本 fork 改动最大的 C++ 部分。**上游 GLIM 通过最初 `initialization_window_size` 秒的加速度计均值来确定世界坐标系朝向**，前提是 IMU 在该窗口期间静止。当数据从车辆已开始运动时录起 —— 赛道、中途重启、被截掉静止段的 bag 重放 —— 这一前提就会被打破。线加速度被错误积分进"重力估计"，得到一个倾斜的世界坐标系，污染下游所有阶段。

GLIM++ 完全移除了"由加速度计估计重力"这条路径，并要求在优化器启动前提供来自外部 INS 的位姿。

### 5.1 SLAM 核心改动

| 文件 | 改动 |
|------|--------|
| [`glim/src/glim/odometry/initial_state_estimation.cpp`](glim/src/glim/odometry/initial_state_estimation.cpp) | `NaiveInitialStateEstimation::initial_pose()` 在 `force_init==true` 之前一律返回 `nullptr`。`acc_dir → T_world_imu` 的导出代码已移除。`insert_imu()` 不再累积 `sum_acc`。 |
| [`glim/src/glim/odometry/odometry_estimation_imu.cpp`](glim/src/glim/odometry/odometry_estimation_imu.cpp) | 构造函数始终实例化 `NaiveInitialStateEstimation`，不再走 `LooseInitialStateEstimation` 分支或基于 `estimate_init_state` 的分支选择。新增 public 方法 `OdometryEstimationIMU::set_init_state(T, v)`，通过 `dynamic_cast` 转给 Naive 实例。 |
| [`glim/include/glim/odometry/odometry_estimation_base.hpp`](glim/include/glim/odometry/odometry_estimation_base.hpp) | 新增虚方法 `set_init_state(T, v)`（默认 no-op），保证 .so 边界类型安全。 |
| [`glim/include/glim/odometry/async_odometry_estimation.hpp`](glim/include/glim/odometry/async_odometry_estimation.hpp) 与 [其 `.cpp`](glim/src/glim/odometry/async_odometry_estimation.cpp) | public `set_init_state(T, v)` 把值放进互斥锁保护的槽位；worker 线程在 `run()` 顶部消耗，从而保证估计器的 `set_init_state` 始终只在单线程调用，与 `insert_imu` / `insert_frame` 无竞态。幂等。 |

### 5.2 ROS wrapper 改动

| 文件 | 改动 |
|------|--------|
| [`glim_ros2/include/glim_ros/glim_ros.hpp`](glim_ros2/include/glim_ros/glim_ros.hpp) 与 [`.cpp`](glim_ros2/src/glim_ros/glim_ros.cpp) | 新增对 `ins_pose_topic`（默认 `/pose`，`geometry_msgs/PoseStamped`）和 `ins_odom_topic`（默认空，`nav_msgs/Odometry`）的订阅。第一条通过 §6 门控的消息会调用 `odometry_estimation->set_init_state(T, v)`，把优化器的重力参考一次性钉死。 |

完整数据通路见 [`docs/moving_start_initialization.md`](docs/moving_start_initialization.md)。

**本改动要求 GLIM 启动时必须有可用 INS。** 对纯 LiDAR 配置而言，需要回退 §5 —— 之前的 LOOSE / NAIVE 路径已经被移除。Hitch Sensor Dome 始终配备 Atlas Duo，因此交易是无条件的：以"必须有可用 INS"换"运动起步不再失败"。

## 6. 初始位姿的 RTK-fixed 准入门控

朴素的"接受第一条 pose"会愉快地咬住一个仍处于冷启动、IMU dead-reckoning、或 RTK-float 的 INS。整张 SLAM 地图都钉在这一条 pose 上，错一次就要重录。

wrapper 在调用 `set_init_state` 之前强制走**三阶段门控**：

| 阶段 | 检查 | 默认阈值 |
|------|------|----------|
| 1. fix 状态 | `NavSatFix.status.status ≥ STATUS_GBAS_FIX`（RTK 级） | `ins_require_rtk_fixed = true` |
| 2. 协方差 | `position_covariance` 对角线最大 σ ≤ 阈值 | `0.10 m` |
| 3. 稳定性 | 最近 N 条 INS pose 互相一致（平移 jitter、`\|q1·q2\|`） | `N=10`、`0.05 m`、`0.999` |

只要任一阶段未通过，2 秒钟一次的 wall-timer 就会触发 `ins_init_timeout_tick()`，**每 10 秒打印一次粗体红色多行警告**，列明最近一次拒绝的原因和补救步骤。`ins_init_timeout_s = 60 s` 后警告升级为"TIMEOUT" —— 但 **GLIM 永远不会自动 abort**，由操作员决定。

新增的八个 ROS 参数（在 [`launch/hitch_sensor_dome.launch.py`](launch/hitch_sensor_dome.launch.py) 中声明、在 [`glim/config/config_ros.json`](glim/config/config_ros.json) 中注释）：

```
ins_pose_topic                    默认 /pose
ins_odom_topic                    默认 ""
ins_fix_topic                     默认 /gps/fix
ins_require_rtk_fixed             默认 true
ins_max_position_stddev           默认 0.10
ins_min_pose_window_samples       默认 10
ins_max_pose_jitter_trans         默认 0.05
ins_min_quat_dot                  默认 0.999
ins_init_timeout_s                默认 60.0
```

## 7. RTK 门控的 GNSS 因子桥

上游的 `libgnss_global.so` 给全局图加 GNSS 软先验因子，但其 dispatcher 只识别 `nav_msgs/msg/Odometry` 与 `geometry_msgs/msg/PoseWithCovarianceStamped`。让它直接订阅 `/gps/fix`（NavSatFix）会静默丢弃所有消息 —— 本 fork 的早期版本正好就是这种错配，因此 §4 中的多圈 z-drift 调参在桥接出现前其实是 no-op。

上游模块（按其自身头文件注释）"忽略 GNSS 观测协方差" —— 即便能收到，RTK-fixed 与 RTK-float 也会被等权处理。

fork 在 wrapper 内增加了一个进程内桥：

```
fusion_engine_driver
   ├── /pose          (PoseStamped)
   └── /gps/fix       (NavSatFix, RTK 门控信号)
                  │
                  ▼
       GlimROS::try_publish_gnss_factor
           - 要求 RTK-fixed 状态（可配置）
           - 要求 pos σ ≤ 阈值
                  │
                  ▼
       /gnss/pose_rtk_only (PoseWithCovarianceStamped)
                  │
                  ▼
       libgnss_global.so 在此订阅
                  │
                  ▼
       把软因子加入全局图
       —— 仅在 RTK-fixed 时段。
```

| 参数 | 默认 | 用途 |
|-----------|---------|---------|
| `gnss_factor_topic` | `/gnss/pose_rtk_only` | 桥的发布 topic；设为 `""` 则禁用桥 |
| `gnss_factor_require_rtk_fixed` | `true` | 为 true 时仅在 RTK 级 fix 期间转发 |
| `gnss_factor_max_position_stddev` | `0.10 m` | NavSatFix 协方差超过则丢弃 |

每 10 秒的状态日志会打印 `N published, M rejected`，方便操作员实时观察桥是否在正常工作。

**RTK 失锁时的行为。** 当 RTK 退化为 float / no-fix（隧道、城市峡谷），桥会静默暂停；`gnss_global` 看到一个安静的 topic，本段时间不加任何因子。优化器靠 LiDAR 代价撑过这段空档。RTK 重新锁定后，下一条 `/pose` 起又开始发因子。**会话期间的 RTK 要求只对初始位姿（§6）是硬性的；后续每条消息的门控只是软暂停，不是硬阻塞。**

涉及文件：[`glim_ros2/src/glim_ros/glim_ros.cpp`](glim_ros2/src/glim_ros/glim_ros.cpp)、[`glim_ros2/include/glim_ros/glim_ros.hpp`](glim_ros2/include/glim_ros/glim_ros.hpp)、[`glim_ext/config/config_gnss_global.json`](glim_ext/config/config_gnss_global.json)、[`launch/hitch_sensor_dome.launch.py`](launch/hitch_sensor_dome.launch.py)。

## 8. 项目集成

`GLIM_plusplus/` 下三个新增的顶层目录，仅放集成代码（不动任何上游 GLIM 源码）：

| 目录 | 内容 |
|--------|----------|
| [`config/`](config/) | `generate_sensor_dome_urdf.py` 把 [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) 转成 `sensor_dome.urdf`，GLIM 通过 `config_sensors.json` 中的 `urdf_path` 字段读取，既用于 `T_lidar_imu` 也用于多 LiDAR `lidar_concat`。是采集、可视化、建图三方共享的单一信息源。TF YAML 改动后重跑此脚本。 |
| [`launch/`](launch/) | `hitch_sensor_dome.launch.py` —— 从 `sensor_dome_tf.yaml` 发布静态 TF、用项目调好的配置启动 `glim_rosnode`、起 `foxglove_bridge` 做可视化、跑一次预飞静止性检查。 |
| [`scripts/`](scripts/) | `check_init_stationarity.py` —— 预飞诊断脚本；读 `/imu/data` 前 3 秒，若 bag 不静止则打印粗体红色警告。在 fork 中已退化为信息提示（C++ INS-init 路径自身能处理运动起步），但仍可用于排查 Atlas Duo 锁定缓慢、CI gating 等。 |

## 9. 新增文档

[`docs/`](docs/) 目录下：

- [`moving_start_initialization.md`](docs/moving_start_initialization.md) —— §5 + §6 + §7 的完整设计。覆盖数据通路、RTK 门控语义、警告/成功输出示例，以及无 RTK 场景如何运作。
- [`multi_lap_loop_closure.md`](docs/multi_lap_loop_closure.md) —— 里程计初始化与 VGICP 收敛域之间 chicken-and-egg 的根因分析、三层修复、运行后四项验证（GNSS 因子计数、可视化器中的位姿图边、`T_world_utm.txt` 是否稳定、轨迹高度 vs GNSS 高度），以及五步升级方案（默认修复仍不足时使用）。

## 10. 未改动的部分

很重要 —— 让采用者知道哪些部分与上游一致，可以继续依赖现有 GLIM 文档：

- 因子图优化器（`gtsam_points::IncrementalFixedLagSmootherExtWithFallback`） —— 未改动。
- VGICP 扫描匹配本身（cost、梯度、voxel 结构） —— 未改动。
- sub-mapping、global mapping 模块结构 —— 未改动。
- `gtsam_points` / GTSAM 依赖 —— 未改动。
- Iridescence 原生可视化器 —— 未改动。
- IMU 积分（`imu_integration.cpp`） —— 未改动。
- 点云预处理（`cloud_preprocessor.cpp`、`cloud_deskewing.cpp`） —— 未改动。
- 除 `gnss_global` 外所有扩展模块 —— 未改动（gravity_estimator、flat_earther、deskewing、imu_validator、imu_prediction、velocity_suppressor、orb_slam、scan_context_loop_detector、dbow_loop_detector）。
- License 文本 —— 每一个上游 LICENSE / NOTICE 都原文保留。
- 子包 README（`glim/README.md`、`glim_ext/README.md`、`glim_ros2/README.md`） —— 未改动。它们按上游自身视角写就，改动会模糊上游归属。

## 11. 按文件汇总的差异

| 文件 | 状态 | 类别 |
|------|--------|-----------------|
| `glim/include/glim/odometry/odometry_estimation_base.hpp` | 修改 | §5（新增虚方法） |
| `glim/include/glim/odometry/odometry_estimation_imu.hpp` | 修改 | §5（override 声明） |
| `glim/include/glim/odometry/async_odometry_estimation.hpp` | 修改 | §5（forwarder 声明 + 状态） |
| `glim/src/glim/odometry/initial_state_estimation.cpp` | 修改 | §5（移除"由加速度计估计重力"） |
| `glim/src/glim/odometry/odometry_estimation_imu.cpp` | 修改 | §5（强制 NAIVE、`set_init_state` 实现） |
| `glim/src/glim/odometry/async_odometry_estimation.cpp` | 修改 | §5（forwarder 实现 + 队列消耗） |
| `glim/config/config_ros.json` | 修改 | §1 §2 §3 §6（topic、frame、IMU、RTK 门控参数） |
| `glim/config/config_sensors.json` | 修改 | §1 §3（Robin W 适配、IMU 噪声、deskewing） |
| `glim/config/config_preprocess.json` | 修改 | §3（下采样目标、k_correspondences） |
| `glim/config/config_odometry_gpu.json` | 修改 | §3（voxel 尺度、smoother 窗口） |
| `glim/config/config_sub_mapping_gpu.json` | 修改 | §3（submap 关键帧、voxel） |
| `glim/config/config_global_mapping_gpu.json` | 修改 | §3 §4（回环阈值） |
| `glim_ext/config/config_gnss_global.json` | 修改 | §4 §7（prior_inf_scale、桥的 topic / type） |
| `glim_ros2/include/glim_ros/glim_ros.hpp` | 修改 | §5 §6 §7（订阅、参数、因子桥状态） |
| `glim_ros2/src/glim_ros/glim_ros.cpp` | 修改 | §5 §6 §7（ROS 侧完整实现） |
| `config/generate_sensor_dome_urdf.py` | 新增 | §8 |
| `config/sensor_dome.urdf` | 新增（生成） | §8 |
| `launch/hitch_sensor_dome.launch.py` | 新增 | §8 |
| `scripts/check_init_stationarity.py` | 新增 | §8 |
| `docs/moving_start_initialization.md` | 新增 | §9 |
| `docs/multi_lap_loop_closure.md` | 新增 | §9 |
| 其他所有上游文件 | 未改动 | §10 |

## 快速开始

为您的安装编辑一次 [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml)，然后：

```bash
cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py
cd ../..

# 编译（与上游一致：CMake / colcon、gtsam、gtsam_points、Iridescence）
colcon build --packages-select glim glim_ext glim_ros \
             --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 实时建图（直接对接采集栈）：
ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py

# 离线对 MCAP 包重放：
ros2 run glim_ros glim_rosbag recording/data/session_<ts>/rosbag2 \
    --ros-args -p config_path:=GLIM_plusplus/glim/config \
                -p dump_path:=glim_maps/session_<ts>
```

ROS 2 包名（`glim`、`glim_ext`、`glim_ros`）与上游一致 —— 仅 workspace 文件夹名不同，所以 `colcon build --packages-select glim …` 用法不变。

## 编译依赖

与上游 GLIM 完全相同。具体清单见上游文档。

```bash
sudo apt install -y libeigen3-dev libboost-all-dev libfmt-dev libomp-dev \
                    libmetis-dev ros-${ROS_DISTRO}-tf2-eigen \
                    ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-foxglove-bridge
# 之后是 GTSAM、gtsam_points、Iridescence —— 见上游 README。
```

## Credits

GLIM 由以下人员开发：

- **GLIM** —— Kenji Koide、Masashi Yokozuka、Shuji Oishi、Atsuhiko Banno (AIST)。<https://github.com/koide3/glim>
- **gtsam_points** —— Kenji Koide。<https://github.com/koide3/gtsam_points>
- **Iridescence** —— Kenji Koide。<https://github.com/koide3/iridescence>
- **GTSAM** —— Frank Dellaert 与 Georgia Tech Borg Lab。<https://github.com/borglab/gtsam>

`GLIM_plusplus/{config, launch, scripts, docs}/` 中的集成代码以及 §1 – §7 中所列改动属于 **Hitch Sensor Dome** 项目，由 Dr. Allen Y. Yang（Hitch Interactive · 加州大学伯克利分校）设计与维护。实现测试由 **Berkeley AI Racing Tech** 团队完成（见 [`../README.md`](../README.md) Credits）。

## License

本 fork 继承所有组成包的 license。**任何上游 license 文本都未被修改。**

- **GLIM** —— MIT License（Kenji Koide / AIST）
- **gtsam_points** —— MIT License
- **GTSAM** —— BSD License
- **Iridescence** —— MIT License
- **Hitch Sensor Dome 集成代码** —— 见 [`../LICENSE`](../LICENSE)。

每个包目录（`glim/`、`glim_ext/`、`glim_ros2/`）内部完整保留了上游 license 原文。

## Citation

如果在学术工作中使用了 GLIM++，请引用上游 GLIM 论文：

```bibtex
@article{koide2024glim,
  title   = {GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors},
  author  = {Koide, Kenji and Yokozuka, Masashi and Oishi, Shuji and Banno, Atsuhiko},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2024}
}
```

如果本 README 中的特定改动对您有帮助，附加引用：

```bibtex
@misc{yang2026hitchsensordome,
  title  = {Hitch Sensor Dome: a 3D-printable modular multi-sensor mount for vehicle-roof mapping},
  author = {Yang, Allen Y.},
  year   = {2026},
  note   = {GitHub repository — 含 GLIM++ fork 于 GLIM_plusplus/}
}
```
