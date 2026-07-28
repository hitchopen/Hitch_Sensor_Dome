# GLIM++ —— 面向 Hitch Sensor Dome 的深度改造 GLIM fork

> **这不是原版 GLIM。** 文件夹名称特意采用 `GLIM_plusplus/`：本版本在算法行为层面（不仅是配置）已与上游 [`koide3/glim`](https://github.com/koide3/glim) 出现差异。如果您是来寻找原版 GLIM 的，原版仓库地址为 <https://github.com/koide3/glim>，并强烈建议从那里开始 —— 除非您拥有 [Hitch Sensor Dome](../README.md) 硬件。GLIM 算法与实现的全部功劳归 **Kenji Koide、Masashi Yokozuka、Shuji Oishi、Atsuhiko Banno（AIST）** 所有。详见 [Credits](#credits)、[License](#license)、[Citation](#citation)。

本文档是 GLIM++ 与上游 `koide3/glim` 之间的**完整变更日志**。结构按机制分节，方便采用者逐项判断哪些改动适合自己的使用场景，哪些需要回退。

> **⚠ 本中文文档落后于英文版。** 以下章节目前**仅存在于**
> [`README.md`](README.md)，尚未翻译：
>
> - §12 按文件汇总的差异（file-by-file diff summary）
> - §13 2026-07 P1–P5 改进（合并证据、yaw 质量门控、GNSS 回填）
> - §14 2026-07-27 上游重新合并（逐点时间戳的扫描匹配、GNSS 锚定健康门控、PR #15）
> - 快速开始中的单元测试与建图 profile 生成器用法
>
> Robin W 的规范现已确认：驱动把原始相对点偏移补全为
> `timestamp/FLOAT64` 绝对 Unix 秒。详情请以英文版 §14 为准。
> 涉及配置或时间戳行为的改动，请以英文版为准。

## 变更索引

1. [面向 Hitch Sensor Dome 的传感器适配](#1-面向-hitch-sensor-dome-的传感器适配)
2. [车辆无关主体坐标系（`base_frame_id = imu_link`）](#2-车辆无关主体坐标系)
3. [面向户外 / 车辆尺度的整体调参（24 项参数变更）](#3-面向户外--车辆尺度的整体调参)
4. [多圈回环修复](#4-多圈回环修复)
5. [初始化重写 —— INS 驱动，移除"由加速度计估计重力"](#5-初始化重写)
6. [初始位姿的 RTK-fixed 准入门控](#6-初始位姿的-rtk-fixed-准入门控)
7. [RTK 门控的 GNSS 因子桥（init 之后）](#7-rtk-门控的-gnss-因子桥)
8. [项目集成 —— URDF 生成器、启动助手、诊断脚本](#8-项目集成)
9. [设计说明 —— 回环闭合、运动起步、TF 验证、合并记录](#9-设计说明)
10. [未改动的部分](#10-未改动的部分)
11. [按文件汇总的差异](#11-按文件汇总的差异)

## 1. 面向 Hitch Sensor Dome 的传感器适配

topic、frame、字段名均面向 Hitch Sensor Dome 参考配置（3× Seyond Robin W + Point One Atlas Duo + 4× e-con RouteCAM）。

| 项目 | Hitch Sensor Dome 配置 |
|------|-----|
| IMU topic | `/imu/data` |
| 主 LiDAR | `/robin_w_front/points` |
| 辅 LiDAR | `/robin_w_rear_left/points`、`/robin_w_rear_right/points` |
| GNSS | `/gps_p1/fix`（适配器同步 NavSatFix 门控信号） |
| 摄像头 | `/cam_front_left/image_raw` |
| `intensity_field` | `intensity` |
| `ring_field` | `ring` |
| `flip_points_y` | `false`（Robin W 设置 `coordinate_mode:=3` 后符合 REP-103） |
| LiDAR–IMU 外参来源 | 由 [`config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) 生成的 URDF —— 见 §8 |

涉及文件：`glim/config/config_sensors.json`、`glim/config/config_ros.json`。

### 1.1 单天线 vs 双天线模式

Hitch Sensor Dome 支持单天线或双天线 GNSS 配置（在已知偏置位置加装第二根天线即可启用无漂移 RTK 航向）。模式由 [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) 自动检测：副天线平移默认为哨兵值 `(0, 0, 0)`（单天线模式），任何非零平移（范数 ≥ 0.05 m）都会让 GLIM++ 在启动时切换到双天线模式。启用双天线的完整步骤参见项目 [根 README](../README_zh.md#-双-gnss-天线--强烈推荐)；GLIM++ 在两种模式间的算法差异汇总如下。

| 维度 | 单天线 | 双天线 |
|--------|----------------|--------------|
| **航向来源** | IMU 陀螺仪积分（随 bias 漂移） | RTK-fixed 双天线基线（无漂移，约 0.1°–1°，取决于基线长度） |
| **Init 门控 `ins_max_attitude_residual_deg`** | 姿态残差 `2.5°` | 自动收紧到 `0.8°` |
| **Init 门控 `ins_min_pose_window_samples`** | `10` 条连续一致样本 | 自动缩短到 `5`（航向锁定更快） |
| **Init 门控 `ins_init_timeout_s`** | `60 s` | 自动缩短到 `30 s` |
| **Factor-bridge 朝向协方差** | 不填充（PoseStamped 的朝向信息不可用） | 由基线长度推导出的紧 yaw σ，松 pitch/roll —— 见 §7 |
| **会话期航向漂移** | 随 IMU bias 累积；只能靠 LiDAR 扫描匹配抑制 | 在整段会话中由 RTK 航向约束（数据通路已就绪，因子模块暂缓 —— 见 §9「会话期航向修正」） |
| **长 / 多圈轨迹下的地图质量** | yaw 稳定性依赖 LiDAR 特征丰富度；只要 RTK 位置 fixed，z 锚点仍可靠 | 初始化时朝向精度更高，并为后续会话期航向修正预留了清晰数据路径 |
| **硬件需求** | 一根 SP1（或同级天线） | 两根天线，固定偏置（推荐基线 1.0–1.5 m） |
| **启动时操作员看到的日志** | `Hitch fork: SINGLE-antenna mode — heading derived from IMU (drift-prone).` | `Hitch fork: DUAL-antenna mode — baseline=1.000 m, expected heading σ=0.010 rad (0.57°). Init gates auto-tightened.` |

模式切换**完全自动** —— 没有独立的"双天线"启动参数。GLIM++ 启动时读取 TF YAML、计算副天线平移范数、自行选择。如要从双天线回退到单天线（例如现场副天线失效），把 `sensor_dome_tf.yaml` 里副天线平移清零即可，其它部分照常工作 —— 主天线一根就足以为 GLIM++ 的初始化门控（§6）和因子桥（§7）提供 RTK 位置。

## 2. 车辆无关主体坐标系

[`glim/config/config_ros.json`](glim/config/config_ros.json) 中：

```json
"imu_frame_id":  "imu_link",
"lidar_frame_id":"lidar_front_link",
"base_frame_id": "imu_link"      // 原值: "" (auto-detect)
```

将 `base_frame_id` 固定为 `imu_link`，使 GLIM 围绕 Atlas Duo 导航中心（CoN）建图，而不是某个具体车辆的 `base_link`。各下游车辆集成商再发布自己的静态 `imu_link → base_link` —— 这样建好的地图无需重跑 SLAM 即可在不同车辆平台上复用。

这是相对上游的有意分歧：上游默认 `base_frame_id` 为空，会沿用 IMU frame 的名字（由驱动决定、与具体车辆耦合的字符串）。

## 3. 面向户外 / 车辆尺度的整体调参

七个 JSON 配置中共 24 项参数变更，每项在文件内附注释。按主题分组：

| 维度 | 文件 | 方向 |
|------|-------|-----------|
| **IMU 噪声 / bias** | `config_sensors.json` | 放宽：`acc 0.05 → 0.2`、`gyro 0.02 → 0.05`、`bias 1e-5 → 1e-4`。降低 IMU 信任、提高 LiDAR 权重。 |
| **加速度计标度** | `config_ros.json` | `acc_scale: 0.0 → 1.0`。Atlas Duo 原生输出 m/s²，无需自动检测。 |
| **初始化** | `config_odometry_gpu.json` | 窗口 `1.0 → 3.0` s（后已被完全替换 —— 见 §5）。 |
| **GNSS 先验** | `config_gnss_global.json` | `prior_inf_scale: [0,0,0] → [100, 100, 25]`，并启用协方差感知的 `prior_inf_floor` / `prior_inf_cap` 与 Huber 宽度（上游默认值相当于禁用 GNSS）。`min_baseline: 1.0 → 10.0`（world↔UTM 对齐拟合门限；此前的 `0.5` 建立在一个**错误的**「提高因子密度」前提上）。 |
| **运动畸变补偿** | `config_sensors.json` | `global_shutter_lidar: true → false`。重新启用畸变补偿（上游多 LiDAR 时间戳重基 bug 已被修复，无需再绕开）。 |
| **每点时间戳** | `config_sensors.json` | Robin W 使用 `timestamp/FLOAT64` 绝对 Unix 秒；`autoconf_perpoint_times: false`、`perpoint_relative_time: false`、比例 `1.0`。 |
| **下采样** | `config_preprocess.json` | `random_downsample_target: 10000 → 30000`、`k_correspondences: 10 → 20`。3 路传感器拼成的 360° 点云需更高密度；3× 上游值之所以可接受，是因为穹顶数据流程是**离线**针对录制 MCAP 包跑 GLIM —— 没有实时单帧时间预算。如果改为在线建图，建议降回 15-20K。 |
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
| GNSS `prior_inf_scale[2]` (z) | `1e4` | `25` —— 实际配置为 `prior_inf_scale = [100, 100, 25]`、`prior_inf_floor = [100, 100, 25]`、`prior_inf_cap = [2500, 2500, 1000]`。**垂直方向被刻意设置得比水平方向更*弱*，而不是更强。** |
| GNSS `min_baseline` | `1.0` m | `10.0` m（world↔UTM 对齐拟合门限 —— **不是**因子密度旋钮，见下方勘误） |

> **勘误（2026-07-27）。** 上表此前把 `min_baseline` 记为 `0.5` m 并注为「GNSS 因子密度翻倍」。该前提是错误的：`min_baseline` **只**参与一次性的 `T_world_utm` 对齐拟合门限，从不参与因子发射循环 —— 在模块中 grep 即可确认。每一个完成关联的 submap 都会得到一个因子，与 `min_baseline` 无关，因此调低它一个因子也不会多加；它真正的作用是允许 world↔UTM 对齐用一段更短、更嘈杂的基线去拟合，而在 `0.5` m 下这会产生数度的 yaw 误差并在整轮运行中被锁死（H1）。现已恢复为 `10.0` m，并配合 `fit_min_samples` 与样本外验证窗口。要调整 GNSS 相对 LiDAR 的权重，请使用 `prior_inf_scale` / `prior_inf_floor` / `prior_inf_cap`，**不要**动 `min_baseline`。
>
> 同一版本中 `prior_inf_scale[2]` 那一行的方向也写反了：它声称 z 被设置为比水平方向强 5×，而实际配置恰好相反 —— 垂直精度 `25` 对水平 `100`（σ ≈ 0.2 m 对 0.1 m），上限 `1000` 对 `2500`（σ ≈ 3.2 cm 对 2.0 cm）。这符合 RTK 的真实误差特性：垂直精度约为水平的一半，过度信任 z 会把地图高程从 LiDAR 解上拉偏。绝对量级从 `1e4` 降到 `100`，是因为协方差感知的 floor/cap 路径现在才是实际生效的加权机制，`prior_inf_scale` 仅作为「样本不携带可用协方差」时的兜底；若仍保持 `1e4`（1 cm），这类兜底样本反而会比自适应上限更硬。

前三项拓宽了收敛域，使在残余漂移下回环因子仍能触发；GNSS 两项则从源头阻止漂移累积。两层修复缺一不可 —— 原因是 §9「多圈 z 漂移」中详述的 chicken-and-egg：回环因子只有在两圈之间 VGICP 收敛时才会产生，而 VGICP 只有在累积漂移已小于其收敛域时才会收敛；因此一旦越过某个阈值，本应纠正漂移的机制恰恰被漂移本身关掉了。

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
| [`glim_ros2/include/glim_ros/glim_ros.hpp`](glim_ros2/include/glim_ros/glim_ros.hpp) 与 [`.cpp`](glim_ros2/src/glim_ros/glim_ros.cpp) | 支持兼容模式 `ins_pose_topic`（生产默认空）和 `ins_odom_topic`（生产默认 `/gps_p1/filtered_odom_rtk_fixed`，`nav_msgs/Odometry`）。第一条通过 §6 门控的消息会调用 `odometry_estimation->set_init_state(T, v)`。 |

完整数据通路见 §9「运动起步初始化」：门控语义、成功与拒绝时操作员看到的输出、初始速度注入，以及 RTK 失锁期间的行为。

**本改动要求 GLIM 启动时必须有可用 INS。** 对纯 LiDAR 配置而言，需要回退 §5 —— 之前的 LOOSE / NAIVE 路径已经被移除。Hitch Sensor Dome 始终配备 Atlas Duo，因此交易是无条件的：以"必须有可用 INS"换"运动起步不再失败"。

## 6. 初始位姿的 RTK-fixed 准入门控

朴素的"接受第一条 pose"会愉快地咬住一个仍处于冷启动、IMU dead-reckoning、或 RTK-float 的 INS。整张 SLAM 地图都钉在这一条 pose 上，错一次就要重录。

wrapper 在调用 `set_init_state` 之前先要求 adapter 的权威 Fixed-only 数据源，再走三项独立门控：

| 阶段 | 检查 | 默认阈值 |
|------|------|----------|
| 0. 解类型 | 消息来自 adapter 的 Fixed-only odometry | FusionEngine `kRtkFixed` |
| 1. fix 状态 | `NavSatFix.status.status ≥ STATUS_GBAS_FIX`（独立 RTK 级交叉检查） | `ins_require_rtk_fixed = true` |
| 2. 协方差 | 类型已知、对角线有限且为正、最大 σ ≤ 阈值 | `0.10 m` |
| 3. 平滑性 | 最近 N 条 INS 位姿必须*时间连续*，且每一条都符合由其前序样本外推的匀速 / 匀角速度模型。阈值约束的是**残差**而非原始的相邻样本差值，因此匀速行驶或稳定转向在任何速度下都能通过 —— 详见 §9 | `N=10`，残差 `0.05 m` / `2.5°`，最大间隔 `0.5 s` |

只要任一阶段未通过，2 秒钟一次的 wall-timer 就会触发 `ins_init_timeout_tick()`，**每 10 秒打印一次粗体红色多行警告**，列明最近一次拒绝的原因和补救步骤。`ins_init_timeout_s = 60 s` 后警告升级为"TIMEOUT" —— 但 **GLIM 永远不会自动 abort**，由操作员决定。

新增的八个 ROS 参数（在 [`launch/hitch_sensor_dome.launch.py`](launch/hitch_sensor_dome.launch.py) 中声明、在 [`glim/config/config_ros.json`](glim/config/config_ros.json) 中注释）：

```
ins_pose_topic                    默认 ""
ins_odom_topic                    默认 /gps_p1/filtered_odom_rtk_fixed
ins_fix_topic                     默认 /gps_p1/fix
ins_require_rtk_fixed             默认 true
ins_max_position_stddev           默认 0.10
ins_min_pose_window_samples       默认 10
ins_max_pose_jitter_trans         默认 0.05
ins_max_attitude_residual_deg     默认 2.5   （原 ins_min_quat_dot 0.999）
ins_init_timeout_s                默认 60.0
```

## 7. RTK 门控的 GNSS 因子桥

上游的 `libgnss_global.so` 给全局图加 GNSS 软先验因子，但其 dispatcher 只识别 `nav_msgs/msg/Odometry` 与 `geometry_msgs/msg/PoseWithCovarianceStamped`。让它直接订阅 `/gps/fix`（NavSatFix）会静默丢弃所有消息 —— 本 fork 的早期版本正好就是这种错配，因此 §4 中的多圈 z-drift 调参在桥接出现前其实是 no-op。

上游模块（按其自身头文件注释）"忽略 GNSS 观测协方差" —— 即便能收到，RTK-fixed 与 RTK-float 也会被等权处理。

fork 在 wrapper 内增加了一个进程内桥：

```
fusion_engine_driver + adapter
   ├── /gps_p1/filtered_odom_rtk_fixed (Odometry，仅 kRtkFixed)
   └── /gps_p1/fix    (适配器 NavSatFix, RTK 门控信号)
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

**RTK 失锁时的行为。** 当 RTK 退化为 float / no-fix（隧道、城市峡谷），adapter 的 Fixed-only topic 会停止发布；`gnss_global` 本段时间不加任何因子，优化器靠 LiDAR-IMU SLAM 撑过空档。RTK 重新锁定后，下一条 Fixed odometry 起恢复因子。

涉及文件：[`glim_ros2/src/glim_ros/glim_ros.cpp`](glim_ros2/src/glim_ros/glim_ros.cpp)、[`glim_ros2/include/glim_ros/glim_ros.hpp`](glim_ros2/include/glim_ros/glim_ros.hpp)、[`glim_ext/config/config_gnss_global.json`](glim_ext/config/config_gnss_global.json)、[`launch/hitch_sensor_dome.launch.py`](launch/hitch_sensor_dome.launch.py)。

## 8. 项目集成

`GLIM_plusplus/` 下三个新增的顶层目录，仅放集成代码（不动任何上游 GLIM 源码）：

| 目录 | 内容 |
|--------|----------|
| [`config/`](config/) | `generate_sensor_dome_urdf.py` 把 [`../config/sensor_dome_tf.yaml`](../config/sensor_dome_tf.yaml) 转成 `sensor_dome.urdf`，GLIM 通过 `config_sensors.json` 中的 `urdf_path` 字段读取，既用于 `T_lidar_imu` 也用于多 LiDAR `lidar_concat`。是采集、可视化、建图三方共享的单一信息源。TF YAML 改动后重跑此脚本。 |
| [`launch/`](launch/) | `hitch_sensor_dome.launch.py` —— 从 `sensor_dome_tf.yaml` 发布静态 TF、用项目调好的配置启动 `glim_rosnode`、起 `foxglove_bridge` 做可视化、跑一次预飞静止性检查。 |
| [`scripts/`](scripts/) | `check_init_stationarity.py` —— 预飞诊断脚本；读 `/imu/data` 前 3 秒，若 bag 不静止则打印粗体红色警告。在 fork 中已退化为信息提示（C++ INS-init 路径自身能处理运动起步），但仍可用于排查 Atlas Duo 锁定缓慢、CI gating 等。 |

## 9. 设计说明

本节刻意做成自包含的：采用者操作、排障或复现本 fork 行为所需的全部内容都在
本文件内，没有需要另行索取的配套文档，本节链接也不指向仓库以外的任何东西
（已公开发表的上游参考资料除外）。

### 多圈 z 漂移 —— §4 背后的 chicken-and-egg

现象：第 1 圈建图正常，从第 2 圈起轨迹整体上翘，闭合回路的终点比起点高出几十
厘米甚至数米。这不是扫描匹配的 bug，而是回环闭合的**未发生**。

每帧微小的 z 误差会单调累积，因为没有任何东西约束它。车辆回到已建图区域时本应
产生一个隐式回环因子把位姿图拉平，但该因子只有在第 1 圈与第 2 圈的 submap 之间
VGICP 收敛时才存在 —— 而 VGICP 只有在累积偏移已经落在其体素收敛域内时才会收敛。
越过这个阈值之后，本应纠正漂移的机制恰恰是被漂移关掉的那一个。仅仅拓宽收敛域也
不够：漂移足够大时任何有限的收敛域都会失效，所以还必须从源头阻止漂移累积。这正
是 §4 同时调整一个收敛域旋钮和一个 GNSS 旋钮的原因；只做其中一半，故障模式依然
可达。

**运行后如何验证。** 四项检查，建议按此顺序：

1. **GNSS 因子确实在插入。** 读退出时机器可解析的 `gnss_global summary:` 行。
   `factors_delivered` 应稳定增长 —— 每个完成关联的 submap 一个因子，与
   `min_baseline` **无关**（后者只作用于一次性的 world↔UTM 拟合门限，从不参与
   因子发射循环）。若计数偏低，说明 GNSS 被过滤掉了：检查 `gap_unanchored`
   （RTK 失锁）、`yaw_gate_skips` 和 `submaps_dropped_no_bracket`，并确认驱动
   上报的协方差没有悲观到让 RTK 门控拒绝每一个样本。
2. **进入第 2 圈时回环因子触发。** 车辆重新进入已建图区域时，可视化器中应出现
   连接时间上相距较远的 submap 之间的位姿图边。
3. **`T_world_utm.txt` 只写一次且保持稳定。** 若运行结束仍未生成该文件，说明
   GNSS 从未完成对齐 —— 请把 `min_baseline` 与首次 RTK 锁定前实际行驶的距离作
   对比。
4. **轨迹高程跟随 GNSS 高程。** 两者的差值随圈数持续拉大，就是漂移本身的直接
   图像。

**默认配置仍不够时如何升级。** 继续加大 `submap_voxel_resolution_max`，或提高
`prior_inf_scale` / 收紧 `prior_inf_floor` + `prior_inf_cap`，让 GNSS 在与
LiDAR 因子质量的竞争中占更大权重。**不要调低 `min_baseline`** —— 见 §4 的勘误：
它一个因子都不会增加，只会劣化 world↔UTM 拟合。

### 运动起步初始化 —— §5–§7 背后的数据通路

上游 GLIM 用前 `initialization_window_size` 秒的加速度计均值估计世界坐标系朝向，
这只有在该窗口内 IMU 全程静止时才成立。本 fork 用取自 Atlas Duo 的外部 INS 先验
取而代之，因此录制可以从车辆已经在运动的状态开始。

**门控语义。** SLAM 只在存在**经过校验且新鲜的 RTK-Fixed** 解时才启动。这是刻意
的设计，不是图省事的默认值：在构建任何因子之前，地图原点必须具备全局参考，INS
姿态必须通过校验。在此之前，初始化看门狗每 10 秒告警一次并附上当前的拒绝原因 ——
离线回放与在线运行都会打印。

**为什么门控检验的是平滑性而不是静止性。** 最初的稳定性检查把原始的相邻样本位移
与 `ins_max_pose_jitter_trans`（0.05 m）比较。在 Atlas Duo 10 Hz 的位姿频率下，
这等于一道 0.5 m/s 的硬上限 —— 它恰好禁止了该特性本应支持的运动起步，因为它无法
区分 INS 噪声与车辆的真实平移。现在门控的两半都改为基于残差：位置用匀速残差，
朝向用匀角速度残差，另加连续性要求。形如 `|q1·q2| > 0.999` 的朝向判据在旋转轴上
有同样的缺陷 —— 它把 yaw 速率隐式地限制在约 25°/s —— 所以被替换而非保留。

同一个阈值上还藏着第二个更隐蔽的缺陷，已于 2026-07-27 修复。对单位四元数有
`|q_a·q_b| = cos(θ/2)`，因此用点积表示的界限，实际允许的角度**恰好是**读者据此
推算出来的**两倍**。本 fork 中处处把 `0.999` 记作「2.5°」，而它实际允许 **5.125°**；
双天线的 `0.9999` 被记作「0.8°」，实际允许 **1.621°**。现在该界限直接以角度配置
（`ins_max_attitude_residual_deg`，默认 `2.5`，双天线模式自动收紧到 `0.8`），
内部再转换成点积 —— 单位被写进了参数名里，这类错误无法再次发生。该门控因此比此
日期之前的任何一次运行都真正严格了 2 倍；如果旧 bag 现在在姿态项上无法完成初始化，
请有意识地调大这个角度值，而不要回退 —— 5.125° 从来就不是设计意图。

**初始速度。** 直接采用 INS 速度，而不是对位姿做有限差分，这也是首选
`nav_msgs/Odometry` 作为初始化源的原因。Point One 在平台机体系（前-左-上）中报告
该速度，而 GLIM 期望的是世界系，因此 fork 施加 `v_world = T.linear() * v_body`。
不做旋转直接透传，会让估计器沿错误的世界轴初始化，偏差正是车辆航向角 —— 在 90°
yaw 时，20 m/s 的前向速度会被横向注入。

**RTK 失锁期间。** 出厂 profile 从不接受 Float 作为 GNSS 因子。会话一旦从 Fixed
完成初始化，RTK 丢失只会让 adapter 的 Fixed-only 话题安静下来，LiDAR–IMU SLAM
在无锚定状态下继续；Fixed 恢复后因子随之恢复。放宽 NavSatFix 协方差阈值绕不过
权威解算类型检查，也不是在无 RTK 条件下启动一次运行的受支持方式。

**如何验证一次运动起步。** 确认该次运行是在非零速度下完成初始化的，且上报的初始
速度大小与 INS 一致。相比静止起步，优化器收敛所需时间会略长，因为要从非零初始
状态估计更多 IMU bias；`scripts/check_init_stationarity.py` 仍作为信息性的预飞
诊断与 CI 门控保留，但已不再是前置条件。

### 会话期航向修正 —— 数据通路已就绪，因子模块暂缓

因子桥已经在每一条发布的位姿上填好了正确的朝向协方差，双天线模式下 yaw σ 很紧。
上游 `gnss_global` 完全忽略协方差、只消费位置，所以这部分朝向信息目前止步于桥。
有两种用法：改造 `gnss_global`，在 yaw σ 足够紧时添加带协方差的朝向因子（对上游
模块侵入较大）；或新增一个独立扩展模块，订阅桥的话题、提取 yaw 与 yaw σ，向全局
图贡献一个仅 yaw 的先验（与上游干净解耦，经 `extension_modules` 选择性启用）。
两条路径所需的数据通路都已就绪；因子模块被刻意留到后续迭代。

### TF 验证 —— 外参对照 3D 设计

上游合并完成后，P1 与 Robin W 的外参已从 OpenSCAD 源头一路核验到 GLIM 配置：

```
3D files/sensor_dome.scad          （唯一几何来源）
        │
        ▼
config/sensor_dome_tf.yaml         （唯一 TF 信息源）
        │  generate_sensor_dome_urdf.py     │ launch：每条 YAML 条目一个
        ▼                                   ▼ static_transform_publisher (tf2_ros)
GLIM_plusplus/config/sensor_dome.urdf
        │
        ▼
config_sensors.json（经 urdf_path 得到 T_lidar_imu、lidar_concat 辅助帧）
config_gnss_global.json（杆臂）
```

全部通过：

| Frame | YAML / URDF 值 | 设计推导 | STL 实测 |
|---|---|---|---|
| `lidar_front_link` | (0.080, 0, 0.1145)，yaw 0° | 安装环 r = 80 mm @ 0° | 4× M6 孔位与名义值相差 **0.05 mm** 以内 |
| `lidar_rear_left_link` | (−0.040, 0.069282, 0.1145)，yaw 120° | 80·(cos 120°, sin 120°) = (−40, 69.282) | **0.05 mm** 以内 |
| `lidar_rear_right_link` | (−0.040, −0.069282, 0.1145)，yaw 240° | 80·(cos 240°, sin 240°) | **0.05 mm** 以内 |

z 偏置 0.1145 m 由安装面 6 mm（L1 板）+ 133 mm（立柱）= 139 mm 与 Atlas 导航中心
6 + 18.5 = 24.5 mm 相减得到。实测一体件总高 151.0 mm，确认 139 mm 为实打印值。
只用 yaw 旋转是成立的，因为 Seyond 驱动被固定为 `coordinate_mode:=3`（REP-103），
Robin W 的原生坐标轴在驱动层就已重映射。

IMU 侧：Atlas 装配图把导航中心定在距孔位基准 (68.7, 47.8) mm、安装面上方 18.5 mm
处，而 SCAD 把该零件摆放成让导航中心正好落在设计原点。四个 M4 安装特征在
220 × 100 mm 跨距上与 SCAD 位置相差 **0.1 mm** 以内。`enable_lever_arm` 为
`false`、`urdf_gnss_frame` 为空，对于本身就在导航中心输出位姿的紧耦合 Atlas Duo
而言是正确的。`sensor_dome.urdf` 与从 YAML 重新生成的结果逐字节一致，因此两者不
可能各自漂移。

两条长期成立的注意事项。其一，TF 把 `imu_link` 的坐标轴视为与车辆对齐；Atlas 的
导航系与其机械朝向之间的对齐关系属于 Point One 的输出配置，无法从 3D 文件推导
（假定正确，并已由此前的实车运行验证）。其二，`gnss_antenna_primary_link` 的
z = 0.273 只是支架的名义高度，应按实际安装测量 —— 由于杆臂补偿已关闭，它对 GLIM
而言仅具文档意义。

### 上游合并记录

英文版 §14 即合并记录，说明了从 `ucb-roar` 与 PR #15 移植了什么、刻意跳过了什么，
以及每一项穹顶专有行为如何在冲突解决中保留下来。上游来源可直接访问：
[`koide3/glim`](https://github.com/koide3/glim)（基础项目）与
[`augcog/DLIO_plusplus`](https://github.com/augcog/DLIO_plusplus)（加固分支）。

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

`GLIM_plusplus/{config, launch, scripts}/` 中的集成代码以及 §1 – §7 中所列改动属于 **Hitch Sensor Dome** 项目，由 Dr. Allen Y. Yang（Hitch Interactive · 加州大学伯克利分校）设计与维护。实现测试由 **Berkeley AI Racing Tech** 团队完成（见 [`../README.md`](../README.md) Credits）。

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
