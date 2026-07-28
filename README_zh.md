# Hitch Sensor Dome

可 3D 打印的模块化传感器穹顶，通过吸盘式相机支架将多传感器测绘平台安装到车顶。

> **⚠ 本中文文档落后于英文版。** 以下内容目前**仅存在于**
> [`README.md`](README.md)，尚未翻译：
>
> - 「2026-07 定位与建图改进 (P1–P5)」整节
> - 「2026-07-27 GLIM++ 上游重新合并」小节
> - Robin W 逐点时间戳的已确认规范：原始点偏移由驱动补全为
>   `timestamp/FLOAT64` 绝对 Unix 秒
>
> 硬件、装配与接线部分的中文内容仍然有效。涉及软件行为的部分请以英文版为准。

## 传感器

- 3× Seyond Robin W LiDAR —— 单台 120° 水平视场，按 120° 间隔排布，实现 360° 环视
- 1× Point One Nav Atlas Duo INS —— 居中安装，导航中心 (CoN) 与几何原点重合
- 1× **Point One SP1** 多频段 (L1/L2/L5) GNSS 天线 —— 安装在 ArduSimple 测量支架上，置于 CoN 正上方
- 4× e-con RouteCAM_P_CU25_CXLC_IP67 摄像头 —— 前向立体对 (110 mm 基线) + 后向对称对

> **GNSS 天线说明。** Atlas Duo 的 GNSS 端口为有源 LNA 提供 **3.3 V DC 偏置** (见 [Point One Atlas User Guide](https://pointonenav.com/wp-content/uploads/2024/06/Atlas-User-Guide.pdf))。[Point One SP1](https://store.pointonenav.com/products/sp1-high-precision-gnss-antenna) 出厂即与该电源匹配，是推荐默认选择。BOM 中所用 ArduSimple "Magnetic Stand for Survey GNSS Antenna" 采用 **5/8"-11 UNC** 螺纹（测量杆的标准接口），各候选天线与该支架的螺纹匹配情况如下表所示。
>
> | 天线 | 频段 | 与 ArduSimple 支架的螺纹匹配 |
> |------|------|-------------------------------|
> | [Point One SP1](https://store.pointonenav.com/products/sp1-high-precision-gnss-antenna) | L1 / L2 / L5 | **直接匹配。** SP1 套件出厂自带磁吸底座 + 75 mm 立柱，带 5/8"-11 UNC 螺纹。 |
> | [Tallysman TW3972](https://www.tallysman.com/product/tw3972-triple-band-gnss-antenna-with-l-band/) | L1 / L2 / L5 + L-band | **需转接器。** 原生为穿孔 / 平底安装，需加一个 Calian / Tallysman [Pipe Mount Adapter PN 23-0065-0](https://www.calian.com/advanced-technologies/gnss_product/pipe-mount-adapter-screw-compression-pn-23-0065-0/)（或 PN 23-0040-0 L 形支架）以引出 5/8"-11 UNC 螺纹。 |
> | [Harxon HX-CSX601A](https://en.harxon.com/product/detail/99) | GPS/GLONASS/Galileo/BeiDou 多频段 | **请以最新数据手册为准。** 测量级，TNC-F 接头。公开资料对其螺纹是 5/8"-11 UNC（测量行业标准）还是 5/8"-12 说法不一；下单前请核对 Harxon 当前发布的产品手册再决定。 |
>
> 接入前请核对每个候选天线的 LNA 电压 / 电流是否与 3.3 V 兼容；若该天线已由其它电源供电，需加一个 DC block 串联保护。**请勿**直接借用某些 LTE + GNSS 组合天线（如 Peplink 系列）的 GNSS 馈线 —— 即使是较高端的组合天线，在频段覆盖与噪声系数上也明显逊于 SP1，会显著降低 RTK 解算质量。

> ### ⭐ 双 GNSS 天线 —— 强烈推荐
>
> 在已知偏置位置加装第二根 SP1 类多频段 GNSS 天线，可以把 RTK-fixed 状态升级为**无漂移航向参考**（精度通常 0.1°–1°，取决于基线长度）。如果只装一根天线，车辆航向就只能从 IMU 陀螺仪积分得来，会随时间漂移 —— 这正是 GLIM++ 不得不去补偿的多圈 z-drift / yaw-drift 失败模式。
>
> > **⚠ 重要 —— 3D 参考设计中仅包含一根 GNSS 天线支架。** [`3D files/`](3D%20files/) 中的 SCAD 模型与 [`3D files/README.md`](3D%20files/README.md) 中的 BOM 只覆盖一根位于 Atlas Duo 导航中心正上方的磁吸式测量支架。**副天线支架的几何形状是部署级别的设计**，因为最佳基线取决于车辆 —— 轿车顶、赛车壳、测量车驾驶室所适合的安装位置各不相同。每个真实部署都应另外加装一根副天线 —— 本项目 3D 文件只是起点，不是最终的机械方案。副天线必须刚性固定（金属底板或 3D 打印延伸件螺接到穹顶上，**不要**用胶带或磁吸临时固定），否则行驶过程中基线会形变；测量 `imu_link → gnss_antenna_secondary_link` 偏置到厘米级精度，按下文步骤填入 [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml)。
>
> **在本项目中启用双天线的步骤：**
>
> 1. 把第二根 SP1（或同级多频段天线）安装到与主天线已知相对位置的位置上。**车辆顶置安装推荐基线为 1.0–1.5 m**；最低 0.5 m 才能获得可用精度。
> 2. 接入 Atlas Duo 的副 GNSS RF 输入（用天线分配器或 Atlas Duo 单元自带的副端口，如果有的话）。在 Atlas Duo Web UI 中开启双天线航向模式。
> 3. **编辑 [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml)**，记录副天线的实际安装位置。对应字段如下：
>
>    ```yaml
>    # 摘自 config/sensor_dome_tf.yaml —— 副天线条目。
>    # 把平移 (0, 0, 0) 替换为从 imu_link（Atlas Duo CoN）到副天线相位中心
>    # 的实测偏置，单位米。旋转保持单位四元数 —— 基线只是平移。
>    - frame_id: imu_link
>      child_frame_id: gnss_antenna_secondary_link
>      translation:
>        x: 0.0      # ← 前后偏置实测值，+X 向前，米
>        y: 1.200    # ← 左右偏置实测值，+Y 向左，米（示例：左侧 1.2 m）
>        z: 0.273    # ← 上下偏置实测值，+Z 向上，米（通常与主天线高度一致）
>      rotation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
>    ```
>
>    文件中默认的 `(0, 0, 0)` 是显式禁用双天线模式的**哨兵值** —— 任何实际安装都会有非零偏置。用卷尺测量到 ±1 cm 即可；启动脚本会按 `σ ≈ atan2(0.01 m, baseline_m)` 把基线换算成预期 RTK-fixed 航向 σ，例如 1.2 m 基线对应 σ ≈ 0.48°（远低于下文运行期合理性检查的阈值）。此处填入的偏置**还必须**写入 Atlas Duo 固件的 `gnss_lever_arm_secondary`（见下文杆臂补偿说明）—— 两处必须一致。
> 4. 重新生成 URDF：`cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py`。脚本会打印 `GNSS antenna mode: DUAL` 以及基线长度和预期的 RTK-fixed 航向 σ。如果输出是 `GNSS antenna mode: SINGLE`，说明第 3 步的平移仍是哨兵值，URDF 生成器拒绝切换到双天线模式。
>
> **若只装单根天线作为回退：** 保留 `gnss_antenna_secondary_link` 的默认 `(0, 0, 0)` 即可。启动助手会识别该哨兵并报告单天线硬件；主天线仍为 GLIM++ 初始化和整段会话的 GNSS 位置因子提供 RTK 位置，只缺少无漂移航向。仓库算法配置已经与该硬件默认值一致：权威配置 [`GLIM_plusplus/glim/config/config_gnss_global.json`](GLIM_plusplus/glim/config/config_gnss_global.json) 中的 `enable_orientation_prior` 为 `false`。

## 传感器布局

下方示意图采用当前 SCAD 坐标约定，以 ROS REP 103 车体坐标系（+X 前、+Y 左、+Z 上）标注每个传感器相对于 Atlas Duo 导航中心（原点）的位置。

![俯视图](3D%20files/sensor_dome_layout_top.jpg)

*俯视图。LiDAR 1–3 朝外指向 0° / 120° / 240°；摄像头 1–2 在 LiDAR 1 两侧形成 110 mm 前向立体基线；摄像头 3–4 位于后左与后右两个六边形面上。*

![等轴侧视图](3D%20files/sensor_dome_layout_iso.jpg)

*等轴侧视图，展示双层穹顶：LiDAR 悬挂在 L2 下表面，摄像头位于 L2 上表面，GNSS 天线通过磁吸支架升出板中心上方。*

## 仓库结构

```
3D files/                  OpenSCAD 模型、READMEs、导出的 STL 文件
  sensor_dome.scad         参数化 OpenSCAD 源 (v17r) —— 两件式螺栓装配版
                           (L1 + L2 由 10 颗 M6 BHCS 螺栓连接)
  sensor_dome_unibody.scad 一体式 (unibody) 变体 —— 包装 sensor_dome.scad，
                           将 L1 + 支柱 + L2 融合为单一可打印件
  README.md                详细设计规范 (英文)
  README_zh.md             详细设计规范 (中文)
  *.stl                    已导出的可打印网格 (L1, L2)

Documents/           组件数据手册
  Pointonenav-assembly-atlas-duo.pdf
  Seyond-Robin-W1G-Manual.pdf
  e-con_RouteCAM_CU25_IP67_Datasheet.pdf
  e-con_RouteCAM_CU25_IP67_Lens_Datasheet.pdf
  Datasheet_Magnetic_Stand_for_Survey_GNSS_Antenna.pdf

config/              项目级配置 (单一信息源)
  sensor_dome_tf.yaml     静态 TF 变换 (所有传感器 → imu_link)
  network_config.yaml     网卡、主机 IP、传感器 IP、DHCP 池
  load_network_config.sh  由 setup_*.sh source 用以导出 NETCFG_*

PTP_sync/                    一次性的主机 + 传感器时间同步搭建
  1_install_packages.sh      apt 依赖、RT 权限、ROS 2 Humble/Jazzy
  2_configure_host_network.sh 主机 NIC 静态 IP、硬件时间戳检测
  3_setup_ins_to_pc_sync.sh   gpsd + chrony + ptp4l GM + phc2sys + p1 驱动
  4_setup_lidar_ptp.sh       Robin W PTP slave + Seyond ROS 2 驱动
  5_setup_camera_ptp.sh      RouteCAM PTP slave + Aravis（仅 Tier 2）
  provision_robin_w_multiunit.sh 每台 LiDAR 一次性 IP 重分配 + UDP 端口
  README.md                  架构、验证、故障排查

recording/           运行期数据采集 + Foxglove 仪表盘
  sensor_recorder.py      检测传感器 → 验证时钟同步 → 录制 .mcap
  rate_monitor.py         每个 topic 的频率发布到 /sensor_dome/rates
  sensor_config.yaml      IP、frame_id、同步阈值、驱动启动命令
  foxglove/               预制的 Foxglove Studio 布局
  launch/                 静态 TF 启动助手
  data/                   会话录制的默认输出根目录
  README.md               架构和运行流程

GLIM_plusplus/                LiDAR-Inertial 建图 (koide3/glim 的 fork)
  config/                 sensor_dome.urdf + URDF 生成器
  launch/                 hitch_sensor_dome.launch.py
  glim/                   上游 GLIM 核心 (附项目调参)
  glim_ext/               上游扩展模块 (GNSS 先验已重新启用)
  glim_ros2/              ROS 2 封装（多 LiDAR 拼接、INS/GNSS bag 输入）
  README.md               Fork 声明、集成说明、多圈修复

GICP_plusplus/                LiDAR-only 定位 (vectr-ucla/DLIO 的 fork)
  cfg/                    localization.yaml (race) + localization_safe.yaml
  launch/                 localization_with_tf.launch.py + systemd 单元
  scripts/                merge_glim_submaps.py + 诊断工具
  include/, src/          small_gicp 适配层 + 定位器 + nav_sat_gated_odom
  thirdparty/small_gicp/  内置的纯头文件配准后端
  README.md               Fork 声明、双模式设计、race vs safe
```

## 快速开始

安装 [OpenSCAD](https://openscad.org/)，并在下面两种打印方案中任选其一。两者最终几何完全相同，区别只在于穹顶是两片螺栓装配，还是融合为单件打印。

### 方案 A —— 两件式螺栓装配 (默认，推荐)

原始设计。L1（底板 + 6 根支撑，整体高 139 mm）与 L2（顶板，厚
12 mm）分开打印，再由 10 颗 M6×25 mm BHCS 螺栓在支撑顶端连接。
打印更快、无需支撑材料、便于拆卸维护，单独更换其中一块板时也无需重新打印另一块。

1. 在 OpenSCAD 中打开 `3D files/sensor_dome.scad`。
2. 设置 `RENDER_MODE = 1`，渲染 (F6)，导出 STL (F7) —— 这是 **L1 打印件**。
3. 设置 `RENDER_MODE = 2`，渲染并导出 —— 这是 **L2 打印件**。
4. 在 305 × 305 mm 打印床上打印两件 (PETG 或 ABS，填充 50–60%)。两件都不需要支撑：支柱是垂直墙体，L2 是水平平板。
5. 依照 [`3D files/README.md`](3D%20files/README.md) 中的 BOM 装配。

### 方案 B —— 一体式 (unibody) 单件打印

适用于希望 L1 与 L2 之间没有任何螺栓接合的部署 —— 抗扭刚度略高、长期使用不会出现螺栓松动、切片时只处理一个文件。代价：打印时间长，L2 下方需要相当数量的支撑材料。

1. 在 OpenSCAD 中打开 `3D files/sensor_dome_unibody.scad`。该文件通过 `include` 引用了 `sensor_dome.scad`，所以所有参数 (支柱高度、板尺寸、传感器孔位等) 都会自动与两件式版本保持同步。
2. 渲染（F6）并导出 STL（F7）—— 这是**整体穹顶打印件**（约 280 × 300 × 151 mm，在默认 L1 朝下的方向下可放进 305 × 305 × 300 mm 床面）。
3. **必须开启支撑材料。** L2 顶板悬挂在支柱顶端之间，跨度覆盖 280 × 300 mm 的近乎空腔区域。请使用 **树形 / 有机支撑** (PrusaSlicer 2.6+、Bambu Studio、Cura "Tree")，**仅** 放在 L2 下表面、不要放进支柱之间。常规参数下约需 1.5–2 kg PETG 支撑材料，打印时间约 28–36 小时；这种高度下溶解性支撑并不实用。本方案不需要 BOM 中的 12 颗 M6 BHCS 支柱螺栓。

完整设计规范、BOM 和装配说明见 [`3D files/README.md`](3D%20files/README.md)。

## 数据采集与可视化

完成穹顶组装与传感器接线之后，由两个目录把硬件转化为可用数据集：

1. **一次性搭建** —— 运行 [`PTP_sync/`](PTP_sync/) 中的脚本，把主机配置成 GPS 校准的 PTP 主时钟，并在每台 LiDAR 与每台摄像头上启用 IEEE 1588 PTP。完成后所有传感器共享亚微秒级的 GPS 时间基准。

2. **逐次采集** —— 同时运行 [`adapter/`](adapter/) 与 [`recording/sensor_recorder.py`](recording/sensor_recorder.py)。录制器自动检测已连接的传感器、验证时钟同步链路和 adapter 的 Fixed-only 里程计，并把 GNSS / IMU / LiDAR / 摄像头数据流录制成 Foxglove 原生的 MCAP rosbag。录制器不会代替你启动 adapter。

```bash
# 在 PTP_sync/ 已经跑过一次之后：
python3 recording/sensor_recorder.py
# 然后在 Foxglove 中：Open Connection → ws://localhost:8765
#                     Layouts → Import → recording/foxglove/sensor_dome_layout.json
```

架构图与运行流程详见 [`recording/README.md`](recording/README.md)。

## 建图 (GLIM++)

针对 SLAM 与 3D 建图，本项目搭载 **GLIM++**，一个对 **GLIM** 做了深度修改的 fork —— 上游 *Graph-based LiDAR-Inertial Mapping* 由 AIST 的 Kenji Koide 等人开发，仓库地址 <https://github.com/koide3/glim>。本 fork 位于 [`GLIM_plusplus/`](GLIM_plusplus/)（双加号意在提示这并非原版 GLIM）。在高层视角下，GLIM++ 与上游的差异分为八个类别：

1. **传感器适配** —— topic、frame、字段名开箱即为 Hitch Sensor Dome（3× Robin W + Atlas Duo + 4× RouteCAM）配置：`/imu/data`、`/robin_w_*/points`，frame 采用 `config/sensor_dome_tf.yaml` 中的 `imu_link`/`lidar_front_link`。
2. **车辆无关主体坐标系** —— GLIM++ 的地图锚定在 `imu_link`（Atlas Duo 导航中心），可跨车辆平台复用。下游定位节点把位姿报告在 `base_link`（ROS 标准车体坐标系）下，由 [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) 中的 `imu_link → base_link` 静态变换桥接 —— 默认 identity，按车辆需要重写（后轴、底盘几何中心等），无需重新建图。
3. **户外 / 车辆尺度调参** —— 24 项参数变更（放宽 IMU 噪声、增大 voxel、加长初始化窗口、提高 sub-mapping 密度），针对高速公路 / 赛道 / 车辆机动场景。
4. **多圈回环修复** —— 拓宽 VGICP 收敛域、放宽隐式回环阈值、使用协方差感知 GNSS 先验并加固 world/UTM 拟合，防止经典的"第二圈轨迹翘向天空"现象。
5. **C++ 重写初始化** —— 移除"由加速度计估计重力"路径；优化器现在必须由外部 INS 位姿启动。这使得从运动状态开始的录制（中途重启、赛道重放、被截过的 bag）也能正常使用。
6. **初始位姿的 RTK-fixed 准入门控** —— 三阶段门控（NavSatFix 状态、协方差、多采样稳定性），未通过时打印粗体红色 CLI 警告。
7. **RTK 门控的 GNSS 因子桥** —— 整段会话期间向全局图持续注入 GNSS 软先验因子，但仅在 RTK 锁定时段；隧道期间静默暂停，重新锁定后自动恢复。
8. **GNSS 航向先验（仅适用于双天线）** —— 当配置为双天线 RTK 时，可使用 `PoseRotationPrior` 因子把每个 submap 的 yaw 拉向 RTK 测得的航向。仓库默认关闭；安装并配置副天线后才显式开启。

URDF 生成器与 `ros2 launch` 助手补全了集成。完整的逐文件变更日志、上游致谢、license 保留、引用方式与编译说明请见 [`GLIM_plusplus/README.md`](GLIM_plusplus/README.md)。

> ### ⚠ 运行要求 —— 启动会话前必须有 RTK-fixed 的 GNSS
>
> **GLIM++ 把 Atlas Duo 的 RTK-fixed GNSS 位姿与速度作为初始化的 ground truth。** 它取代了上游"开机段 IMU 静止"的要求，换成一个更明确的硬性条件：**没有 Atlas Duo 报告 RTK-fixed 状态、协方差达到厘米级，就无法开启建图会话。** GLIM++ 在 C++ 中通过三阶段门控（状态、协方差、多采样稳定性）强制执行，等待期间会周期性打印粗体红色警告。它不会自动 abort —— 但门控未通过时也不会开始构造任何地图因子。
>
> 实战意义：
>
> - **预留 RTK 收敛时间。** 启动 GLIM++ 之前，把车停在天空开阔的位置等 RTK 锁定。室外典型收敛时间为 30–120 秒；条件较差时更久。启动前务必在 Atlas Duo 的 web UI 中确认。
> - **NTRIP 校正必须畅通。** Atlas Duo 经蜂窝路由器接通 NTRIP caster 的链路（见 [`PTP_sync/README.md`](PTP_sync/README.md) §3.1）必须工作；否则无法达到 RTK-fixed。
> - **会话进行中遇到隧道 / 城市峡谷不影响。** 后续每条消息的 RTK 门控只在失锁时段静默暂停因子发布，重新锁定后自动恢复，**不会重启会话** —— RTK 硬性要求只针对*初始位姿*。
> - **完全没有 RTK** 的场景（既没有基站也没有 NTRIP）属于**仅供诊断**的配置，不是受支持的建图模式。做法是在 `config_ros.json` 中指向连续 INS 源，并设置 `"ins_require_rtk_fixed": false` 与 `"ins_max_position_stddev": 0.5` —— 注意这些是配置键，不是启动参数。地图依然可用，但世界坐标系锚点的精度从厘米级退化到米级，运行结束时报告为 `rtk_origin_only` 而非 `rtk_anchored`。生产 profile 始终保持 Fixed-only。另需注意：单纯放宽 NavSatFix 协方差阈值**打不开**这道门 —— 门控读的是 Atlas 的解算类型（solution type），所以无论 Float 解上报的协方差多好看，都会被拒绝。

> ### ⚙ GLIM++ GNSS 天线杆臂补偿 —— 默认关闭（仅适用于 Atlas Duo）
>
> **GLIM++ 默认关闭 GNSS 天线相对 IMU 的杆臂（lever-arm）补偿。** Atlas Duo 是**紧耦合 GNSS+INS**，并不是裸 GNSS 接收机：其内部融合引擎已经把每根天线的 RTK 观测解算到设备 IMU 坐标系下，对外发布的 `/pose` 与 `/odom` 是**位于 IMU 原点**的位姿。若在 GLIM++ 中再做一次杆臂补偿，会重复修正并悄悄把地图偏置。
>
> **该选择只有在 Atlas Duo 内部的天线偏置参数与穹顶实际安装一致时才成立。** Atlas Duo 调试阶段，必须在单元配置（Atlas Duo Web UI → device configuration）中以设备 IMU 坐标系为基准、按米为单位写入天线偏置：
>
> - **`gnss_lever_arm_primary`** 必须等于 [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) 中 `imu_link` 到 `gnss_antenna_primary_link` 的向量 —— 目前为 `(0, 0, 0.273)`。
> - **`gnss_lever_arm_secondary`**（仅当接入副天线时）必须等于 `imu_link` 到 `gnss_antenna_secondary_link` 的向量。
>
> 如果 Atlas 固件中的这两个值填错，Atlas 自身的 RTK-fixed 解算就会被旋转后的杆臂偏置污染 —— **GLIM++ 这一侧再做任何外部补偿都无法挽回**，只能在 Atlas 内重新写入配置并重录会话。
>
> **若将 Atlas Duo 替换为非紧耦合的 GNSS**（例如 SwiftNav / u-blox / NovAtel 等无板载 INS 融合的裸 RTK 接收机，或任何把位置以**天线**参考点而非 IMU 原点发布的松耦合方案），则必须**在 GLIM++ 中开启杆臂补偿** —— 因为天线到 IMU 的修正不再由上游设备完成。建议的实现方式：在 wrapper 的 GNSS 因子桥（`glim_ros.cpp` 的 `try_publish_gnss_factor`）中读取 [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) 的 `imu_link → gnss_antenna_primary_link`，在消息进入 `libgnss_global.so` 之前做 `p_imu_utm = p_antenna_utm − R_world_imu · t_imu_gnss` 修正。

> ### 🧭 GLIM++ GNSS 航向先验 —— 默认关闭
>
> 仓库默认是单天线参考设计：`gnss_antenna_secondary_link` 使用
> `(0, 0, 0)` 哨兵，权威配置
> [`GLIM_plusplus/glim/config/config_gnss_global.json`](GLIM_plusplus/glim/config/config_gnss_global.json)
> 中的 `enable_orientation_prior` 为 `false`。单天线运行无需修改配置。
>
> 只有在刚性安装副天线、测量两根天线杆臂并在 Atlas Duo 中启用双天线航向后，
> 才能开启 `PoseRotationPrior`。把实测副天线变换写入
> [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml)，重新生成 URDF，
> 在 [`GLIM_plusplus/glim/config/config_ros.json`](GLIM_plusplus/glim/config/config_ros.json)
> 中设置 `dual_antenna_enabled`、基线和航向不确定度，最后在权威 GNSS
> 配置中把 `enable_orientation_prior` 改为 `true`。航向因子只消费
> adapter 的 RTK Fixed 样本，并按配置的 yaw sigma 上限拒绝劣化航向。
>
> ### 🛡 航向先验配置错误的三层防御
>
> 航向先验是双天线下最有用的功能，但也是三处配置（物理安装、本仓库的配置文件、Atlas 固件）若有一处错配时最隐蔽的失败点。GLIM++ 设置了三道独立的检查：
>
> 1. **Atlas 固件前置条件（文档约定）。** Atlas Duo 内部的 `gnss_lever_arm_secondary` 与双天线航向模式必须在 Atlas Web UI 中按物理安装方式正确配置。这一项无法由本仓库的代码自动验证 —— 只能在 Atlas 调试期间手动确认。具体参数对应关系见上文的 GNSS 杆臂补偿说明。
> 2. **启动期一致性检查（自动）。** `hitch_sensor_dome.launch.py` 同时读取 [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) 和 [`GLIM_plusplus/glim/config/config_gnss_global.json`](GLIM_plusplus/glim/config/config_gnss_global.json)，若物理双天线意图与 `enable_orientation_prior` 不一致则告警。该助手只做诊断，不会改写 JSON，也不会启动 GLIM 建图进程。
> 3. **运行期 yaw σ 合理性检查（自动）。** GLIM++ 启动后，C++ wrapper 会读取每条 `/odom` 消息中 Atlas 自己上报的 yaw σ（pose 协方差中的对应项），与我们根据双天线基线长度估计出的预期 σ 做对比。前约 20 条样本采集完毕后，若 Atlas 持续上报远宽于预期的 σ，打印粗体黄色一次性警告，说明 Atlas 固件很可能**没有**真正进入双天线航向模式（即便本仓库的配置说它在）。该检查要求 `ins_odom_topic` 已经接入（Odometry 携带协方差，PoseStamped 不携带）。检查通过时会输出一行 info 级别的确认日志，确认双天线航向已生效。

```bash
# (一次性) 从 sensor_dome_tf.yaml 生成 sensor_dome.urdf
cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py

# 启动 TF、配置一致性检查与可视化支持节点（不启动建图）：
ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py

# 生产建图使用已采集的 MCAP 包（bag 中必须包含 adapter 的
# /gps_p1/filtered_odom_rtk_fixed 与 /gps_p1/fix）：
ros2 run glim_ros glim_rosbag recording/data/session_<ts>/rosbag2 \
    --ros-args -p config_path:=GLIM_plusplus/glim/config \
                -p dump_path:=glim_maps/session_<ts>
```

## 定位 (GICP++)

针对在线 scan-to-map 定位（对预建 PCD 地图），本项目搭载 **GICP++**，一个对 **DLIO**（*Direct LiDAR-Inertial Odometry*，UCLA VECTR Lab 的 Kenny J. Chen / Ryan Nemiroff / Brett T. Lopez，上游 <https://github.com/vectr-ucla/direct_lidar_inertial_odometry>）做了深度修改的 fork。本 fork 位于 [`GICP_plusplus/`](GICP_plusplus/)（双加号意在提示这并非原版 DLIO）。GICP++ 在启动时可选 **两种运行模式**：

- **🏁 赛车模式（默认）** —— 40 m 裁剪、32 次 GICP 迭代、yaw-rate 自适应 Kp/Kq 衰减和较少日志。
- **🛡 安全模式** —— 100 m 裁剪、上游严格的 128 次 GICP 迭代、更紧的收敛阈值和完整诊断。

LiDAR 拓扑与调参模式独立。`lidar_mode:=front_only` 是所有 profile
的启动默认值；`mode:=safe` 本身不会开启后置 LiDAR。需要完整穹顶输入时显式使用
`lidar_mode:=three_lidar`。`lidar_mode:=auto` 是显式选择的 profile
跟随模式：safe 解析为三台，race/custom 解析为前置一台。

相对上游 VECTR DLIO 的项目改进：

1. **针对 Robin W + Atlas Duo** —— topic / frame / URDF 默认值开箱适配 Hitch 穹顶（低延迟实时 IMU `/imu/data`、adapter 的 RTK-fixed-only `/gps_p1/filtered_odom_rtk_fixed`、`/robin_w_*/points`；frame 来自 `config/sensor_dome_tf.yaml` 的 `base_link`/`imu_link`/`lidar_front_link`）。
2. **固定解门控与失效回退** —— 生产路径直接使用 adapter 按 FusionEngine `solution_type == RTK_FIXED` 发布的固定解里程计。协方差必须为有限、正值且不超过阈值；RTK float、过期/未知质量或无解时不允许初始化或恢复跳变，而是继续使用 LiDAR + IMU。`nav_sat_gated_odom` 仅用于旧 bag 的 REP-145 兼容路径，因为 `STATUS_GBAS_FIX` 本身无法可靠区分 float 与 fixed。
3. **调参模式与 LiDAR 模式独立** —— `mode:=race|safe|custom` 选择参数 profile，`lidar_mode:=front_only|three_lidar|auto` 选择传感器。启动默认前置一台；两个互斥 systemd 单元分别显式固定 race/front 与 safe/three-lidar。
4. **GLIM++ ENU 地图桥接** —— 规范导出器 `GLIM_plusplus/scripts/export_glim_dump_to_pcd.py` 精确读取 compact 点格式，依次应用 `T_world_origin` 与 `inverse(T_world_utm)`，输出 surveyed local-ENU PCD 及 `<map>.manifest.yaml`。GICP++ 默认要求 manifest，并在处理点云前把其 datum 与 adapter 的 transient-local `/gps_p1/local_enu_origin` 比对；旧脚本仅保留为该导出器的兼容包装。
5. **GICP 预热** —— 启动时即急切构建 kd-tree + 一次 dummy align，把 OpenMP 线程池启动、Eigen JIT、source 端 kd-tree 分配等首次接触开销在第一次真实扫描之前烧掉。
6. **Yaw-rate 自适应观测器** —— 几何观测器中的 `Kp` 与 `Kq` 在车体系 yaw rate 较高时自动衰减，让 IMU 预测在转弯入弯（此时 GICP 最易滑动）瞬间占主导。
7. **IMU 标定运动方差门** —— 若车辆运动（σ‖a‖ > 0.10 m/s²），拒绝静止偏置标定窗口；回退到 gt_odom 路径的 RTK 驱动标定。
8. **操作员侧健康检查** —— 若 10 秒后仍未收到 `gt_odom`，发出粗体黄色一次性警告；dead-reckoning 阶段输出节流诊断日志。
9. **base_link / imu_link 拆分** —— 即使 GLIM++ 把地图锚定在 `imu_link`，定位器仍把位姿报告在 `base_link`（可通过 [`config/sensor_dome_tf.yaml`](config/sensor_dome_tf.yaml) 按车辆配置）。同一份地图可跨多个车辆框架使用。

```bash
# 1. 使用与 adapter 完全相同的 surveyed local-ENU 原点导出地图。
#    命令同时生成 /tmp/race_map.pcd.manifest.yaml。
ENU_ORIGIN="37.87150000,-122.27300000,52.125"
python3 GLIM_plusplus/scripts/export_glim_dump_to_pcd.py \
    /tmp/dump /tmp/race_map.pcd \
    --frame enu --enu-origin "${ENU_ORIGIN}" --voxel-size 0.4

# 2. 在赛车模式（默认）下启动定位器：
ros2 launch gicp_localization localization_with_tf.launch.py \
    map_path:=/tmp/race_map.pcd \
    local_enu_origin:="${ENU_ORIGIN}"

# 或安全模式（3× LiDAR、全覆盖、全 debug）：
ros2 launch gicp_localization localization_with_tf.launch.py \
    mode:=safe \
    map_path:=/tmp/race_map.pcd \
    local_enu_origin:="${ENU_ORIGIN}"
```

完整的逐文件变更日志、race vs safe 双模式参数对比表、systemd 单元安装、延迟预算分解参见 [`GICP_plusplus/README.md`](GICP_plusplus/README.md)。

## 坐标系 (ROS REP 103)

- **+X** = 前向，**+Y** = 左，**+Z** = 上
- **原点** = Atlas Duo 导航中心 (CoN)

## Credits

本项目由 **Dr. Allen Y. Yang** (Hitch Interactive · 加州大学伯克利分校) 设计并维护。

实现测试与现场验证由 **Berkeley AI Racing Tech** 团队完成：来自 UC Berkeley（按姓氏字母顺序）—— Bryan Chang、Logan Kinajil-Moran、Moises Lopez Mendoza、Gary Passon、Tanishaa Viral Shah、Joshua Sun、Jovan Yap；来自 UC San Diego —— Kevin Shin。

如在衍生工作中复用本仓库的机械设计、ROS 2 TF 配置、PTP 同步流水线，或采集 / 可视化工具，请引用：

> Yang, A. Y. *Hitch Sensor Dome: a 3D-printable modular multi-sensor mount for vehicle-roof mapping.* GitHub repository, 2026.

感谢 OpenSCAD、ROS 2、linuxptp、chrony、Aravis、Foxglove、MCAP 等社区，本项目基于这些开源工具搭建。建图管线建立在 **GLIM** 之上，作者为 AIST 的 Kenji Koide、Masashi Yokozuka、Shuji Oishi、Atsuhiko Banno —— 完整的上游致谢、许可证保留与引用方式见 [`GLIM_plusplus/README.md`](GLIM_plusplus/README.md)。

## License

详见 [LICENSE](LICENSE)。
