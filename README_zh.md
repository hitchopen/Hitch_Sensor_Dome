# Hitch Sensor Dome

可 3D 打印的模块化传感器穹顶，通过吸盘式相机支架将多传感器测绘平台安装到车顶。

## 传感器

- 3× Seyond Robin W LiDAR —— 单台 120° 水平视场，按 120° 间隔排布，实现 360° 环视
- 1× Point One Nav Atlas Duo INS —— 居中安装，导航中心 (CoN) 与几何原点重合
- 1× **Point One SP1** 多频段 (L1/L2/L5) GNSS 天线 —— 安装在 ArduSimple 测量支架上，置于 CoN 正上方
- 4× e-con RouteCAM_P_CU25_CXLC_IP67 摄像头 —— 前向立体对 (104 mm 基线) + 后向对称对

> **GNSS 天线说明。** Atlas Duo 的 GNSS 端口为有源 LNA 提供 **3.3 V DC 偏置** (见 [Point One Atlas User Guide](https://pointonenav.com/wp-content/uploads/2024/06/Atlas-User-Guide.pdf))。[Point One SP1](https://store.pointonenav.com/products/sp1-high-precision-gnss-antenna) 出厂即与该电源匹配，是推荐默认选择。BOM 中所用 ArduSimple "Magnetic Stand for Survey GNSS Antenna" 采用 **5/8"-11 UNC** 螺纹（测量杆的标准接口），各候选天线与该支架的螺纹匹配情况如下表所示。
>
> | 天线 | 频段 | 与 ArduSimple 支架的螺纹匹配 |
> |------|------|-------------------------------|
> | [Point One SP1](https://store.pointonenav.com/products/sp1-high-precision-gnss-antenna) | L1 / L2 / L5 | **直接匹配。** SP1 套件出厂自带磁吸底座 + 75 mm 立柱，带 5/8"-11 UNC 螺纹。 |
> | [Tallysman TW3972](https://www.tallysman.com/product/tw3972-triple-band-gnss-antenna-with-l-band/) | L1 / L2 / L5 + L-band | **需转接器。** 原生为穿孔 / 平底安装，需加一个 Calian / Tallysman [Pipe Mount Adapter PN 23-0065-0](https://www.calian.com/advanced-technologies/gnss_product/pipe-mount-adapter-screw-compression-pn-23-0065-0/)（或 PN 23-0040-0 L 形支架）以引出 5/8"-11 UNC 螺纹。 |
> | [Harxon HX-CSX601A](https://en.harxon.com/product/detail/99) | GPS/GLONASS/Galileo/BeiDou 多频段 | **请以最新数据手册为准。** 测量级，TNC-F 接头。公开资料对其螺纹是 5/8"-11 UNC（测量行业标准）还是 5/8"-12 说法不一；下单前请核对 Harxon 当前发布的产品手册再决定。 |
>
> 接入前请核对每个候选天线的 LNA 电压 / 电流是否与 3.3 V 兼容；若该天线已由其它电源供电，需加一个 DC block 串联保护。**请勿**直接借用某些 LTE + GNSS 组合天线（如 Peplink 系列）的 GNSS 馈线 —— 即使是较高端的组合天线，在频段覆盖与噪声系数上也明显逊于 SP1，会显著降低 RTK 解算质量。

## 传感器布局

下方示意图（由 v17c SCAD 模型生成）以 ROS REP 103 车体坐标系（+X 前、+Y 左、+Z 上）标注每个传感器相对于 Atlas Duo 导航中心 (原点) 的位置。

![俯视图](3D%20files/sensor_dome_layout_top.jpg)

*俯视图。LiDAR 1–3 朝外指向 0° / 120° / 240°；摄像头 1–2 在 LiDAR 1 两侧形成 104 mm 前向立体基线；摄像头 3–4 位于后左与后右两个六边形面上。*

![等轴侧视图](3D%20files/sensor_dome_layout_iso.jpg)

*等轴侧视图，展示双层穹顶：LiDAR 悬挂在 L2 下表面，摄像头位于 L2 上表面，GNSS 天线通过磁吸支架升出板中心上方。*

## 仓库结构

```
3D files/           OpenSCAD 模型、READMEs、导出的 STL 文件
  sensor_dome.scad   参数化 OpenSCAD 源 (v17c)
  README.md          详细设计规范 (英文)
  README_zh.md       详细设计规范 (中文)
  *.stl              已导出的可打印网格 (L1, L2)

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

PTP_sync/            一次性的主机 + 传感器时间同步搭建
  setup_ubuntu_sync.sh    GPS 校准的 PTP 主时钟 (gpsd → chrony → ptp4l)
  setup_robin_w_sync.sh   在 Seyond Robin W LiDAR 上启用 PTP
  setup_camera_sync.sh    在 RouteCAM 摄像头上启用 IEEE 1588 PTP
  README.md               架构、验证、故障排查

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
  docs/                   多圈回环调试指南
  glim/                   上游 GLIM 核心 (附项目调参)
  glim_ext/               上游扩展模块 (GNSS 先验已重新启用)
  glim_ros2/              上游 ROS 2 封装 (未改动)
  README.md               Fork 声明、集成说明、多圈修复
```

## 快速开始

1. 安装 [OpenSCAD](https://openscad.org/)
2. 打开 `3D files/sensor_dome.scad`
3. 设置 `RENDER_MODE = 1` 渲染 Level 1，`RENDER_MODE = 2` 渲染 Level 2
4. 渲染 (F6) 并导出 STL (F7)
5. 在 305 × 305 mm 打印床上打印两个零件 (PETG 或 ABS，填充 50–60%)

完整设计规范、BOM 和装配说明见 [`3D files/README.md`](3D%20files/README.md)。

## 数据采集与可视化

完成穹顶组装与传感器接线之后，由两个目录把硬件转化为可用数据集：

1. **一次性搭建** —— 运行 [`PTP_sync/`](PTP_sync/) 中的脚本，把主机配置成 GPS 校准的 PTP 主时钟，并在每台 LiDAR 与每台摄像头上启用 IEEE 1588 PTP。完成后所有传感器共享亚微秒级的 GPS 时间基准。

2. **逐次采集** —— 运行 [`recording/sensor_recorder.py`](recording/sensor_recorder.py) 自动检测已连接的传感器、验证时钟同步链路、并把 GNSS / IMU / LiDAR / 摄像头数据流录制成 Foxglove 原生的 MCAP rosbag。配套的 Foxglove Studio 布局会在采集过程中实时显示三路 Robin W 点云在 IMU 坐标系下的叠加、四个摄像头视图、GNSS 地图、IMU 曲线，以及每个 topic 的实时帧率。

```bash
# 在 PTP_sync/ 已经跑过一次之后：
sudo python3 recording/sensor_recorder.py
# 然后在 Foxglove 中：Open Connection → ws://localhost:8765
#                     Layouts → Import → recording/foxglove/sensor_dome_layout.json
```

架构图与运行流程详见 [`recording/README.md`](recording/README.md)。

## 建图 (GLIM++)

针对 SLAM 与 3D 建图，本项目搭载 **GLIM++**，一个对 **GLIM** 做了深度修改的 fork —— 上游 *Graph-based LiDAR-Inertial Mapping* 由 AIST 的 Kenji Koide 等人开发，仓库地址 <https://github.com/koide3/glim>。本 fork 位于 [`GLIM_plusplus/`](GLIM_plusplus/)（双加号意在提示这并非原版 GLIM）。在高层视角下，GLIM++ 与上游的差异分为七个类别：

1. **传感器适配** —— 把 topic、frame、字段名从此前的 AV-24 / Luminar 部署切换到 Hitch Sensor Dome（3× Robin W + Atlas Duo + 4× RouteCAM）。
2. **车辆无关主体坐标系** —— `base_frame_id = imu_link`，地图围绕 Atlas Duo 导航中心建立，可跨车辆平台复用。
3. **户外 / 车辆尺度调参** —— 24 项参数变更（放宽 IMU 噪声、增大 voxel、加长初始化窗口、提高 sub-mapping 密度），针对高速公路 / 赛道 / 车辆机动场景。
4. **多圈回环修复** —— 拓宽 VGICP 收敛域、放宽隐式回环阈值、提升 GNSS z 先验权重，防止经典的"第二圈轨迹翘向天空"现象。
5. **C++ 重写初始化** —— 移除"由加速度计估计重力"路径；优化器现在必须由外部 INS 位姿启动。这使得从运动状态开始的录制（中途重启、赛道重放、被截过的 bag）也能正常使用。
6. **初始位姿的 RTK-fixed 准入门控** —— 三阶段门控（NavSatFix 状态、协方差、多采样稳定性），未通过时打印粗体红色 CLI 警告。
7. **RTK 门控的 GNSS 因子桥** —— 整段会话期间向全局图持续注入 GNSS 软先验因子，但仅在 RTK 锁定时段；隧道期间静默暂停，重新锁定后自动恢复。

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
> - **完全没有 RTK** 的场景（既没有基站也没有 NTRIP），可通过 `ins_require_rtk_fixed:=false ins_max_position_stddev:=0.5` 放宽门控，允许 RTK-float / SBAS 初始化。地图依然可用，但世界坐标系锚点的精度从厘米级退化到米级。详见 [`GLIM_plusplus/docs/moving_start_initialization.md`](GLIM_plusplus/docs/moving_start_initialization.md) "Operating without RTK"一节。

```bash
# (一次性) 从 sensor_dome_tf.yaml 生成 sensor_dome.urdf
cd GLIM_plusplus/config && python3 generate_sensor_dome_urdf.py

# 对接采集栈做实时建图：
#   1. 把车停在天空开阔的位置等 Atlas Duo RTK-fixed 锁定。
#   2. 启动：
ros2 launch GLIM_plusplus/launch/hitch_sensor_dome.launch.py

# 或对已采集的 MCAP 包做离线建图（bag 中必须包含 /pose 与 /gps/fix）：
ros2 run glim_ros glim_rosbag recording/data/session_<ts>/rosbag2 \
    --ros-args -p config_path:=GLIM_plusplus/glim/config \
                -p dump_path:=glim_maps/session_<ts>
```

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
