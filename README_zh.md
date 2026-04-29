# Hitch Sensor Dome

可 3D 打印的模块化传感器穹顶，通过吸盘式相机支架将多传感器测绘平台安装到车顶。

## 传感器

- 3× Seyond Robin W LiDAR —— 单台 120° 水平视场，按 120° 间隔排布，实现 360° 环视
- 1× Point One Nav Atlas Duo INS —— 居中安装，导航中心 (CoN) 与几何原点重合
- 1× 测量级 GNSS 天线 —— 商用磁吸支架，置于 CoN 正上方
- 4× e-con RouteCAM_P_CU25_CXLC_IP67 摄像头 —— 前向立体对 (104 mm 基线) + 后向对称对

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

GLIM/                LiDAR-Inertial 建图 (koide3/glim 的 fork)
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

## 建图 (GLIM)

针对 SLAM 与 3D 建图，本项目集成了 **GLIM** 的 fork —— 由 AIST 的 Kenji Koide 等人开发的 *Graph-based LiDAR-Inertial Mapping*，上游仓库见 <https://github.com/koide3/glim>。本 fork 几乎完整保留了上游的 `glim`、`glim_ext`、`glim_ros2` 三个包，并在此基础上增加：

- URDF 生成器 (`GLIM/config/generate_sensor_dome_urdf.py`)，把 `config/sensor_dome_tf.yaml` 转成 GLIM 用于 `T_lidar_imu` 与多 LiDAR 拼接的 URDF —— 使同一份 TF YAML 成为采集、可视化、建图三者共享的单一信息源。
- `base_frame_id = "imu_link"`，使全局地图相对于 Atlas Duo 导航中心 (CoN) 在传感器主体坐标系中构建。下游车辆集成商再发布自己的 `imu_link → base_link` 静态变换 —— 这样地图就和具体载具解耦，可在不同车辆平台间复用。
- 启动助手 (`GLIM/launch/hitch_sensor_dome.launch.py`)，从 `sensor_dome_tf.yaml` 发布静态 TF、用项目调好的配置启动 `glim_rosnode`，并起 `foxglove_bridge` 做实时可视化。
- 针对多圈回环失败模式的修复 —— 防止"第二圈轨迹翘向天空"的现象 (更强的 GNSS z 先验、更密的 GNSS 因子、更宽的 VGICP 收敛域、更宽松的隐式回环阈值)。详见 [`GLIM/docs/multi_lap_loop_closure.md`](GLIM/docs/multi_lap_loop_closure.md)。

```bash
# (一次性) 从 sensor_dome_tf.yaml 生成 sensor_dome.urdf
cd GLIM/config && python3 generate_sensor_dome_urdf.py

# 对接采集栈做实时建图：
ros2 launch GLIM/launch/hitch_sensor_dome.launch.py

# 或对已采集的 MCAP 包做离线建图：
ros2 run glim_ros glim_rosbag recording/data/session_<ts>/rosbag2 \
    --ros-args -p config_path:=GLIM/glim/config \
                -p dump_path:=glim_maps/session_<ts>
```

完整的 fork 声明（上游致谢、许可证保留、引用方式）、集成细节、编译说明请参考 [`GLIM/README.md`](GLIM/README.md)。

## 坐标系 (ROS REP 103)

- **+X** = 前向，**+Y** = 左，**+Z** = 上
- **原点** = Atlas Duo 导航中心 (CoN)

## Credits

本项目由 **Dr. Allen Y. Yang** (Hitch Interactive · 加州大学伯克利分校) 设计并维护。

实现测试与现场验证由 **Berkeley AI Racing Tech** 团队完成：来自 UC Berkeley（按姓氏字母顺序）—— Bryan Chang、Logan Kinajil-Moran、Moises Lopez Mendoza、Gary Passon、Tanishaa Viral Shah、Joshua Sun、Jovan Yap；来自 UC San Diego —— Kevin Shin。

如在衍生工作中复用本仓库的机械设计、ROS 2 TF 配置、PTP 同步流水线，或采集 / 可视化工具，请引用：

> Yang, A. Y. *Hitch Sensor Dome: a 3D-printable modular multi-sensor mount for vehicle-roof mapping.* GitHub repository, 2026.

感谢 OpenSCAD、ROS 2、linuxptp、chrony、Aravis、Foxglove、MCAP 等社区，本项目基于这些开源工具搭建。建图管线建立在 **GLIM** 之上，作者为 AIST 的 Kenji Koide、Masashi Yokozuka、Shuji Oishi、Atsuhiko Banno —— 完整的上游致谢、许可证保留与引用方式见 [`GLIM/README.md`](GLIM/README.md)。

## License

详见 [LICENSE](LICENSE)。
