# 传感器记录系统：Point One Nav Atlas Duo + Seyond Robin W LiDAR + RouteCAM 摄像机

基于 Ubuntu 24.04 的高性能 GNSS/IMU/LiDAR/摄像机数据记录系统安装和配置指南，采用 ROS 2 Jazzy 和 PREEMPT_RT 实时内核。

**目标硬件：**
- Ubuntu 24.04 LTS 工作站（已在 Lenovo ThinkPad P1 Gen 6、Intel i9-13900H 上测试）
- Point One Nav Atlas Duo（GNSS/INS，带 PPS 输出）
- 最多 3 个 Seyond Robin W 指向性 LiDAR
- 4 个 e-con RouteCAM_P_CU25_CXLC_IP67 GigE Vision 摄像机（PoE、IEEE 1588 PTP、2MP 全局快门）

---

## 1. 设置脚本

三个脚本可自动化整个安装过程。在全新的 Ubuntu 24.04 系统上按顺序运行它们。

**第 1 步**安装所有系统前提条件、配置 GPS 校准 PTP 主时钟（gpsd → chrony → ptp4l → phc2sys）、安装 ROS 2 Jazzy 以及构建 Point One Nav 驱动程序和主机工具。

```bash
./setup_ubuntu_sync.sh --eth enp0s31f6
```

**第 2 步**在每个 Seyond Robin W LiDAR 上启用 PTP 同步并构建 Seyond ROS 2 驱动程序。当 LiDAR 通电并连接时运行。

```bash
./setup_robin_w_sync.sh --eth enp0s31f6 --ips 192.168.1.10,192.168.1.11,192.168.1.12
```

**第 3 步**安装 Aravis GigE Vision 库、在每个 RouteCAM 摄像机上启用 IEEE 1588 PTP，以及安装 ROS 2 摄像机软件包。当摄像机通过 PoE 供电时运行。

```bash
./setup_camera_sync.sh --eth enp0s31f6 --ips 192.168.1.20,192.168.1.21,192.168.1.22,192.168.1.23
```

第 2 步和第 3 步彼此独立——可以任意顺序集成 LiDAR 和摄像机。每个脚本末尾都包含自我测试，用于验证 PTP 同步质量、服务状态和传感器可达性。

---

## 2. 前提条件（手动步骤）

设置脚本处理大部分安装，但运行脚本前需要手动执行两个步骤。

### 2.1 安装 PREEMPT_RT 内核

需要 Ubuntu Pro（个人用户在最多 5 台计算机上免费）。这些脚本不会安装内核，因为它需要您的个人 Pro 令牌和重启。

```bash
# Get a free token at ubuntu.com/pro
sudo pro attach YOUR_TOKEN_HERE
sudo pro status
sudo pro enable realtime-kernel
sudo reboot
```

重启后，验证：

```bash
uname -a
# Should show: ... SMP PREEMPT_RT ...
```

**NVIDIA GPU 注意事项：** NVIDIA 内核模块（`nvidia.ko`）无法在 PREEMPT_RT 内核上加载。引导到 RT 内核时不提供 CUDA、cuDNN 或 TensorRT。使用 Intel iGPU 显示。参见[附录 C](#c-nvidia-gpu-和-rt-内核兼容性)了解推荐工作流程（在 RT 上记录，在通用内核上处理）。

### 2.2 确定您的以太网接口

找到连接到传感器网络的接口名称——您将通过 `--eth` 将其传递给所有三个脚本：

```bash
ip link show
# Look for your wired Ethernet (e.g., enp0s31f6, eth0, eno1)
```

---

## 3. 时间同步架构

跨所有传感器的精确时间同步对于传感器融合至关重要。Atlas Duo 的 GPS 校准 PPS 信号规范了主机系统时钟，然后通过 IEEE 1588 PTP 分配给所有传感器。

```
┌─────────────────────────┐
│  GPS Satellites          │
└───────────┬─────────────┘
            │ RF
┌───────────▼─────────────┐
│  Point One Nav Atlas Duo │  ← GPS 校准时钟（<20 ns 精度）
│  - PPS output (1Hz pulse)│
│  - NMEA/FusionEngine     │
│  - IMU @ 200Hz           │
└─────┬──────────┬────────┘
      │ PPS      │ NMEA serial
      │ /dev/pps0│ /dev/ttyUSB0
┌─────▼──────────▼─────────────────────────────────┐
│  Ubuntu 24.04 主机（PTP 主时钟）                 │
│                                                    │
│  gpsd ← PPS + NMEA → shared memory (SHM)          │
│  chrony ← SHM → CLOCK_REALTIME (<100 ns to GPS)   │
│  phc2sys: CLOCK_REALTIME → NIC PHC (/dev/ptp0)    │
│  ptp4l:   NIC PHC → PTP announce on Ethernet      │
└──┬────────┬────────┬────────┬────────────────────┘
   │ PTP    │ PTP    │ PTP    │ PTP (GigE Vision IEEE 1588)
┌──▼──────┐┌▼──────┐┌▼──────┐┌▼──────────────────────┐
│Robin W  ││Robin W││Robin W││ 4× RouteCAM            │
│Front    ││Rear-L ││Rear-R ││ P_CU25_CXLC_IP67       │
│(0°)     ││(120°) ││(240°) ││ PoE GigE Vision 从机   │
└─────────┘└───────┘└───────┘└────────────────────────┘
```

**每个阶段的预期同步精度：**

| 组件 | 精度 |
|-----------|----------|
| Atlas Duo GNSS PPS | < 20 ns to UTC |
| chrony (GPS 校准 CLOCK_REALTIME) | < 100 ns |
| NIC PHC via phc2sys | < 200 ns |
| LiDAR PTP slave | 300–800 ns |
| RouteCAM GigE Vision PTP slave | 1–10 µs |

> RouteCAM 数值假设使用带 IEEE 1588 边界时钟的网管 PoE 交换机（如 Planet WGS-6325-8UP2X —— 见 §3.1）。若使用非网管交换机（无边界时钟），实际值约为 5–50 µs。

**关键：** 主机时钟必须由 GPS 规范（通过 Atlas Duo PPS），而不是仅由互联网 NTP 规范。NTP 仅提供约 1–10 ms 精度，而 GPS PPS 提供 < 100 ns。

### 3.1 网络要求

| 配置 | 估计带宽 | 最小网卡 |
|--------------|-------------------|-------------|
| 1 Robin W | ~60 Mbps | 1 GbE |
| 3 Robin W | ~180 Mbps | 1 GbE |
| 4 RouteCAM (2MP @ 20fps) | ~400 Mbps | 1 GbE |
| 3 Robin W + 4 RouteCAM + PTP | ~600 Mbps + overhead | 1 GbE 可用；**推荐 10 GbE** 以留出余量并支持 jumbo frame |

> **Robin W 带宽说明：** Seyond 官方数据表标定 Robin W 单机输出约 60 Mbps。早期文档误写为 150 Mbps（把峰值脉冲当作持续带宽），使总量高估约 3×。3× Robin W ≈ 180 Mbps 持续；建议按 1.5× 预留抖动/突发余量。

**参考网络设计**

推荐的硬件组合是 **Teltonika RUTM50 / RUTM54** 5G/4G 蜂窝路由器（仅作为蜂窝 WAN 网关）配合 **Planet WGS-6325-8UP2X** 网管型 PoE++ 交换机（带 IEEE 1588 边界时钟和 10 GbE 上行的传感器局域网）。主机 PC 直接连接到交换机的 10 G SFP+ 端口 —— 不经过蜂窝路由器 —— 这样 PTP 报文的路径为 `主机 → 边界时钟 → 传感器`，不会穿越不支持 PTP 的路由器。

```
Internet (5G/4G)
       │
       ▼
┌──────────────────────────┐
│ Teltonika RUTM50/RUTM54  │  192.168.1.1
│  (仅作为蜂窝网关)         │  WAN 口未使用
└─────────────┬────────────┘
              │ 1 GbE  (RUTM50 LAN1 → Planet GbE 端口)
              ▼
┌────────────────────────────────────────────────────────┐
│ Planet WGS-6325-8UP2X (网管 L3, IEEE 1588 BC, PoE++)    │  192.168.1.2 (管理)
├────────────────────────────────────────────────────────┤
│ 端口    类型             设备                    IP      │
│ ----    ------------    --------------------    ------ │
│ 1       1 GbE           → RUTM50 LAN1           (上行)  │
│ 2       2.5 GbE PoE++   RouteCAM 前右           .20    │
│ 3       2.5 GbE PoE++   RouteCAM 前左           .21    │
│ 4       2.5 GbE PoE++   RouteCAM 后左           .22    │
│ 5       2.5 GbE PoE++   RouteCAM 后右           .23    │
│ 6       1 GbE  PoE++    Robin W 前              .10    │
│ 7       1 GbE  PoE++    Robin W 后左            .11    │
│ 8       1 GbE  PoE++    Robin W 后右            .12    │
│ SFP1    10 G            主机 PC (10G 网卡)       .40    │
│ SFP2    10 G            备用 (Atlas Duo 以太 / NAS)    │
└────────────────────────────────────────────────────────┘

Atlas Duo INS —— 三条到主机的物理通道（并非全部走以太网）：
  PPS  ──► 主机 /dev/pps0     (BNC, 硬件 PPS —— 绝对时间源)
  USB  ──► 主机 /dev/ttyUSB0  (NMEA + FusionEngine 报文)
  以太 ──► Planet 交换机      (192.168.1.30, 仅用于 NTRIP RTK 校正)
```

**IP 规划** —— 所有静态设备地址都在 `.100` 以下，因此 RUTM50 出厂默认 DHCP 池 (`.100–.249`) 保持不动，无需修改路由器配置：

| 192.168.1.x | 角色 |
|-------------|------|
| .1 | RUTM50 路由器 + DHCP 服务器 + 默认网关 |
| .2 | Planet WGS-6325-8UP2X 管理接口 |
| .10 – .12 | Robin W 激光雷达（在传感器上设静态） |
| .20 – .23 | RouteCAM 摄像头（通过 Aravis Web UI 设静态） |
| .30 | Atlas Duo INS 以太网（静态，仅用于 NTRIP RTK） |
| .40 | 主机 PC 10 GbE 网卡（通过 netplan / NetworkManager 设静态） |
| .100 – .249 | RUTM50 DHCP 池 —— 出厂默认，保持不动 |

**为什么主机连交换机而不是路由器**

1. *PTP 完整性。* 主机是 PTP grandmaster。它的同步报文必须经过支持 PTP 的硬件才能到达每个从机。RUTM50 不是 PTP 边界时钟；如果主机挂在 RUTM50 LAN 口上，每个 PTP 报文都会经过路由器并产生驻留时间抖动，这会使边界时钟交换机的作用完全失效。
2. *带宽。* 约 600 Mbps 的传感器流量进入 1 GbE LAN 口约占用 60%。主机到交换机 10 G SFP+ 链路消除瓶颈，并为 jumbo frame 和摄像头突发流量预留余量。

RUTM50 → Planet 上行链路只承载 NTRIP RTCM3（约 5 kbps）和主机端的互联网流量（SSH、系统更新、NTP 备援），利用率约 0.1%。传感器数据从不离开 Planet 交换机。

**Atlas Duo 以太网（RTK NTRIP）**

Atlas Duo 的以太网口在本设计中只承担一项任务：从您的 caster（Trimble VRS Now、本地基站等）拉取 RTK NTRIP 校正（RTCM3）。通过 Atlas Web UI 配置：静态 IP `192.168.1.30`、网关 `192.168.1.1`、DNS `192.168.1.1`，将 NTRIP 客户端指向您的 caster。PPS 仍然走自己的 BNC 物理线缆到 `/dev/pps0`，保持作为绝对时间源 —— 以太网通道仅添加 RTK，不改变时间同步链。

### 3.2 硬件 vs 软件时间戳

用 `ethtool -T <interface>` 检查您的网卡功能。设置脚本会自动检测。

| 时间戳 | PTP 精度 | 足以用于 |
|-------------|-------------|----------------|
| 硬件 | < 1 µs | 生产传感器融合 |
| 软件 | 20–50 µs | 开发、建图、通用机器人 |

大多数 Intel 网卡（I210、I225、X550、X710）支持硬件时间戳。USB 以太网适配器通常不支持。

---

## 4. 验证（手动）

运行设置脚本后，验证完整的同步链。脚本会自动运行自我测试，但您可以随时手动重新检查。

```bash
# GPS fix
gpsmon
# Should show satellites, fix type (3D), and time

# chrony — GPS PPS should be starred, not NTP
chronyc sources -v
chronyc tracking
# "Reference ID" should show PPS or NMEA, not an NTP server IP

# PTP grandmaster
sudo journalctl -u ptp4l-grandmaster -f
# Look for: "assuming the grand master role"
# master offset values should be < 1000 ns (hardware) or < 50 µs (software)

# PHC sync (hardware timestamping only)
sudo journalctl -u phc2sys-grandmaster -f
# offset values should be < 1000 ns

# PTP slave devices (LiDARs and cameras)
sudo pmc -u -b 0 'GET PORT_DATA_SET'

# Service status
sudo systemctl status gpsd chrony ptp4l-grandmaster phc2sys-grandmaster
```

---

## 5. 记录数据

### 5.1 记录架构

该系统使用**零 ROS 记录方法**以获得最大性能：

| 传感器 | 捕获方法 | 格式 | CPU 负载 |
|--------|---------------|--------|----------|
| Seyond Robin W LiDAR | `tcpdump`（内核级） | .pcap | 每个 LiDAR ~1% |
| Point One Nav Atlas Duo | `p1_runner`（本机二进制） | .p1log | ~1% |
| RouteCAM 摄像机 | Aravis / `tcpdump` | .pcap 或原始帧 | 每个摄像机 ~2–5% |

与 rosbag 记录（~30–50% CPU）相比，这跳过了点云解码、ROS 序列化和 DDS 中间件。数据在重放时解码。

### 5.2 配置 Atlas Duo 消息速率

```bash
cd ~/p1-host-tools

python3 bin/config_tool.py apply uart2_message_rate fe ROSPoseMessage 100ms
python3 bin/config_tool.py apply uart2_message_rate fe ROSGPSFixMessage 100ms
python3 bin/config_tool.py apply uart2_message_rate fe ROSIMUMessage on
python3 bin/config_tool.py save
```

### 5.3 使用快速记录器脚本进行记录

```bash
# Single LiDAR
sudo python3 sensor_recorder_fast.py

# 3 LiDARs
sudo python3 sensor_recorder_fast.py --num-lidars 3 \
    --lidar1-ip 192.168.1.10 \
    --lidar2-ip 192.168.1.11 \
    --lidar3-ip 192.168.1.12

# With YAML config
sudo python3 sensor_recorder_fast.py --config sensor_config.yaml
```

交互式命令：`R` 开始记录、`S` 停止、`H` 健康、`Q` 退出。

### 5.4 会话输出结构

```
~/recordings/session_20260311_143022/
├── lidar_pcap/
│   ├── robin_w_front.pcap       # Raw network capture (kernel-level)
│   ├── robin_w_rear_left.pcap   # All timestamps PTP-synchronized
│   └── robin_w_rear_right.pcap
├── camera_pcap/
│   ├── cam_front_right.pcap     # GigE Vision raw packets (PTP-stamped)
│   ├── cam_front_left.pcap
│   ├── cam_rear_left.pcap
│   └── cam_rear_right.pcap
├── p1nav/
│   └── *.p1log                  # FusionEngine binary (GNSS + IMU + pose)
├── session_metadata.json
└── session_stats.json
```

所有传感器都经过 PTP 同步，所以时间戳共享相同的 GPS 时间基准，可在后处理中对齐而无需额外的时钟校正。

### 5.5 YAML 配置参考

```yaml
point_one_nav:
  device_port: "/dev/ttyUSB0"
  baud_rate: 460800

lidars:
  - name: "robin_w_front"
    ip: "192.168.1.10"
    port: 8010
  - name: "robin_w_rear_left"
    ip: "192.168.1.11"
    port: 8010
  - name: "robin_w_rear_right"
    ip: "192.168.1.12"
    port: 8010

recording:
  output_dir: "~/recordings"
  interface: "eth0"
```

### 5.6 替代方案：ROS 2 原生记录

如果您更喜欢 rosbag（CPU 更高但回放更简单）：

```bash
# Terminal 1 — Point One Nav
ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args \
    -p connection_type:=tty -p tty_port:=/dev/ttyUSB0

# Terminal 2 — Seyond Robin W
ros2 launch seyond start.py

# Terminal 3 — Record
ros2 bag record -a -o my_dataset
```

确保 `use_sim_time` 在所有节点中为 `false`。当 `use_sim_time` 为 `false` 时，ROS 2 消息头使用 `CLOCK_REALTIME`，它通过 PTP 链由 GPS 校准。

```bash
# Verify timestamps are realistic (not zeros)
ros2 topic echo /robin_w_front/points --field header.stamp --once
```

---

## 6. 重放和可视化

### 6.1 重放 Point One Nav 数据 (.p1log)

```bash
SESSION=~/recordings/session_20260311_143022

# Interactive trajectory + IMU + GNSS plots
p1_display $SESSION/p1nav/

# Decode messages to terminal
p1_print $SESSION/p1nav/*.p1log

# Export to CSV / KML
p1_extract $SESSION/p1nav/
p1_extract --kml $SESSION/p1nav/
```

### 6.2 将 Seyond Robin W PCAP 重放到 ROS 2

```bash
ros2 launch seyond start.py \
    pcap_file:=$SESSION/lidar_pcap/robin_w_front.pcap \
    lidar_name:=robin_w_front \
    frame_id:=robin_w_front \
    frame_topic:=/robin_w_front/points
```

### 6.3 在 RViz2 / Foxglove Studio 中可视化

```bash
# RViz2
rviz2
# Add → By topic → PointCloud2. Set Fixed Frame. Set Point Size ~0.02.

# Foxglove Studio
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
foxglove-studio
# Connect to ws://localhost:8765
```

### 6.4 检查原始 PCAP

```bash
tshark -r $SESSION/lidar_pcap/robin_w_front.pcap -q -z io,stat,1
wireshark $SESSION/lidar_pcap/robin_w_front.pcap
```

### 6.5 将重放的数据转换为 Rosbag

```bash
ros2 bag record -o $SESSION/replayed_rosbag \
    /robin_w_front/points /robin_w_left/points /robin_w_right/points \
    /tf /tf_static
```

### 6.6 工具总结

| 工具 | 用途 | 输入 |
|------|---------|-------|
| `p1_display` | 交互式轨迹/IMU/GNSS 图表 | .p1log |
| `p1_extract` | 导出为 CSV / KML | .p1log |
| `p1_print` | 将消息解码到终端 | .p1log |
| `rviz2` | 3D 点云可视化 | ROS 2 topics |
| `foxglove-studio` | 多面板传感器仪表板 | ROS 2 topics |
| `tshark` / `wireshark` | 原始数据包检查 | .pcap |
| `arv-viewer-0.8` | 实时 GigE Vision 摄像机查看器 | Camera stream |

---

## 附录

### A. Seyond Robin W 默认参数

| 参数 | 默认值 | 说明 |
|-----------|---------|-------|
| IP Address | 192.168.1.10 | 通过网页 UI 或 `innovision_lidar_util` 更改 |
| Data Port | 8010 | TCP and UDP |
| Coordinate Mode | 3 (forward/left/up) | 匹配 ROS REP-103 |
| PTP | Supported | 由 `setup_robin_w_sync.sh` 启用 |
| FOV | 120° × 70° | |
| Points/sec | 1.28M | 192 scan lines |
| Range | 0.1–150 m | 在 10% 反射率下 70 m |

### B. RouteCAM_P_CU25_CXLC_IP67 关键规格

| 特性 | 值 |
|---------|-------|
| Sensor | AR0234 1/2.6" 2MP global shutter |
| 分辨率 | 1920 × 1200 |
| FOV | 158° DFOV, 134° HFOV, 73° VFOV |
| Interface | GigE Vision (M12 X-coded Ethernet) |
| Power | PoE (IEEE 802.3af) |
| Time Sync | IEEE 1588 PTP via GigE Vision |
| Protection | IP67 |
| Dome layout | 前立体对（104 mm baseline）+ 后对称对 |

### C. NVIDIA GPU 和 RT 内核兼容性

NVIDIA 内核模块（`nvidia.ko`）无法在 PREEMPT_RT 内核上加载。RT 内核上不提供 CUDA、cuDNN、TensorRT 或 GPU 计算。

| 内核 | CUDA 可用 | 最适合 |
|--------|---------------|----------|
| **Generic** | Yes | 训练、模拟、GPU 推理 |
| **RT** | **No** | 确定性传感器记录、实时控制 |

**推荐工作流：** 在 RT 内核上记录（确定性 PTP 时序、无需 GPU），然后重启到通用内核以使用完整 CUDA 访问进行后处理。

```
Field recording:   RT kernel → tcpdump + p1_runner (no GPU needed)
Post-processing:   Generic kernel → CUDA + PyTorch/TensorRT on recorded data
```

要在引导时选择内核，在 GRUB 菜单中选择 **Advanced options for Ubuntu**，然后选择 `*-realtime` 或 `*-generic` 条目。

### D. 故障排除

**PTP 未同步（大主偏移）：** 验证 ptp4l 配置中的接口名称与您的实际接口匹配。确认在每个 Robin W 上启用了 PTP。检查电缆连接。

**chronyc 显示 NTP 为主要（不是 PPS/NMEA）：** gpsd 可能未向 chrony 提供时间。运行 `sudo systemctl status gpsd` 和 `gpsmon`。如果 `/dev/pps0` 不存在，则未暴露 PPS——检查您的串行连接类型。

**LiDAR 数据包丢失：** 可能是网络带宽饱和。使用 `sudo ethtool -S eth0 | grep -i drop` 检查。对于 3 个 Robin W LiDAR + 4 个摄像机，需要 10 GbE 网卡。

**未检测到 GigE Vision 摄像机：** 检查 PoE 交换机电源，验证摄像机在同一子网上，并尝试使用 `arv-tool-0.8` 扫描。防火墙规则可能阻止 GigE Vision 发现数据包。

**rosbag 中的时间戳不对齐：** 验证 `use_sim_time` 在所有 ROS 2 节点中为 `false`。检查 `chronyc tracking` 确认 GPS 校准处于活动状态。

**gpsd 显示"NO FIX"：** 确保 Atlas Duo 天线具有清晰的天空视线。冷启动可能需要长达 30 分钟。验证正确的波特率（Atlas Duo 默认为 460800）。

**`<cstdint>` 构建错误：** Ubuntu 24.04 上的 GCC 14 更严格。`setup_ubuntu_sync.sh` 脚本会自动修补此问题。

**Seyond 构建失败：** 在 `seyond_ros_driver/` 内使用 `./build.bash`（不是从工作区根目录使用 `colcon build`）。脚本 `setup_robin_w_sync.sh` 处理此问题。
