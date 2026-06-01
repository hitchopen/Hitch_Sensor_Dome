# 传感器记录系统：Point One Nav Atlas Duo + Seyond Robin W LiDAR + RouteCAM 摄像头

在 Ubuntu 24.04 + ROS 2 Jazzy（可选 PREEMPT_RT 实时内核）上搭建的高性能 GNSS / IMU / LiDAR / 摄像头数据采集系统的安装与配置指南。

**目标硬件：**
- Ubuntu 24.04 LTS 工作站（在 Lenovo ThinkPad P1 Gen 6、Intel i9-13900H 上验证）
- Point One Nav Atlas Duo（GNSS / INS，仅以太网）
- 最多 3 台 Seyond Robin W 方向性 LiDAR
- 4 台 e-con RouteCAM_P_CU25_CXLC_IP67 GigE Vision 摄像头（PoE、IEEE 1588 PTP、2MP 全局快门）

**部署流程。** 本目录下 5 个编号脚本按顺序运行。每个脚本只负责一个逻辑步骤，自带自检并在结束时提示下一步运行哪一个脚本。

| # | 脚本 | 功能 |
|---|--------|--------|
| 1 | [`1_install_packages.sh`](1_install_packages.sh) | apt 依赖、RT 调度权限、内核调优、ROS 2 Jazzy |
| 2 | [`2_configure_host_network.sh`](2_configure_host_network.sh) | 主机 NIC 静态 IP、硬件时间戳检测、RUTM50 可达性 |
| 3 | [`3_setup_ins_to_pc_sync.sh`](3_setup_ins_to_pc_sync.sh) | gpsd（TCP 上的 NMEA）、chrony、ptp4l grandmaster、phc2sys、fusion-engine-driver（TCP） |
| 4 | [`4_setup_lidar_ptp.sh`](4_setup_lidar_ptp.sh) | Robin W PTP slave 启用 + Seyond ROS 2 驱动 |
| 5 | [`5_setup_camera_ptp.sh`](5_setup_camera_ptp.sh) | RouteCAM PTP slave 启用 + Aravis（仅 Tier 2） |

每台 LiDAR 的一次性配置由 [`provision_robin_w_multiunit.sh`](provision_robin_w_multiunit.sh) 负责 —— 见第 4 节。

**网络默认值。** 所有脚本从 [`../config/network_config.yaml`](../config/network_config.yaml) 读取默认值。为你的部署改一次这个 YAML（主要是 `host.interface`），脚本会自动拾取其他全部参数（主机 IP、传感器 IP、网关、DHCP 池）。运行时命令行参数和环境变量仍然可以覆盖 YAML —— 见每个脚本头部的用法说明。

---

## 第 1 节：安装 Ubuntu 实时内核与其他软件包

本节安装主机所需的、与网络和 PTP 配置无关的内容：apt 依赖、实时调度权限、内核 `sysctl` 调优，以及 ROS 2 Jazzy。

### 1.1 PREEMPT_RT 内核 —— 仅用于硬实时控制时才需要

**仅采集数据并不需要 RT 内核。** 用于感知/定位数据采集 —— [`recording/`](../recording/) 工作流将 GNSS / IMU / LiDAR / 摄像头打包成 Foxglove 原生 MCAP 包，供后续建图、感知训练、SLAM 评估使用 —— 普通 Ubuntu 24.04 通用内核**已经够用**。通用内核下的 PTP 同步精度通常落在 RT 内核数值的 1–2 倍以内（第 3 节给出具体数值），仍远高于穹顶里任何传感器所能分辨的精度。

只有当主机需要对传感器数据作出**有界延迟响应**时，RT 内核才真正有用：闭合实时控制环（转向、刹车、机械臂伺服）、运行确定性安全监控，或任何不能容忍毫秒级调度抖动的场景。如果你的应用是「先记录，再离线处理」，整段子节都可以跳过。

| 使用场景 | 内核 | 原因 |
|----------|--------|-----|
| 录制用于建图、感知、训练、离线分析 | **通用（默认）** | 安装更简单，支持完整 NVIDIA / CUDA / TensorRT，PTP 仍满足传感器融合精度需求 |
| 实时控制、确定性安全回路、硬件在环 | PREEMPT_RT | 调度延迟有界，代价是失去 CUDA 支持 —— 见附录 C |

确实需要 RT 时，通过 Ubuntu Pro 安装（个人使用最多 5 台机器免费），然后重启再继续：

```bash
# 在 ubuntu.com/pro 申请免费令牌
sudo pro attach YOUR_TOKEN_HERE
sudo pro status
sudo pro enable realtime-kernel
sudo reboot
```

重启后验证：

```bash
uname -a
# 应显示：... SMP PREEMPT_RT ...
```

**NVIDIA GPU 注意事项（仅 RT 内核）：** NVIDIA 内核模块（`nvidia.ko`）在 PREEMPT_RT 内核上无法加载 —— RT 内核下没有 CUDA、cuDNN、TensorRT。请使用 Intel 集显作为显示输出。完整双内核工作流见 [附录 C](#c-nvidia-gpu-and-rt-kernel-compatibility)。

### 1.2 运行安装脚本

```bash
chmod +x 1_install_packages.sh
./1_install_packages.sh
```

它会安装：

- **apt 依赖：** `build-essential`、`cmake`、`git`、`linuxptp`、`chrony`、`gpsd`、`pps-tools`、`tcpdump`、`ethtool`、`libyaml-cpp-dev`、Python 相关工具。
- **RT 调度组 + 限制：** 建立 `realtime` 组并加入当前用户，写入 `/etc/security/limits.d/99-realtime.conf`，授予该组 `rtprio 99` 和 `memlock unlimited`。无论是否运行 RT 内核，这些限制都是 PTP 守护进程以实时优先级运行的必要条件。
- **内核 `sysctl` 调优：** 大 UDP 缓冲区（`net.core.rmem_max=32 MiB`）和 `vm.swappiness=10`，应对高带宽 LiDAR / 摄像头流。
- **ROS 2 Jazzy：** desktop + 开发工具 + `rviz2` + `foxglove-bridge` + `pcl-ros` + `tf2-tools`。在 `~/.bashrc` 中 `source /opt/ros/jazzy/setup.bash`。
- **`tcpdump` 权限位：** `cap_net_raw+ep`，让录制脚本无需 `sudo` 即可抓 UDP。

脚本末尾的自检会验证：RT 内核标志、组成员资格、`sysctl` 值、ROS 2 安装。

完成后需要**注销并重新登录**才能让组成员关系生效 —— 这是唯一需要人工介入的一步。

---

## 第 2 节：在 RUTM50 + PoE 交换机上的参考网络配置

传感器局域网围绕一台 **Teltonika RUTM50** 5G/4G 蜂窝路由器搭建。参考设计分为**两层 (tier)**，避免「只装 LiDAR」的部署还得为没用上的硬件买单：

- **Tier 1** 只用 RUTM50、不需要单独的网管 PoE 交换机 —— 足以支撑 3 台 LiDAR + INS + PC。
- **Tier 2** 保留 Tier 1 不变，在其下游加一台 **Planet WGS-6325-8UP2X** PoE++ 交换机以接入 4 台 RouteCAM（摄像头本身依赖 PoE；该交换机的 IEEE 1588 边界时钟是顺带的好处）。

### 2.1 Tier 1 —— 仅 LiDAR（5 台设备，仅用 RUTM50，无独立交换机）

RUTM50 共有 5 个 RJ45 千兆口（1× WAN + 4× LAN）。把 WAN 口重新配置为 LAN 后（RutOS WebUI → Network → LAN → 「Use WAN port as LAN」开关，参见 Teltonika wiki [Setting up WAN as LAN](https://wiki.teltonika-networks.com/view/Setting_up_WAN_as_LAN)），5 个口全部变为 LAN，正好凑齐：1× PC + 1× INS + 3× LiDAR。

```
Internet (5G 蜂窝)
       │
       ▼
┌─────────────────────────────────────────────────────────────┐
│ Teltonika RUTM50                          192.168.1.1       │
│  已把 WAN 配为 LAN                         (蜂窝 WAN 仍内部 │
│  DHCP 池 .100–.249（出厂默认）              使用)            │
├─────────────────────────────────────────────────────────────┤
│ 端口           设备                                 IP        │
│ ----           ------------------------            ------    │
│ WAN-as-LAN     主机 PC                              .5        │
│ LAN 1          Robin W 前                           .10       │
│ LAN 2          Robin W 后左                         .11       │
│ LAN 3          Robin W 后右                         .12       │
│ LAN 4          Atlas Duo 以太 (NTRIP)               .30       │
└─────────────────────────────────────────────────────────────┘

Atlas Duo INS —— 单条以太网通道到主机：
  以太 ──► RUTM50 LAN 4       (192.168.1.30)
                              ├── FusionEngine over TCP 30201 → 主机
                              │    (/pose, /imu, /gps/fix, /odom)
                              ├── NMEA 0183 over TCP 30200 → 主机
                              │    (gpsd → chrony → CLOCK_REALTIME, ~10–100 ms 精度)
                              └── NTRIP RTCM3 from caster → Atlas
                                   (RTK 校正，约 5 kbps)
```

> **仅以太网的 Atlas Duo 连接。** 这是 Atlas Duo 在时间和数据上对外暴露的唯一通道 —— 设备本身不提供 BNC PPS 引脚，也不提供 USB 串口的 NMEA 输出。一条从 Atlas Duo 到 RUTM50 LAN 4 的网线承担全部职责：FusionEngine（姿态 / IMU / GPSFix / odom）走 TCP 30201、NMEA 0183 走 TCP 30200 供 chrony 使用、NTRIP RTCM3 由 caster 反向喂回 Atlas、设备的 REST API 走 HTTP。后果：主机 `CLOCK_REALTIME` 精度为 ~10–100 ms（仅 NMEA via gpsd），而不是硬件 PPS 可达的亚 100 ns；跨传感器 PTP 同步**不受影响**，主机 NIC PHC 仍在局域网上作 PTP grandmaster，LiDAR / 摄像头同步保持在 5–50 µs / 1–10 µs；绝对 GPS 时间对所有从 FusionEngine 消息头读取时间戳的消费者也**不受影响** —— Atlas 在每条消息里嵌入自己的 GPS 时间戳。

**Tier 1 下 LiDAR 的供电。** Robin W 可走 PoE（IEEE 802.3af）**或**12 V DC；RUTM50 LAN 口**不**带 PoE 输出。Tier 1 下有两种选择：

- **推荐：** 通过每台 LiDAR 的 M12 接头用 12 V DC 直接供电（每台一个电源，或一台多路输出实验电源）。
- **替代方案：** 在每条 LiDAR 网线上串一个无源 PoE 注入器 + 48 V DC 适配器。

Atlas Duo 自带独立的 DC 圆头电源输入，两种方案下都单独供电。

**Tier 1 的 PTP 同步取舍。** RUTM50 **不是** PTP 边界时钟 —— 其内部交换芯片只是转发以太网帧。PTP 报文从主机 PC 跨过 RUTM50 抵达 LiDAR 时会带来几微秒的驻留时间抖动。Tier 1 各阶段的预期同步：

| 组件 | Tier 1（仅 RUTM50） | Tier 2（Planet 边界时钟） | 用于传感器融合是否够？ |
|-----|--------------------|--------------------------|----------------------|
| chrony GPS 校准的 `CLOCK_REALTIME` | < 100 ns | < 100 ns | 是（两者都够） |
| NIC PHC via `phc2sys` | < 200 ns | < 200 ns | 是（两者都够） |
| Robin W LiDAR PTP slave | 5–50 µs | 300–800 ns | 是（两者都够） |

对 LiDAR-IMU 融合而言，5–50 µs 完全够 —— Robin W 单帧在 10–20 FPS 下持续 50–100 ms，跨传感器几微秒的偏差远小于一帧周期。第 3 行的显著改善只对摄像头/LiDAR 像素级时间对齐才有意义 —— 而那正是 Tier 2 的价值所在。

### 2.2 Tier 2 —— 增加 4× RouteCAM 摄像头（引入 Planet WGS-6325-8UP2X）

摄像头（e-con RouteCAM_P_CU25_CXLC_IP67）仅支持 PoE 供电，并且明显得益于一台真正的 PTP 边界时钟。在 RUTM50 下游加一台 Planet WGS-6325-8UP2X 同时满足两点：4× PoE++ 口供摄像头使用、加上 IEEE 1588 BC 用于跨传感器紧密同步。

Tier 2 有两种接线方式 —— 按你的组装选其一：

**(a) 摄像头单独挂在 Planet，Tier 1 不动。** 升级最轻 —— LiDAR、INS、PC 全部继续挂在 RUTM50；只有摄像头进 Planet 交换机。把 Planet 的端口 1 上行到 RUTM50 任一空闲 LAN 口，让摄像头能通到主机。LiDAR PTP 保持 Tier 1 的 5–50 µs；摄像头 PTP 借助 Planet 的边界时钟达到 1–10 µs。布线改动最少时推荐这种。

**(b) 全部迁到 Planet。** 主机 PC 改接 Planet 的 SFP1（10 GbE），全部 LiDAR 改接 PoE++ 端口 6–8，RUTM50 退化为「仅作为通往互联网拉 NTRIP 校正的上行」。所有传感器都坐在支持 PTP 的交换芯片上，LiDAR PTP 进一步收紧到 300–800 ns。当你还想在约 600 Mbps 的传感器总流量下保持 jumbo frame 稳定时这种是必须的（1 GbE 链路 60% 利用率不舒服，10 GbE 主机链路彻底消除瓶颈）。原始 Hitch 穹顶参考设计采用的就是这种布局。

布局 (b) 作为参考：

```
Internet (5G 蜂窝)
       │
       ▼
┌──────────────────────────┐
│ Teltonika RUTM50         │  192.168.1.1
│  (仅作为蜂窝网关)         │  不再需要 WAN-as-LAN
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
│ SFP1    10 G            主机 PC (10G 网卡)        .5    │
│ SFP2    10 G            备用 (Atlas Duo 以太 / NAS)    │
└────────────────────────────────────────────────────────┘
```

### 2.3 IP 规划

Tier 1 和 Tier 2 共用同一份地址表。所有静态设备都在 `.100` 以下，所以 RUTM50 出厂默认 DHCP 池 `.100–.249` 保持不动，除了静态租约保留之外路由器无需任何配置。

| 192.168.1.x | 角色 | Tier 1 | Tier 2 |
|-------------|------|--------|--------|
| .1 | RUTM50 路由器 + DHCP 服务器 + 默认网关 | ✓ | ✓ |
| .2 | Planet WGS-6325-8UP2X 管理接口 | — | ✓ |
| .5 | 主机 PC 网卡（通过 netplan / NetworkManager 设静态） | ✓ | ✓ |
| .10 – .12 | Robin W 激光雷达（在传感器上设静态） | ✓ | ✓ |
| .20 – .23 | RouteCAM 摄像头（通过 Aravis Web UI 设静态） | — | ✓ |
| .30 | Atlas Duo INS 以太网（静态，仅用于 NTRIP RTK） | ✓ | ✓ |
| .100 – .249 | RUTM50 DHCP 池 —— 出厂默认，保持不动 | ✓ | ✓ |

> **为什么 `.5` 和 `.30` 固定到这两个地址。** 在 RUTM50 的 Network → LAN → Static Leases 页通过 MAC 地址把 PC 静态租约到 .5、把 Atlas Duo 静态租约到 .30。这样两台设备重启之后还是同一地址，而设备端完全不需要写静态网络配置 —— RUTM50 每次都发同一个 IP。完整的 Static Leases 操作步骤见项目根目录 README 的网络小节。

**Atlas Duo 以太网（时间 + 数据 + NTRIP）。** Atlas Duo 唯一对外暴露的时间与数据接口就是以太网口 —— 它的硬件不带 BNC PPS 引脚，也不带 USB 串口的数据输出。同一条网线既把 FusionEngine（TCP 30201）和 NMEA（TCP 30200）送到主机，也从你的 caster（Trimble VRS Now、本地基站等）反向拉取 RTK NTRIP 校正（RTCM3）。通过 Atlas Web UI 配置：静态 IP `192.168.1.30`（或 DHCP + 保留）、网关 `192.168.1.1`、DNS `192.168.1.1`，将 NTRIP 客户端指向你的 caster。整条时间同步链（gpsd → chrony → ptp4l）都跑在这同一条以太网上 —— 主机侧的配置见 §3。

### 2.4 各层带宽估算

| 配置 | 估计带宽 | 最小网卡 | 适用层级 |
|------|---------|---------|---------|
| 1 Robin W | ~60 Mbps | 1 GbE | Tier 1 |
| 3 Robin W | ~180 Mbps | 1 GbE | Tier 1 |
| 4 RouteCAM (2MP @ 20fps) | ~400 Mbps | 1 GbE | 需要 Tier 2 |
| 3 Robin W + 4 RouteCAM + PTP | ~600 Mbps + overhead | 1 GbE 可用；**推荐 10 GbE** 以留出余量并支持 jumbo frame | Tier 2 (b) |

> **Robin W 带宽说明：** Seyond 官方数据表标定 Robin W 单机持续输出约 60 Mbps（不是早期文档把脉冲峰值当作持续值的 150 Mbps）。3× Robin W ≈ 180 Mbps 持续；建议按 1.5× 预留抖动/突发余量。

### 2.5 运行主机网络脚本

```bash
chmod +x 2_configure_host_network.sh
./2_configure_host_network.sh
```

它会做的事：

- 把传感器侧 NIC 配为 `network_config.yaml` 中的静态 IP（默认 `192.168.1.5/24`）。
- 通过 `ethtool -T` 探测 IEEE 1588 能力，决定使用硬件还是软件时间戳。结果写入 `/run/hitch_dome_net.env`，第 3 节脚本读取并自动选择 `ptp4l` 模式。
- ping RUTM50 路由器以验证可达性。
- 当传入 `--add-factory-alias` 时，可选地在传感器 NIC 上加一个临时 `172.168.1.100/24` 别名，用于工厂状态 LiDAR 的配置（见第 4 节）。

**先确认你的网卡名称**：

```bash
ip link show
# 找到有线以太网（例如 enp0s31f6、eth0、eno1）
```

### 2.6 硬件 vs 软件 PTP 时间戳

用 `ethtool -T <interface>` 检查你网卡的能力 —— 第 2 节脚本会替你做并把结果写入 `/run/hitch_dome_net.env`。

| 时间戳 | PTP 精度 | 足以用于 |
|------|---------|---------|
| 硬件 | < 1 µs | 生产级传感器融合 |
| 软件 | 20–50 µs | 开发、建图、通用机器人 |

大多数 Intel 网卡（I210、I225、X550、X710）支持硬件时间戳。USB 以太网适配器通常不支持。

### 2.7 每台 LiDAR 的一次性网络配置

每台全新出厂的 Robin W 都使用同一个出厂 IP `172.168.1.10`（见 [*Robin W1G User Manual* V2.2 §3.1](https://www.seyond.com/wp-content/uploads/2025/03/Seyond-Robin-W1G-LiDAR_User-Manual_V2.2_EN_Public_20250103.pdf)）和同一个 UDP 端口。让三台跑在同一以太网段上需要为每台分配独立的 IP **以及** 独立的 UDP 目的端口 —— 穹顶分配为 `.10` / `.11` / `.12` 和端口 `8337` / `8338` / `8339`。这纯粹是网络配置（IP + 端口），所以归在第 2 节；PTP 启用要到第 4 节才做。

配置过程**一次只接一台 Robin W**，按物理位置配。因为三台出厂态都共用 IP `172.168.1.10`，重新编号过程中线上同一时刻只能有一台。

**先决定哪台物理单元对应哪个位置。** 穹顶 SCAD 模型定义三个位置：

| 位置 | 穹顶角度 | 目标 IP | UDP 端口 |
|------|--------|--------|---------|
| `front` | 0°（朝 +X / 前） | `192.168.1.10` | `8337` |
| `rear_left` | 120°（朝后左） | `192.168.1.11` | `8338` |
| `rear_right` | 240°（朝后右） | `192.168.1.12` | `8339` |

最简单的做法：**开始前用美纹胶带在三台 LiDAR 上贴 FRONT / REAR-LEFT / REAR-RIGHT**，然后按这个顺序逐一连接主机。如果你想根据物理状况选择（外壳更整洁的放在最显眼的前向位置等），那就先接一台、浏览 `http://172.168.1.10/api/v1/static_info` 读出序列号再决定。

**操作流程：**

```bash
# 把 Robin W #1 通过以太网接到主机的传感器网卡。设备会以出厂 IP
# 172.168.1.10 上线。然后：
chmod +x provision_robin_w_multiunit.sh
./provision_robin_w_multiunit.sh --position front

# 给 Robin W #1 断电 / 拔线，接 Robin W #2：
./provision_robin_w_multiunit.sh --position rear_left

# 最后是 Robin W #3：
./provision_robin_w_multiunit.sh --position rear_right
```

每次调用脚本会做：

1. 在传感器网卡上临时加一个 `172.168.1.100/24` 别名，让主机能联系出厂 IP 的 LiDAR（退出时自动移除）。
2. 发现已接入的设备（出厂态的全新单元在 `172.168.1.10`，已配置过的单元会在该位置的目标 IP）。
3. 通过 `innovusion_lidar_util get_static_info` 读出 LiDAR 的序列号。
4. 调用 `set_network` 把单元从 `172.168.1.10` 移到 `192.168.1.10` / `.11` / `.12`，重启，等待约 25 秒。
5. 上传位置专用的 `PCS_ENV`（`RobinW_FW2835_Multiunit/RW_FW2835_robin_w_<位置>_unicast.env`），再次重启。
6. 把这个单元的序列号写入或更新到 [`RobinW_FW2835_Multiunit/serial_inventory.yaml`](RobinW_FW2835_Multiunit/serial_inventory.yaml)，保留 SN ↔ 位置的对应关系以便维护与备件管理。

脚本是幂等的 —— 对已经配置好的 LiDAR 再次运行会识别状态匹配并打印 `[SKIP]`。脚本还会拒绝覆盖：若 inventory 中记录的某个位置的序列号与当前接入的单元不同，脚本会停止并提示你确认 —— 这能在错误传播之前抓出「插错单元」的失误。如果某台 Robin W 需要更换（RMA、故障），删除 `serial_inventory.yaml` 中该位置的整段记录，然后用相应的 `--position` 重新运行脚本即可。

当三个位置都在 `serial_inventory.yaml` 中有记录之后，网络配置就完整了，可以进入第 3 节（INS-to-PC 时间同步）和第 4 节（LiDAR PTP 启用 + ROS 2 驱动）。

---

## 第 3 节：INS 到 PC 的 PTP 同步

Point One Nav Atlas Duo 通过 TCP 输出 NMEA 用于规范主机 `CLOCK_REALTIME`，同时通过 TCP 输出 FusionEngine 把姿态 / IMU / GPSFix / 里程信息送给 ROS 2 驱动。主机则在传感器局域网上扮演 PTP grandmaster 角色 —— 第 4、5 节配置同步到它的传感器 *slave*。

```
┌─────────────────────────┐
│  GPS 卫星                 │
└───────────┬─────────────┘
            │ RF
┌───────────▼──────────────────┐
│  Point One Nav Atlas Duo      │  ← GPS 定位 + 通过 Polaris/NTRIP 拉 RTK
│  - FusionEngine TCP 30201     │  ← /pose, /imu, /gps/fix, /odom
│  - NMEA 0183  TCP 30200       │  ← gpsd 源 (GPS 时间，无 PPS)
│  - REST API HTTP 80           │  ← 配置 + 监控
└─────────────┬─────────────────┘
              │ 以太网 (单根线到 RUTM50 LAN 4)
              │ Atlas at 192.168.1.30
┌─────────────▼─────────────────────────────────────┐
│  Ubuntu 24.04 主机 (PTP Grandmaster, 192.168.1.5) │
│                                                    │
│  gpsd ← tcp://192.168.1.30:30200 (NMEA) → SHM     │
│  chrony ← SHM → CLOCK_REALTIME (~10–100 ms 到 GPS) │
│  phc2sys: CLOCK_REALTIME → NIC PHC (/dev/ptp0)    │
│  ptp4l:   NIC PHC → 在以太网上 announce PTP        │
│  fusion-engine-driver: TCP 30201 → /pose /imu /…   │
└────────────────────────────────────────────────────┘
```

**主机侧各阶段的预期同步精度**（传感器侧数值见第 4、5 节）：

| 组件 | 精度 | 说明 |
|------|------|-----|
| Atlas Duo 内部 GPS 时间 | < 20 ns to UTC | 在设备内部；通过 FusionEngine 消息时间戳暴露出来 |
| chrony (NMEA 规范的 `CLOCK_REALTIME`) | ~10–100 ms | 仅 NMEA，无 PPS。见下方说明。 |
| NIC PHC via `phc2sys` | 跟踪 `CLOCK_REALTIME` ± 200 ns | 即使 `CLOCK_REALTIME` 相对 GPS 漂移，相对跟踪仍然紧密 |
| FusionEngine 消息 `header.stamp` | < 20 ns to GPS | Atlas 嵌入每条消息，与主机时钟无关 |

> **为什么主机时钟变松。** Atlas Duo 唯一对外暴露的时间输出是通过 TCP 出来的 NMEA 句子流（设备硬件本身没有 PPS BNC 输出，也没有 USB 串口数据输出）。NMEA 是句速率（1 Hz），没有亚秒级边沿，所以 chrony 通常能把 `CLOCK_REALTIME` 规范到 ±10–100 ms 量级 —— 不是硬件 PPS 边沿能给的 < 100 ns 精度。这对于 ROS 话题 `header.stamp` 来说没问题，因为录制脚本会从 FusionEngine 消息头里取 Atlas 时间戳（那个仍然是设备端的 GPS 精度，与主机时钟漂移无关）。raw `tcpdump` 包的内核时间戳继承的是较松的 `CLOCK_REALTIME` 精度 —— 如果你在乎包级时间精确对齐到 GPS 亚毫秒，做后处理时让它与同一份抓包里的 FusionEngine 消息互相对齐即可。跨传感器 PTP（主机 PHC ↔ LiDAR / 摄像头从机）不受影响 —— 相对跨传感器同步仍是亚 µs（硬件时间戳）或亚 50 µs（软件时间戳）。

### 3.1 运行 INS-to-PC 同步脚本

```bash
chmod +x 3_setup_ins_to_pc_sync.sh
./3_setup_ins_to_pc_sync.sh
```

它会配置：

- **gpsd：** 通过 TCP 从 Atlas Duo 的 `tcp://192.168.1.30:30200` 读 NMEA。落到共享内存供 chrony 使用。不再用 `/dev/ttyUSB0` 或 `/dev/pps0`。
- **chrony：** 用 gpsd SHM（仅 NMEA）规范 `CLOCK_REALTIME`。NTP 池保留作为备援。稳态预期精度约 10–100 ms。
- **`ptp4l` grandmaster：** 在传感器 NIC 上 announce。由于主机时钟不再被规范到 < 100 ns 的 GPS 精度，脚本默认把 `clockClass` 设为 **13**（应用专用、锁到内部参考），而不是 6（锁到一级 GPS 参考）。跨传感器 PTP 同步仍然紧密；改成 13 只是诚实告诉 PTP 从机当前的绝对时间归属。
- **`phc2sys`：** 把 `CLOCK_REALTIME` 复制到 NIC PHC（仅硬件时间戳时相关）。
- **systemd：** enable 并启动 `gpsd`、`chrony`、`ptp4l-grandmaster`、`phc2sys-grandmaster`。整个链路无需重启即可生效。
- **fusion-engine-driver：** 克隆并 `colcon build` Point One Nav ROS 2 驱动到 `~/ros2_ws/`。自动打 GCC 14 的 `<cstdint>` 补丁。驱动调用使用 `connection_type:=tcp` 加 Atlas Duo IP（不再使用 `connection_type:=tty`）。
- **Point One host tools：** 把 `p1-host-tools/` 克隆到 `$HOME`，`pip install fusion-engine-client[all]`。

### 3.2 手动验证

脚本的自检覆盖了主要内容，如下命令可以手动深入检查：

```bash
# GPS 定位
gpsmon
# 应显示卫星、定位类型 (3D)、时间

# Atlas Duo 可达性
ping -c 3 192.168.1.30
curl -s http://192.168.1.30/api/v1/device/status | python3 -m json.tool

# 从 Atlas Duo 拉 TCP NMEA 流
nc -w 5 192.168.1.30 30200 | head -20
# 应看到 $GPGGA、$GPRMC 等句子

# 从 Atlas Duo 拉 TCP FusionEngine 流
nc -w 5 192.168.1.30 30201 | xxd | head -5
# 应看到二进制 FusionEngine 帧

# chrony —— NMEA 应被星号标记
chronyc sources -v
chronyc tracking
# "Reference ID" 应显示 NMEA，而不是 NTP 服务器 IP

# PTP grandmaster
sudo journalctl -u ptp4l-grandmaster -f
# 应看到："assuming the grand master role"
# master offset 应 < 1000 ns (硬件) 或 < 50 µs (软件)

# PHC 同步 (仅硬件时间戳)
sudo journalctl -u phc2sys-grandmaster -f
# offset 应 < 1000 ns

# 服务状态
sudo systemctl status gpsd chrony ptp4l-grandmaster phc2sys-grandmaster
```

---

## 第 4 节：PC 到 LiDAR 的 PTP 同步

把 3 台 Seyond Robin W 配置为第 3 节中 grandmaster 的 PTP slave，并安装 Seyond ROS 2 驱动。假定 [§2.7](#27-每台-lidar-的一次性网络配置) 的一次性网络配置已经完成，把每台 Robin W 从工厂 IP `172.168.1.10` 改成穹顶 IP `192.168.1.10` / `.11` / `.12`。如果 `4_setup_lidar_ptp.sh` 在这些地址上 ping 不到 LiDAR，请先回到 §2.7 跑配置脚本。

### 4.1 运行 LiDAR PTP 同步脚本

```bash
chmod +x 4_setup_lidar_ptp.sh
./4_setup_lidar_ptp.sh
```

它会做的事：

- 校验第 3 节的 PTP grandmaster 正在运行。
- ping 每台 LiDAR 的配置后 IP。
- 通过 `innovusion_lidar_util` 在每台 Robin W 上启用标准 IEEE 1588 PTP（不是 automotive gPTP）。
- 克隆并构建 Seyond ROS 2 驱动（在 `seyond_ros_driver/` 内运行 `./build.bash`，因为从工作区根目录的 `colcon build` 在 Seyond 上不起作用）。
- 对每台 LiDAR 自检 PTP slave 同步状态。

配置完成后预期精度：

| 组件 | 精度（Tier 1 / RUTM50） | 精度（Tier 2 / Planet BC） |
|------|----------------------|--------------------------|
| Robin W PTP slave 到 grandmaster | 5–50 µs | 300–800 ns |

---

## 第 5 节：PC 到摄像头的 PTP 同步

> **仅 Tier 2。** 摄像头本身要求 PoE 且明显得益于 Planet WGS-6325-8UP2X 的边界时钟 —— 见 §2.2。如果你跑的是 Tier 1 仅 LiDAR 部署，整节都可以跳过。

### 5.1 运行摄像头 PTP 同步脚本

```bash
chmod +x 5_setup_camera_ptp.sh
./5_setup_camera_ptp.sh
```

它会做的事：

- 校验第 3 节的 PTP grandmaster 正在运行。
- 安装 Aravis GigE Vision 库与工具（`arv-tool-0.8`、`arv-viewer-0.8`）。
- 安装 ROS 2 摄像头相关包（`image_transport`、`camera_info_manager` 等）。
- 配置 GigE Vision 网络设置：传感器 NIC 上的 jumbo frame（MTU 9000）、大 UDP 接收缓冲区。
- 通过 Aravis 发现每台 RouteCAM。
- 通过 GenICam `GevPTPMode = Slave` 寄存器在每台摄像头上启用 PTP。
- 对每台摄像头自检 PTP slave 状态，并报告 `Synchronized` / `Master` / `Listening`。

通过 Planet WGS-6325-8UP2X 边界时钟后的预期精度：

| 组件 | 精度 |
|------|------|
| RouteCAM GigE Vision PTP slave | 1–10 µs |

---

## 第 6 节：数据采集

### 6.1 采集架构

本系统采用 **零 ROS 采集方案** 以追求最大性能：

| 传感器 | 采集方式 | 格式 | CPU 负载 |
|--------|---------|------|---------|
| Seyond Robin W LiDAR | `tcpdump`（内核级） | .pcap | 每台 ~1% |
| Point One Nav Atlas Duo | `p1_runner`（原生二进制） | .p1log | ~1% |
| RouteCAM 摄像头 | Aravis / `tcpdump` | .pcap 或原始帧 | 每台 ~2–5% |

相比 rosbag 录制（约 30–50% CPU），跳过了点云解码、ROS 序列化、DDS 中间件等开销。数据在回放时再解码。

### 6.2 配置 Atlas Duo 报文速率

```bash
cd ~/p1-host-tools

python3 bin/config_tool.py apply uart2_message_rate fe ROSPoseMessage 100ms
python3 bin/config_tool.py apply uart2_message_rate fe ROSGPSFixMessage 100ms
python3 bin/config_tool.py apply uart2_message_rate fe ROSIMUMessage on
python3 bin/config_tool.py save
```

### 6.3 用快速录制脚本采集

```bash
# 单台 LiDAR
sudo python3 sensor_recorder_fast.py

# 3 台 LiDAR
sudo python3 sensor_recorder_fast.py --num-lidars 3 \
    --lidar1-ip 192.168.1.10 \
    --lidar2-ip 192.168.1.11 \
    --lidar3-ip 192.168.1.12

# 使用 YAML 配置
sudo python3 sensor_recorder_fast.py --config sensor_config.yaml
```

交互命令：`R` 开始录制、`S` 停止、`H` 健康检查、`Q` 退出。

### 6.4 会话输出结构

```
~/recordings/session_20260311_143022/
├── lidar_pcap/
│   ├── robin_w_front.pcap       # 原始网络抓包（内核级）
│   ├── robin_w_rear_left.pcap   # 所有时间戳均经 PTP 同步
│   └── robin_w_rear_right.pcap
├── camera_pcap/
│   ├── cam_front_right.pcap     # GigE Vision 原始包（带 PTP 时间戳）
│   ├── cam_front_left.pcap
│   ├── cam_rear_left.pcap
│   └── cam_rear_right.pcap
├── p1nav/
│   └── *.p1log                  # FusionEngine 二进制 (GNSS + IMU + 姿态)
├── session_metadata.json
└── session_stats.json
```

所有传感器都已 PTP 同步，时间戳共享 GPS 时间基准，后处理无需额外时钟修正即可对齐。

### 6.5 YAML 配置参考

```yaml
point_one_nav:
  connection_type: "tcp"
  tcp_host: "192.168.1.30"
  tcp_port: 30201

lidars:
  - name: "robin_w_front"
    ip: "192.168.1.10"
    port: 8337
  - name: "robin_w_rear_left"
    ip: "192.168.1.11"
    port: 8338
  - name: "robin_w_rear_right"
    ip: "192.168.1.12"
    port: 8339

recording:
  output_dir: "~/recordings"
  interface: "eth0"
```

### 6.6 备选：ROS 2 原生录制

如果你更习惯 rosbag（CPU 占用更高但回放更简单）：

```bash
# 终端 1 —— Point One Nav（TCP 上的 FusionEngine）
ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args \
    -p connection_type:=tcp \
    -p tcp_host:=192.168.1.30 \
    -p tcp_port:=30201

# 终端 2 —— Seyond Robin W
ros2 launch seyond start.py

# 终端 3 —— 录制
ros2 bag record -a -o my_dataset
```

确保所有节点的 `use_sim_time` 为 `false`。`use_sim_time=false` 时 ROS 2 消息头使用 `CLOCK_REALTIME`，该时钟通过 PTP 链路由 GPS 规范。

```bash
# 验证时间戳合理（不是 0）
ros2 topic echo /robin_w_front/points --field header.stamp --once
```

---

## 第 7 节：回放与可视化

### 7.1 回放 Point One Nav 数据 (.p1log)

```bash
SESSION=~/recordings/session_20260311_143022

# 交互式轨迹 + IMU + GNSS 图
p1_display $SESSION/p1nav/

# 终端解码消息
p1_print $SESSION/p1nav/*.p1log

# 导出 CSV / KML
p1_extract $SESSION/p1nav/
p1_extract --kml $SESSION/p1nav/
```

### 7.2 把 Seyond Robin W PCAP 回放为 ROS 2

```bash
ros2 launch seyond start.py \
    pcap_file:=$SESSION/lidar_pcap/robin_w_front.pcap \
    lidar_name:=robin_w_front \
    frame_id:=robin_w_front \
    frame_topic:=/robin_w_front/points
```

### 7.3 在 RViz2 / Foxglove Studio 中可视化

```bash
# RViz2
rviz2
# Add → By topic → PointCloud2。设置 Fixed Frame、Point Size ~0.02。

# Foxglove Studio
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
foxglove-studio
# 连接到 ws://localhost:8765
```

### 7.4 检视原始 PCAP

```bash
tshark -r $SESSION/lidar_pcap/robin_w_front.pcap -q -z io,stat,1
wireshark $SESSION/lidar_pcap/robin_w_front.pcap
```

### 7.5 把回放数据转为 rosbag

```bash
ros2 bag record -o $SESSION/replayed_rosbag \
    /robin_w_front/points /robin_w_left/points /robin_w_right/points \
    /tf /tf_static
```

### 7.6 工具一览

| 工具 | 用途 | 输入 |
|------|------|------|
| `p1_display` | 交互式轨迹 / IMU / GNSS 图 | .p1log |
| `p1_extract` | 导出 CSV / KML | .p1log |
| `p1_print` | 终端解码消息 | .p1log |
| `rviz2` | 3D 点云可视化 | ROS 2 主题 |
| `foxglove-studio` | 多面板传感器面板 | ROS 2 主题 |
| `tshark` / `wireshark` | 原始包检视 | .pcap |
| `arv-viewer-0.8` | 实时 GigE Vision 摄像头查看 | 摄像头流 |

---

## 附录

### A. Seyond Robin W 默认参数

| 参数 | 出厂默认 | Hitch 穹顶（配置后） | 备注 |
|-----|---------|--------------------|------|
| IP 地址 | `172.168.1.10` | `192.168.1.10` / `.11` / `.12` | 出厂 IP 见 Seyond *Robin W1G User Manual* V2.2 §3.1。由 `provision_robin_w_multiunit.sh` 重新分配。 |
| 数据端口（UDP + TCP） | `8010` | `8337` / `8338` / `8339` | 每台一个端口。基于 Seyond 的 8010/8020/8030 范例换段以避开 Hadoop 的 8020/8030 和繁忙的 8000-8099 开发服务器段。 |
| 坐标模式 | `3`（前/左/上） | 同 | 匹配 ROS REP-103 |
| PTP | 支持 | 启用（标准 L3 IEEE 1588） | 由 `4_setup_lidar_ptp.sh` 启用 |
| FOV | 120° × 70° | 同 | |
| 点率 | 1.28M / 秒 | 同 | 192 扫描线 |
| 量程 | 0.1–150 m | 同 | 70 m @ 10% 反射率 |

### B. RouteCAM_P_CU25_CXLC_IP67 关键参数

| 特性 | 数值 |
|------|------|
| 传感器 | AR0234 1/2.6" 2MP 全局快门 |
| 分辨率 | 1920 × 1200 |
| FOV | 158° DFOV、134° HFOV、73° VFOV |
| 接口 | GigE Vision (M12 X-coded 以太网) |
| 供电 | PoE (IEEE 802.3af) |
| 时间同步 | IEEE 1588 PTP via GigE Vision |
| 防护等级 | IP67 |
| 穹顶布局 | 前向立体对（基线 104 mm）+ 后向对称对 |

### C. NVIDIA GPU 与 RT 内核兼容性

NVIDIA 内核模块（`nvidia.ko`）无法在 PREEMPT_RT 内核上加载。RT 内核下不提供 CUDA、cuDNN、TensorRT 或 GPU 计算。

| 内核 | 支持 CUDA | 适用于 |
|------|---------|--------|
| **通用** | 是 | 训练、仿真、GPU 推理 |
| **RT** | **否** | 确定性传感器记录、实时控制 |

**推荐工作流：** 在 RT 内核下采集（PTP 时序确定，不需要 GPU），然后重启进入通用内核做后处理（完整 CUDA 可用）。

```
现场采集：   RT 内核 → tcpdump + p1_runner（不需要 GPU）
后处理：     通用内核 → 在录制数据上跑 CUDA + PyTorch/TensorRT
```

启动时选择内核：在 GRUB 菜单的 **Advanced options for Ubuntu** 中选择 `*-realtime` 或 `*-generic` 项。

### D. 故障排查

**PTP 不同步（master offset 很大）：** 确认 `ptp4l` 配置中的接口名称与实际匹配。确认每台 Robin W 都启用了 PTP。检查电缆连接。

**chronyc 显示 NTP 为主源（不是 NMEA）：** gpsd 可能没拿到 Atlas Duo 的 TCP 流。先 `ping 192.168.1.30` 确认可达，再确认 Atlas Duo 已在 Web UI 上点了 Start Navigating（不启动导航引擎，就没有 NMEA 输出）。运行 `sudo systemctl status gpsd` 和 `gpsmon`。绕开 gpsd 直接测原始 NMEA：`nc -w 5 192.168.1.30 30200 | head`。

**LiDAR 丢包：** 通常是网络带宽饱和。用 `sudo ethtool -S eth0 | grep -i drop` 检查。3 台 Robin W + 4 台摄像头建议用 10 GbE NIC。

**未检测到 GigE Vision 摄像头：** 检查 PoE 交换机供电、确认摄像头在同一子网，用 `arv-tool-0.8` 扫描。防火墙规则可能阻止 GigE Vision 发现包。

**rosbag 时间戳错乱：** 确认所有 ROS 2 节点 `use_sim_time` 为 `false`。用 `chronyc tracking` 确认 GPS 校准仍然有效。

**gpsd 显示 "NO FIX"：** 确保 Atlas Duo 天线视野开阔。冷启动可能需要长达 30 分钟。确认串口波特率正确（Atlas Duo 默认 460800）。

**`<cstdint>` 构建错误：** Ubuntu 24.04 的 GCC 14 更严格。`3_setup_ins_to_pc_sync.sh` 和 `4_setup_lidar_ptp.sh` 会自动修复。

**Seyond 构建失败：** 在 `seyond_ros_driver/` 内运行 `./build.bash`（不要从工作区根目录运行 `colcon build`）。`4_setup_lidar_ptp.sh` 已自动处理。
