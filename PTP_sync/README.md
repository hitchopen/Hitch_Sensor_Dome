# Sensor Recording System: Point One Nav Atlas Duo + Seyond Robin W LiDAR + RouteCAM Cameras

Installation and configuration guide for a high-performance GNSS/IMU/LiDAR/camera data recording system running on Ubuntu 24.04 with ROS 2 Jazzy and the PREEMPT_RT real-time kernel.

**Target Hardware:**
- Ubuntu 24.04 LTS workstation (tested on Lenovo ThinkPad P1 Gen 6, Intel i9-13900H)
- Point One Nav Atlas Duo (GNSS/INS with PPS output)
- Up to 3× Seyond Robin W directional LiDARs
- 4× e-con RouteCAM_P_CU25_CXLC_IP67 GigE Vision cameras (PoE, IEEE 1588 PTP, 2MP global shutter)

---

## 1. Setup Scripts

Three scripts automate the entire installation. Run them in order on a fresh Ubuntu 24.04 system.

**Step 1** installs all system prerequisites, configures the GPS-disciplined PTP grandmaster (gpsd → chrony → ptp4l → phc2sys), installs ROS 2 Jazzy, and builds the Point One Nav driver and host tools.

```bash
./setup_ubuntu_sync.sh --eth enp0s31f6
```

**Step 2** enables PTP synchronization on each Seyond Robin W LiDAR and builds the Seyond ROS 2 driver. Run when LiDARs are powered and connected.

```bash
./setup_robin_w_sync.sh --eth enp0s31f6 --ips 192.168.1.10,192.168.1.11,192.168.1.12
```

**Step 3** installs the Aravis GigE Vision library, enables IEEE 1588 PTP on each RouteCAM camera, and installs ROS 2 camera packages. Run when cameras are powered via PoE.

```bash
./setup_camera_sync.sh --eth enp0s31f6 --ips 192.168.1.20,192.168.1.21,192.168.1.22,192.168.1.23
```

Steps 2 and 3 are independent of each other — integrate LiDARs and cameras in any order. Each script includes a self-test at the end that verifies PTP sync quality, service status, and sensor reachability.

---

## 2. Prerequisites (Manual Steps)

The setup scripts handle most installation, but two steps require manual action before running them.

### 2.1 Install the PREEMPT_RT Kernel

Ubuntu Pro is required (free for personal use on up to 5 machines). The scripts do not install the kernel because it requires your personal Pro token and a reboot.

```bash
# Get a free token at ubuntu.com/pro
sudo pro attach YOUR_TOKEN_HERE
sudo pro status
sudo pro enable realtime-kernel
sudo reboot
```

After reboot, verify:

```bash
uname -a
# Should show: ... SMP PREEMPT_RT ...
```

**NVIDIA GPU note:** The NVIDIA kernel module (`nvidia.ko`) does not load on the PREEMPT_RT kernel. No CUDA, cuDNN, or TensorRT is available when booted into the RT kernel. Use Intel iGPU for display. See [Appendix C](#c-nvidia-gpu-and-rt-kernel-compatibility) for recommended workflows (record on RT, process on generic).

### 2.2 Identify Your Ethernet Interface

Find the interface name connected to the sensor network — you will pass it to all three scripts via `--eth`:

```bash
ip link show
# Look for your wired Ethernet (e.g., enp0s31f6, eth0, eno1)
```

---

## 3. Time Synchronization Architecture

Accurate time synchronization across all sensors is critical for sensor fusion. The Atlas Duo's GPS-disciplined PPS signal disciplines the host system clock, which is then distributed to all sensors via IEEE 1588 PTP.

```
┌─────────────────────────┐
│  GPS Satellites          │
└───────────┬─────────────┘
            │ RF
┌───────────▼─────────────┐
│  Point One Nav Atlas Duo │  ← GPS-disciplined clock (<20 ns accuracy)
│  - PPS output (1Hz pulse)│
│  - NMEA/FusionEngine     │
│  - IMU @ 200Hz           │
└─────┬──────────┬────────┘
      │ PPS      │ NMEA serial
      │ /dev/pps0│ /dev/ttyUSB0
┌─────▼──────────▼─────────────────────────────────┐
│  Ubuntu 24.04 Host (PTP Grandmaster)              │
│                                                    │
│  gpsd ← PPS + NMEA → shared memory (SHM)          │
│  chrony ← SHM → CLOCK_REALTIME (<100 ns to GPS)   │
│  phc2sys: CLOCK_REALTIME → NIC PHC (/dev/ptp0)    │
│  ptp4l:   NIC PHC → PTP announce on Ethernet       │
└──┬────────┬────────┬────────┬────────────────────┘
   │ PTP    │ PTP    │ PTP    │ PTP (GigE Vision IEEE 1588)
┌──▼──────┐┌▼──────┐┌▼──────┐┌▼──────────────────────┐
│Robin W  ││Robin W││Robin W││ 4× RouteCAM            │
│Front    ││Rear-L ││Rear-R ││ P_CU25_CXLC_IP67       │
│(0°)     ││(120°) ││(240°) ││ PoE GigE Vision slaves │
└─────────┘└───────┘└───────┘└────────────────────────┘
```

**Expected synchronization accuracy at each stage:**

| Component | Accuracy |
|-----------|----------|
| Atlas Duo GNSS PPS | < 20 ns to UTC |
| chrony (GPS-disciplined CLOCK_REALTIME) | < 100 ns |
| NIC PHC via phc2sys | < 200 ns |
| LiDAR PTP slave | 300–800 ns |
| RouteCAM GigE Vision PTP slave | 1–10 µs |

> The RouteCAM number assumes a managed PoE switch with an IEEE 1588 boundary clock (e.g. Planet WGS-6325-8UP2X — see §3.1). Through an unmanaged switch (no boundary clock), expect 5–50 µs instead.

**Critical:** The host clock MUST be disciplined by GPS (via the Atlas Duo PPS), not by internet NTP alone. NTP provides only ~1–10 ms accuracy, whereas GPS PPS provides < 100 ns.

### 3.1 Network Requirements

| Configuration | Estimated Bandwidth | Minimum NIC |
|--------------|-------------------|-------------|
| 1 Robin W | ~60 Mbps | 1 GbE |
| 3 Robin W | ~180 Mbps | 1 GbE |
| 4 RouteCAM (2MP @ 20fps) | ~400 Mbps | 1 GbE |
| 3 Robin W + 4 RouteCAM + PTP | ~600 Mbps + overhead | 1 GbE OK; **10 GbE recommended** for headroom and jumbo-frame stability |

> **Robin W raw bandwidth:** the Seyond datasheet lists the Robin W output at ~60 Mbps per unit. Earlier drafts of this doc cited 150 Mbps (conflating peak burst with sustained), which overestimates the aggregate by roughly 3×. 3× Robin W = ~180 Mbps sustained; plan jitter/burst headroom at ~1.5× for safety.

**Reference network design**

The recommended hardware combination is a **Teltonika RUTM50 / RUTM54** 5G/4G cellular router (cellular WAN gateway) paired with a **Planet WGS-6325-8UP2X** managed PoE++ switch (sensor LAN with IEEE 1588 boundary clock and 10 GbE uplink). The host PC connects directly to the switch's 10 G SFP+ port — never through the cellular router — so PTP messages travel `host → boundary clock → sensor` and never cross the non-PTP-aware router.

```
Internet (5G/4G)
       │
       ▼
┌──────────────────────────┐
│ Teltonika RUTM50/RUTM54  │  192.168.1.1
│  (cellular gateway only) │  WAN port unused
└─────────────┬────────────┘
              │ 1 GbE  (RUTM50 LAN1 → Planet GbE port)
              ▼
┌────────────────────────────────────────────────────────┐
│ Planet WGS-6325-8UP2X (managed L3, IEEE 1588 BC, PoE++)│  192.168.1.2 (mgmt)
├────────────────────────────────────────────────────────┤
│ Port    Type            Device                  IP      │
│ ----    ------------    --------------------    ------ │
│ 1       1 GbE           → RUTM50 LAN1           (uplink)
│ 2       2.5 GbE PoE++   RouteCAM Front-Right    .20    │
│ 3       2.5 GbE PoE++   RouteCAM Front-Left     .21    │
│ 4       2.5 GbE PoE++   RouteCAM Rear-Left      .22    │
│ 5       2.5 GbE PoE++   RouteCAM Rear-Right     .23    │
│ 6       1 GbE  PoE++    Robin W Front           .10    │
│ 7       1 GbE  PoE++    Robin W Rear-Left       .11    │
│ 8       1 GbE  PoE++    Robin W Rear-Right      .12    │
│ SFP1    10 G            Host PC (10G NIC)       .40    │
│ SFP2    10 G            spare (Atlas Duo Eth / NAS)    │
└────────────────────────────────────────────────────────┘

Atlas Duo INS — three physical paths to the host (not all over Ethernet):
  PPS  ──► host /dev/pps0     (BNC, hardware PPS — absolute time origin)
  USB  ──► host /dev/ttyUSB0  (NMEA + FusionEngine messages)
  Eth  ──► Planet switch      (192.168.1.30, NTRIP RTK corrections only)
```

**IP plan** — every static device sits below `.100`, so the RUTM50's factory DHCP pool (`.100–.249`) stays untouched and no router configuration is required:

| 192.168.1.x | Role |
|-------------|------|
| .1 | RUTM50 router + DHCP server + default gateway |
| .2 | Planet WGS-6325-8UP2X management interface |
| .10 – .12 | Robin W LiDARs (static on sensor) |
| .20 – .23 | RouteCAM cameras (static via Aravis web UI) |
| .30 | Atlas Duo INS Ethernet (static, NTRIP RTK only) |
| .40 | Host PC 10 GbE NIC (static via netplan / NetworkManager) |
| .100 – .249 | RUTM50 DHCP pool — factory default, untouched |

**Why the host connects to the switch, not the router**

1. *PTP integrity.* The host is the PTP grandmaster. Its sync messages must reach every slave through PTP-aware hardware. The RUTM50 is not a PTP boundary clock; if the host sat on a RUTM50 LAN port, every PTP packet would traverse the router and pick up residence-time jitter that defeats the entire purpose of the boundary-clock switch.
2. *Bandwidth.* ~600 Mbps of sensor traffic onto a 1 GbE LAN port runs at ~60% saturation. A 10 G SFP+ link host-to-switch eliminates the bottleneck and gives headroom for jumbo frames and bursty camera traffic.

The RUTM50 → Planet uplink carries only NTRIP RTCM3 (~5 kbps) and any host-side internet traffic (SSH, OS updates, NTP fallback). It runs at ~0.1% utilization. Sensor data never leaves the Planet switch.

**Atlas Duo Ethernet (RTK NTRIP)**

The Atlas Duo's Ethernet port has one job in this design: pull RTK NTRIP corrections (RTCM3) from your caster (Trimble VRS Now, your local base, etc.). Configure via the Atlas web UI: static IP `192.168.1.30`, gateway `192.168.1.1`, DNS `192.168.1.1`, and NTRIP client pointed at your caster. PPS still travels its own physical BNC wire to `/dev/pps0` and remains the absolute time origin — the Ethernet path adds RTK without altering the time-sync chain.

### 3.2 Hardware vs Software Timestamping

Check your NIC's capabilities with `ethtool -T <interface>`. The setup scripts auto-detect this.

| Timestamping | PTP Accuracy | Sufficient For |
|-------------|-------------|----------------|
| Hardware | < 1 µs | Production sensor fusion |
| Software | 20–50 µs | Development, mapping, general robotics |

Most Intel NICs (I210, I225, X550, X710) support hardware timestamping. USB Ethernet adapters typically do not.

---

## 4. Verification (Manual)

After running the setup scripts, verify the full synchronization chain. The scripts run self-tests automatically, but you can re-check manually at any time.

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

## 5. Recording Data

### 5.1 Recording Architecture

This system uses a **zero-ROS recording approach** for maximum performance:

| Sensor | Capture Method | Format | CPU Load |
|--------|---------------|--------|----------|
| Seyond Robin W LiDARs | `tcpdump` (kernel-level) | .pcap | ~1% per LiDAR |
| Point One Nav Atlas Duo | `p1_runner` (native binary) | .p1log | ~1% |
| RouteCAM cameras | Aravis / `tcpdump` | .pcap or raw frames | ~2–5% per camera |

Compared to rosbag recording (~30–50% CPU), this skips point cloud decoding, ROS serialization, and DDS middleware during capture. Data is decoded on replay.

### 5.2 Configure Atlas Duo Message Rates

```bash
cd ~/p1-host-tools

python3 bin/config_tool.py apply uart2_message_rate fe ROSPoseMessage 100ms
python3 bin/config_tool.py apply uart2_message_rate fe ROSGPSFixMessage 100ms
python3 bin/config_tool.py apply uart2_message_rate fe ROSIMUMessage on
python3 bin/config_tool.py save
```

### 5.3 Recording with the Fast Recorder Script

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

Interactive commands: `R` start recording, `S` stop, `H` health, `Q` quit.

### 5.4 Session Output Structure

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

All sensors are PTP-synchronized, so timestamps share the same GPS time base and can be aligned in post-processing without additional clock correction.

### 5.5 YAML Configuration Reference

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

### 5.6 Alternative: ROS 2 Native Recording

If you prefer rosbag (higher CPU but simpler replay):

```bash
# Terminal 1 — Point One Nav
ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args \
    -p connection_type:=tty -p tty_port:=/dev/ttyUSB0

# Terminal 2 — Seyond Robin W
ros2 launch seyond start.py

# Terminal 3 — Record
ros2 bag record -a -o my_dataset
```

Ensure `use_sim_time` is `false` in all nodes. When `use_sim_time` is `false`, ROS 2 message headers use `CLOCK_REALTIME`, which is GPS-disciplined through the PTP chain.

```bash
# Verify timestamps are realistic (not zeros)
ros2 topic echo /robin_w_front/points --field header.stamp --once
```

---

## 6. Replay and Visualization

### 6.1 Replay Point One Nav Data (.p1log)

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

### 6.2 Replay Seyond Robin W PCAPs into ROS 2

```bash
ros2 launch seyond start.py \
    pcap_file:=$SESSION/lidar_pcap/robin_w_front.pcap \
    lidar_name:=robin_w_front \
    frame_id:=robin_w_front \
    frame_topic:=/robin_w_front/points
```

### 6.3 Visualize in RViz2 / Foxglove Studio

```bash
# RViz2
rviz2
# Add → By topic → PointCloud2. Set Fixed Frame. Set Point Size ~0.02.

# Foxglove Studio
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
foxglove-studio
# Connect to ws://localhost:8765
```

### 6.4 Inspect Raw PCAPs

```bash
tshark -r $SESSION/lidar_pcap/robin_w_front.pcap -q -z io,stat,1
wireshark $SESSION/lidar_pcap/robin_w_front.pcap
```

### 6.5 Convert Replayed Data to Rosbag

```bash
ros2 bag record -o $SESSION/replayed_rosbag \
    /robin_w_front/points /robin_w_left/points /robin_w_right/points \
    /tf /tf_static
```

### 6.6 Tool Summary

| Tool | Purpose | Input |
|------|---------|-------|
| `p1_display` | Interactive trajectory/IMU/GNSS plots | .p1log |
| `p1_extract` | Export to CSV / KML | .p1log |
| `p1_print` | Decode messages to terminal | .p1log |
| `rviz2` | 3D point cloud visualization | ROS 2 topics |
| `foxglove-studio` | Multi-panel sensor dashboard | ROS 2 topics |
| `tshark` / `wireshark` | Raw packet inspection | .pcap |
| `arv-viewer-0.8` | Live GigE Vision camera viewer | Camera stream |

---

## Appendix

### A. Seyond Robin W Default Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| IP Address | 192.168.1.10 | Change via web UI or `innovusion_lidar_util` |
| Data Port | 8010 | TCP and UDP |
| Coordinate Mode | 3 (forward/left/up) | Matches ROS REP-103 |
| PTP | Supported | Enabled by `setup_robin_w_sync.sh` |
| FOV | 120° × 70° | |
| Points/sec | 1.28M | 192 scan lines |
| Range | 0.1–150 m | 70 m at 10% reflectivity |

### B. RouteCAM_P_CU25_CXLC_IP67 Key Specs

| Feature | Value |
|---------|-------|
| Sensor | AR0234 1/2.6" 2MP global shutter |
| Resolution | 1920 × 1200 |
| FOV | 158° DFOV, 134° HFOV, 73° VFOV |
| Interface | GigE Vision (M12 X-coded Ethernet) |
| Power | PoE (IEEE 802.3af) |
| Time Sync | IEEE 1588 PTP via GigE Vision |
| Protection | IP67 |
| Dome layout | Front stereo pair (104 mm baseline) + rear symmetric pair |

### C. NVIDIA GPU and RT Kernel Compatibility

The NVIDIA kernel module (`nvidia.ko`) cannot load on the PREEMPT_RT kernel. No CUDA, cuDNN, TensorRT, or GPU compute is available on the RT kernel.

| Kernel | CUDA Available | Best For |
|--------|---------------|----------|
| **Generic** | Yes | Training, simulation, GPU inference |
| **RT** | **No** | Deterministic sensor recording, real-time control |

**Recommended workflow:** Record on the RT kernel (deterministic PTP timing, no GPU needed), then reboot into the generic kernel for post-processing with full CUDA access.

```
Field recording:   RT kernel → tcpdump + p1_runner (no GPU needed)
Post-processing:   Generic kernel → CUDA + PyTorch/TensorRT on recorded data
```

To select the kernel at boot, choose **Advanced options for Ubuntu** in the GRUB menu, then pick the `*-realtime` or `*-generic` entry.

### D. Troubleshooting

**PTP not syncing (large master offset):** Verify the interface name in the ptp4l config matches your actual interface. Confirm PTP is enabled on each Robin W. Check cable connections.

**chronyc shows NTP as primary (not PPS/NMEA):** gpsd may not be providing time to chrony. Run `sudo systemctl status gpsd` and `gpsmon`. If `/dev/pps0` does not exist, PPS is not exposed — check your serial connection type.

**LiDAR packet drops:** Likely network bandwidth saturation. Check with `sudo ethtool -S eth0 | grep -i drop`. For 3 Robin W LiDARs + 4 cameras, a 10 GbE NIC is required.

**No GigE Vision cameras detected:** Check PoE switch power, verify cameras are on the same subnet, and try `arv-tool-0.8` to scan. Firewall rules may block GigE Vision discovery packets.

**Timestamps misaligned in rosbag:** Verify `use_sim_time` is `false` in all ROS 2 nodes. Check `chronyc tracking` to confirm GPS discipline is active.

**gpsd shows "NO FIX":** Ensure the Atlas Duo antenna has clear sky view. Cold start may take up to 30 minutes. Verify the correct baud rate (Atlas Duo defaults to 460800).

**`<cstdint>` build errors:** GCC 14 on Ubuntu 24.04 is stricter. The `setup_ubuntu_sync.sh` script patches this automatically.

**Seyond build fails:** Use `./build.bash` inside `seyond_ros_driver/` (not `colcon build` from the workspace root). The script `setup_robin_w_sync.sh` handles this.
