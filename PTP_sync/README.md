# Sensor Recording System: Point One Nav Atlas Duo + Seyond Robin W LiDARs

Installation and configuration guide for a high-performance GNSS / IMU / LiDAR
recording system running on Ubuntu 22.04 with ROS 2 Humble or Ubuntu 24.04 with
ROS 2 Jazzy, optionally on a PREEMPT_RT real-time kernel. RouteCAM setup is an
optional extension and is not required by the recorder, GICP++, or GLIM++.

**Target Hardware:**
- Ubuntu 22.04 or 24.04 LTS workstation (tested on Lenovo ThinkPad P1 Gen 6, Intel i9-13900H)
- Point One Nav Atlas Duo (GNSS / INS, Ethernet-only)
- Up to 3× Seyond Robin W directional LiDARs
- Optional: 0–4× e-con RouteCAM_P_CU25_CXLC_IP67 GigE Vision cameras

**Setup flow.** Run scripts 1–4 for the supported camera-free P1 + LiDAR
pipeline. Run script 5 only when RouteCAMs are physically installed. Each
script handles one logical step and has its own self-test.

| # | Script | What it does |
|---|--------|--------------|
| 1 | [`1_install_packages.sh`](1_install_packages.sh) | apt prerequisites, RT scheduling permissions, kernel tuning, ROS 2 Humble/Jazzy |
| 2 | [`2_configure_host_network.sh`](2_configure_host_network.sh) | Host NIC static IP, hardware-timestamping detection, RUTM50 reachability |
| 3 | [`3_setup_ins_to_pc_sync.sh`](3_setup_ins_to_pc_sync.sh) | gpsd (NMEA over TCP), chrony, ptp4l grandmaster, phc2sys, Point One host tools |
| 4 | [`4_setup_lidar_ptp.sh`](4_setup_lidar_ptp.sh) | Robin W PTP slave enable + Seyond ROS 2 driver |
| 5 | [`5_setup_camera_ptp.sh`](5_setup_camera_ptp.sh) | **Optional:** RouteCAM PTP slave enable + Aravis (Tier 2 only) |

A one-time per-LiDAR provisioning step lives in [`provision_robin_w_multiunit.sh`](provision_robin_w_multiunit.sh) — see Section 4.

**Network defaults.** Every script reads its defaults from [`../config/network_config.yaml`](../config/network_config.yaml). Edit that file once for your installation (mainly `host.interface`) and the scripts pick up everything else (host IP, sensor IPs, gateway, DHCP pool). CLI flags and environment variables still override the YAML at runtime — see each script's `--help` style usage block.

---

## Section 1: Install Ubuntu Real-Time Kernel and Other Packages

This section installs everything the host needs that is *not* network or PTP configuration: apt prerequisites, real-time scheduling permissions, kernel `sysctl` tuning, and ROS 2 Humble or Jazzy.

### 1.1 PREEMPT_RT kernel — needed only for hard real-time control

**You do not need the RT kernel just to record sensor data.** For perception and localization data collection — the [`recording/`](../recording/) workflow that captures GNSS / IMU / LiDAR / camera into a Foxglove-native MCAP bag for offline mapping, perception training, or SLAM evaluation — the **stock Ubuntu LTS generic kernel is sufficient**. PTP timing on the generic kernel typically lands within 1–2× of the RT-kernel numbers in Section 3, which is well inside what every sensor in this dome can resolve.

The RT kernel matters when the host has to *act* on sensor data with bounded latency: closing a real-time control loop (steering, braking, manipulator servoing), running a deterministic safety monitor, or any scenario where a millisecond of scheduler jitter is unacceptable. If your application is "record now, process later," skip this subsection entirely.

| Use case | Kernel | Why |
|----------|--------|-----|
| Recording for mapping, perception, training, offline analysis | **Generic (default)** | Simpler install, full NVIDIA / CUDA / TensorRT, PTP still meets sensor-fusion requirements |
| Real-time control, deterministic safety loops, hardware-in-the-loop | PREEMPT_RT | Bounded scheduling latency at the cost of CUDA support — see Appendix C |

If you do need RT, install via Ubuntu Pro (free for personal use on up to 5 machines), then reboot before continuing:

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

**NVIDIA GPU note (RT kernel only):** the NVIDIA kernel module (`nvidia.ko`) does not load on PREEMPT_RT — no CUDA, cuDNN, or TensorRT when booted into the RT kernel. Use Intel iGPU for display. See [Appendix C](#c-nvidia-gpu-and-rt-kernel-compatibility) for the dual-boot workflow.

### 1.2 Run the install script

```bash
chmod +x 1_install_packages.sh
./1_install_packages.sh

# Humble host:
./1_install_packages.sh --ros-distro humble
```

The default is `ROS_DISTRO=jazzy`. Use `--ros-distro humble` or set
`ROS_DISTRO=humble` before running scripts 1, 3, 4, and 5 on Ubuntu 22.04
hosts. Use Jazzy on Ubuntu 24.04 hosts.

What it installs:

- **apt prerequisites:** `build-essential`, `cmake`, `git`, `linuxptp`, `chrony`, `gpsd`, `pps-tools`, `tcpdump`, `ethtool`, `libyaml-cpp-dev`, Python tooling.
- **RT scheduling group + limits:** creates the `realtime` group, adds your user, writes `/etc/security/limits.d/99-realtime.conf` granting `rtprio 99` and `memlock unlimited` to that group. Whether or not you're on the RT kernel, these limits are required for the PTP daemons to run at real-time priority.
- **Kernel `sysctl` tuning:** large UDP buffers (`net.core.rmem_max=32 MiB`) and `vm.swappiness=10` for the high-bandwidth LiDAR / camera streams.
- **ROS 2 Humble/Jazzy:** `ros-base` + dev tools + `rviz2` + `foxglove-bridge` + `pcl-ros` + `tf2-tools` + rosbag2 MCAP/default storage plugins. The default setup does not install camera packages; the optional script 5 provisions those separately. Sources `/opt/ros/$ROS_DISTRO/setup.bash` from `~/.bashrc`.
- **`tcpdump` cap:** `cap_net_raw+ep` so the recorder can sniff sensor UDP without `sudo`.

The script's self-test at the end verifies the RT kernel banner, group membership, the `sysctl` value, and the ROS 2 install.

You will need to **log out and back in** before group membership takes effect — that's the only manual step.

---

## Section 2: Reference Network Configuration on RUTM50 and a PoE Switch

The sensor LAN is built around a **Teltonika RUTM50** 5G/4G cellular router. The reference ships in **two tiers** so a LiDAR-only build doesn't pay for hardware it doesn't need:

- **Tier 1** uses the RUTM50 alone and skips the managed PoE switch — sufficient for 3× LiDARs + INS + PC.
- **Tier 2** keeps Tier 1 untouched and adds a **Planet WGS-6325-8UP2X** PoE++ switch for the 4× RouteCAM cameras (the cameras need PoE; the switch's IEEE 1588 boundary clock is a bonus).

### 2.1 Tier 1 — LiDAR-only (5 devices, RUTM50 only, no separate switch)

The RUTM50 has 5× RJ45 Gigabit ports (1 WAN + 4 LAN). With the WAN port reconfigured as LAN (RutOS WebUI → Network → LAN → "Use WAN port as LAN" toggle — see Teltonika wiki [Setting up WAN as LAN](https://wiki.teltonika-networks.com/view/Setting_up_WAN_as_LAN)), all 5 ports become LAN and you get exactly the count you need: 1 PC + 1 INS + 3 LiDARs.

```
Internet (5G cellular)
       │
       ▼
┌─────────────────────────────────────────────────────────────┐
│ Teltonika RUTM50                          192.168.1.1       │
│  WAN-as-LAN configured                    (cellular WAN     │
│  DHCP pool .100–.249 (factory default)     stays internal)  │
├─────────────────────────────────────────────────────────────┤
│ Port           Device                              IP        │
│ ----           ------------------------            ------    │
│ WAN-as-LAN     Host PC                             .5        │
│ LAN 1          Robin W Front                       .10       │
│ LAN 2          Robin W Rear-Left                   .11       │
│ LAN 3          Robin W Rear-Right                  .12       │
│ LAN 4          Atlas Duo Eth (NTRIP)               .30       │
└─────────────────────────────────────────────────────────────┘

Atlas Duo INS — single Ethernet path to the host:
  Eth  ──► RUTM50 LAN 4       (192.168.1.30)
                              ├── FusionEngine over TCP 30201 → host
                              │    (/pose, /imu, /gps/fix, /odom)
                              ├── NMEA 0183 over TCP 30200 → host
                              │    (gpsd → chrony → CLOCK_REALTIME, ~10–100 ms accuracy)
                              └── NTRIP RTCM3 from caster → Atlas
                                   (RTK corrections, ~5 kbps)
```

> **Ethernet-only Atlas Duo connection.** This is the only path the Atlas Duo exposes for time + data — its hardware does not provide a BNC PPS pin or a USB serial NMEA stream. One cable from the Atlas Duo to RUTM50 LAN 4 carries everything: FusionEngine (pose / IMU / GPSFix / odom) over TCP 30201, NMEA 0183 over TCP 30200 for chrony, NTRIP RTCM3 inbound from the caster, and the device's REST API over HTTP. Consequences: host `CLOCK_REALTIME` discipline is ~10–100 ms (NMEA-only via gpsd) instead of the sub-100 ns a hardware PPS would give; cross-sensor PTP sync is **unaffected** because the host's NIC PHC is still the PTP grandmaster on the LAN, and LiDARs/cameras stay in the 5–50 µs / 1–10 µs ranges (see §3 / §4 / §5 tables); absolute GPS time is **also unaffected** for any consumer that reads it from FusionEngine message headers, since each message carries its own GPS timestamp built in by the Atlas.

**LiDAR power in Tier 1.** Robin W can run on PoE (IEEE 802.3af) **or** 12 V DC; RUTM50 LAN ports do **not** source PoE. In Tier 1 you have two options:

- **Recommended:** power each LiDAR over its M12 connector with a 12 V DC supply (one per LiDAR or a single multi-output bench supply).
- **Cheap alternative:** put a passive PoE injector inline on each LiDAR Ethernet run with a 48 V DC brick.

The Atlas Duo is powered separately via its own DC barrel input either way.

**PTP timing trade-off in Tier 1.** The RUTM50 is **not** a PTP boundary clock — its internal switch fabric just forwards Ethernet frames. PTP messages from the host PC pick up a few microseconds of residence-time jitter as they cross the RUTM50 to reach the LiDARs. Expected sync at each stage:

| Component | Tier 1 (RUTM50 only) | Tier 2 (Planet BC) | Sensor fusion OK? |
|-----------|----------------------|--------------------|-------------------|
| chrony GPS-disciplined `CLOCK_REALTIME` | ~10–100 ms (NMEA-only) | ~10–100 ms (NMEA-only) | yes for ROS messages that carry Atlas GPS time |
| NIC PHC via `phc2sys` | < 1 µs when hardware timestamping is available | < 1 µs when hardware timestamping is available | yes; skipped in software timestamping mode |
| Robin W LiDAR PTP slave | 5–50 µs | 300–800 ns | yes (both) |

For LiDAR-IMU fusion, 5–50 µs is completely fine — Robin W frames are ~50–100 ms long at 10–20 FPS, so a few-µs cross-sensor offset is well under one frame period. The dramatic improvement on row 3 only matters when you need camera/LiDAR pixel-perfect alignment, which is exactly what Tier 2 brings.

### 2.2 Tier 2 — adds 4× RouteCAM cameras (Planet WGS-6325-8UP2X added)

The cameras (e-con RouteCAM_P_CU25_CXLC_IP67) are PoE-only and benefit substantially from a real PTP boundary clock. Adding the Planet WGS-6325-8UP2X downstream of the RUTM50 covers both needs: 4× PoE++ ports for the cameras and IEEE 1588 BC for tight cross-sensor sync.

Two ways to wire Tier 2 — pick whichever fits your build:

**(a) Cameras-only on Planet, Tier 1 untouched.** Simplest upgrade — the LiDARs, INS, and PC stay on the RUTM50; only the cameras live on the Planet switch. Uplink the Planet's port 1 to a free RUTM50 LAN port so the cameras can reach the host. LiDAR PTP stays in the 5–50 µs range from Tier 1; camera PTP picks up the Planet's boundary clock (1–10 µs). Good if you want minimal rewiring.

**(b) Move everything onto the Planet.** Host PC migrates to the Planet's SFP1 (10 GbE), all LiDARs migrate to PoE++ ports 6–8, and the RUTM50 becomes purely an uplink to the internet for NTRIP corrections. Every sensor now sits on a PTP-aware fabric and LiDAR PTP tightens to 300–800 ns. Required if you also want jumbo-frame stability under the full ~600 Mbps aggregate sensor load (~60% of a 1 GbE link is uncomfortable; a 10 GbE host link removes the bottleneck). The original Hitch dome reference design used this layout.

Layout (b), for reference:

```
Internet (5G cellular)
       │
       ▼
┌──────────────────────────┐
│ Teltonika RUTM50         │  192.168.1.1
│  (cellular gateway only) │  WAN-as-LAN no longer needed
└─────────────┬────────────┘
              │ 1 GbE  (RUTM50 LAN1 → Planet GbE port)
              ▼
┌────────────────────────────────────────────────────────┐
│ Planet WGS-6325-8UP2X (managed L3, IEEE 1588 BC, PoE++)│  192.168.1.2 (mgmt)
├────────────────────────────────────────────────────────┤
│ Port    Type            Device                  IP      │
│ ----    ------------    --------------------    ------ │
│ 1       1 GbE           → RUTM50 LAN1           (uplink)
│ 2       2.5 GbE PoE++   RouteCAM Front-Left     .20    │
│ 3       2.5 GbE PoE++   RouteCAM Front-Right    .21    │
│ 4       2.5 GbE PoE++   RouteCAM Rear-Left      .22    │
│ 5       2.5 GbE PoE++   RouteCAM Rear-Right     .23    │
│ 6       1 GbE  PoE++    Robin W Front           .10    │
│ 7       1 GbE  PoE++    Robin W Rear-Left       .11    │
│ 8       1 GbE  PoE++    Robin W Rear-Right      .12    │
│ SFP1    10 G            Host PC (10G NIC)       .5     │
│ SFP2    10 G            spare (Atlas Duo Eth / NAS)    │
└────────────────────────────────────────────────────────┘
```

### 2.3 IP plan

Same across Tier 1 and Tier 2. Every static device sits below `.100`, so the RUTM50's factory DHCP pool (`.100–.249`) stays untouched and no router configuration beyond static-lease reservations is required.

| 192.168.1.x | Role | Tier 1 | Tier 2 |
|-------------|------|--------|--------|
| .1 | RUTM50 router + DHCP server + default gateway | ✓ | ✓ |
| .2 | Planet WGS-6325-8UP2X management interface | — | ✓ |
| .5 | Host PC NIC (static via netplan / NetworkManager) | ✓ | ✓ |
| .10 – .12 | Robin W LiDARs (static on sensor) | ✓ | ✓ |
| .20 – .23 | RouteCAM cameras (static via Aravis web UI) | — | ✓ |
| .30 | Atlas Duo INS Ethernet (static, NTRIP RTK only) | ✓ | ✓ |
| .100 – .249 | RUTM50 DHCP pool — factory default, untouched | ✓ | ✓ |

> **Why `.5` and `.30` are pinned to specific addresses.** Static-lease the PC at .5 and the Atlas Duo at .30 via the RUTM50's Network → LAN → Static Leases page (bind to MAC). This way both devices stay at known IPs across reboots without any device-side static config — the RUTM50 always hands them the same address. Procedure: RUTM50 web UI → Network → LAN → Static Leases → Add, then bind each device's MAC to the address you want it to keep. Reboot both devices and confirm they come back on the same IPs before running any of the scripts here — every step below assumes those addresses are stable.

**Atlas Duo Ethernet (time + data + NTRIP).** The Atlas Duo's Ethernet port is the only interface it exposes for time and data — its hardware does not have a BNC PPS output or a USB serial data output. The same cable that carries FusionEngine (TCP 30201) and NMEA (TCP 30200) to the host also pulls RTK NTRIP corrections (RTCM3) from your caster (Trimble VRS Now, your local base, etc.). Configure via the Atlas web UI: static IP `192.168.1.30` (or DHCP + reservation), gateway `192.168.1.1`, DNS `192.168.1.1`, and NTRIP client pointed at your caster. The time-sync chain (gpsd → chrony → ptp4l) all rides on this same Ethernet path — see §3 for the host-side configuration.

### 2.4 Network bandwidth at each tier

| Configuration | Estimated Bandwidth | Minimum NIC | Tier |
|---------------|---------------------|-------------|------|
| 1 Robin W | ~60 Mbps | 1 GbE | Tier 1 |
| 3 Robin W | ~180 Mbps | 1 GbE | Tier 1 |
| 4 RouteCAM (2MP @ 20fps) | ~400 Mbps | 1 GbE | requires Tier 2 |
| 3 Robin W + 4 RouteCAM + PTP | ~600 Mbps + overhead | 1 GbE OK; **10 GbE recommended** for headroom and jumbo-frame stability | Tier 2 (b) |

> **Robin W raw bandwidth:** the Seyond datasheet lists Robin W output at ~60 Mbps per unit (sustained, not the peak-burst number cited in some earlier drafts). 3× Robin W ≈ 180 Mbps sustained; plan jitter/burst headroom at ~1.5×.

### 2.5 Run the host-network script

```bash
chmod +x 2_configure_host_network.sh
./2_configure_host_network.sh
```

What it does:

- Brings the sensor-side NIC up with the static IP from `network_config.yaml` (default `192.168.1.5/24`).
- Probes IEEE 1588 capability via `ethtool -T` and decides hardware vs software timestamping. The choice is written to `/run/hitch_dome_net.env` so script 3 picks the right `ptp4l` mode automatically.
- Pings the RUTM50 router as a sanity check.
- Optionally adds a temporary `172.168.1.100/24` alias on the sensor NIC for factory-state LiDAR provisioning (see Section 4) when called with `--add-factory-alias`.

**Find your interface name first** if you're not sure:

```bash
ip link show
# Look for your wired Ethernet (e.g., enp0s31f6, eth0, eno1)
```

### 2.6 Hardware vs software PTP timestamping

Check your NIC's capability with `ethtool -T <interface>` — script 2 does this for you and writes the result to `/run/hitch_dome_net.env`.

| Timestamping | PTP Accuracy | Sufficient For |
|--------------|--------------|----------------|
| Hardware | < 1 µs | Production sensor fusion |
| Software | 20–50 µs | Development, mapping, general robotics |

Most Intel NICs (I210, I225, X550, X710) support hardware timestamping. USB Ethernet adapters typically do not.

### 2.7 One-time per-LiDAR network provisioning

Every brand-new Robin W ships from Seyond with the same factory IP `172.168.1.10` (per [*Robin W1G User Manual* V2.2 §3.1](https://www.seyond.com/wp-content/uploads/2025/03/Seyond-Robin-W1G-LiDAR_User-Manual_V2.2_EN_Public_20250103.pdf)) and the same UDP port. Running three units on the same Ethernet segment needs a unique IP **and** a unique UDP destination port per unit — the dome assigns `.10` / `.11` / `.12` and ports `8337` / `8338` / `8339`. This is purely network configuration (IPs + ports), so it lives here in Section 2; PTP enable comes later in Section 4.

The provisioning is **one Robin W at a time**, by physical position. Because all three factory-state units share the same IP `172.168.1.10`, only one can be on the wire at a time during the renumber step.

**Decide which physical unit is which position before you start.** The dome SCAD model defines three positions:

| Position | Dome angle | Target IP | UDP port |
|----------|------------|-----------|----------|
| `front` | 0° (faces +X / forward) | `192.168.1.10` | `8337` |
| `rear_left` | 120° (faces rear-left) | `192.168.1.11` | `8338` |
| `rear_right` | 240° (faces rear-right) | `192.168.1.12` | `8339` |

Easiest: **label the three units with masking tape (FRONT / REAR-LEFT / REAR-RIGHT) before you start**, then provision them in that order. If you'd rather choose based on physical condition (cleaner enclosure to the visible front, etc.), connect one at a time and read each unit's serial from `http://172.168.1.10/api/v1/static_info` before deciding.

**Walk-through:**

```bash
# Attach Robin W #1 to the host's sensor NIC via Ethernet. The unit
# will come up at the factory IP 172.168.1.10. Then:
chmod +x provision_robin_w_multiunit.sh
./provision_robin_w_multiunit.sh --position front

# Power off / disconnect Robin W #1, attach Robin W #2:
./provision_robin_w_multiunit.sh --position rear_left

# And finally Robin W #3:
./provision_robin_w_multiunit.sh --position rear_right
```

What the script does on each invocation:

1. Adds a temporary `172.168.1.100/24` alias to the sensor NIC so the host can reach the factory-IP LiDAR (removed on exit).
2. Discovers the attached unit (either at the factory IP `172.168.1.10` for a brand-new unit, or at the position's target IP for a re-run).
3. Reads the LiDAR's serial number via `innovusion_lidar_util get_static_info`.
4. Calls `set_network` to move the unit from `172.168.1.10` → `192.168.1.10` / `.11` / `.12`, reboots, waits ~25 s.
5. Uploads the position-specific `PCS_ENV` (`RobinW_FW2835_Multiunit/RW_FW2835_robin_w_<position>_unicast.env`), reboots again.
6. Appends or updates the unit's serial in `RobinW_FW2835_Multiunit/serial_inventory.yaml` so the SN ↔ position map is preserved for spares / RMA tracking. That file is **generated by this script at provisioning time** — it is not in the repository, and a fresh clone will not have one until you provision your first unit.

The script is idempotent — re-running it against an already-provisioned LiDAR detects matching state and prints `[SKIP]`. It also refuses to overwrite a position whose inventory entry shows a different serial than the unit currently attached, catching the "I plugged in the wrong unit" mistake. If a Robin W has to be swapped (RMA, failure), delete that position's block in `serial_inventory.yaml` and re-run the script with the appropriate `--position`.

After all three positions show up in `serial_inventory.yaml`, the network is fully configured and you're ready for Section 3 (INS-to-PC time sync) and Section 4 (LiDAR PTP enable + ROS 2 driver).

---

## Section 3: PTP Sync from INS to PC

The Point One Nav Atlas Duo serves NMEA over TCP to discipline the host's
`CLOCK_REALTIME`. Its FusionEngine stream is consumed separately by the
deployment's native-message ROS driver. The host then acts as the PTP
grandmaster on the sensor LAN; Sections 4 and 5 configure the sensor slaves.

```
┌─────────────────────────┐
│  GPS Satellites          │
└───────────┬─────────────┘
            │ RF
┌───────────▼──────────────────┐
│  Point One Nav Atlas Duo      │  ← GPS-fixed solution, RTK on Polaris/NTRIP
│  - FusionEngine TCP 30201     │  ← /pose, /imu, /gps/fix, /odom
│  - NMEA 0183  TCP 30200       │  ← gpsd source (GPS time, no PPS)
│  - REST API HTTP 80           │  ← config + monitoring
└─────────────┬─────────────────┘
              │ Ethernet (single cable to RUTM50 LAN 4)
              │ Atlas at 192.168.1.30
┌─────────────▼─────────────────────────────────────┐
│  Ubuntu 24.04 Host (PTP Grandmaster, 192.168.1.5) │
│                                                    │
│  gpsd ← tcp://192.168.1.30:30200 (NMEA) → SHM     │
│  chrony ← SHM → CLOCK_REALTIME (~10–100 ms to GPS)│
│  phc2sys: CLOCK_REALTIME → NIC PHC (/dev/ptp0)    │
│  ptp4l:   NIC PHC → PTP announce on Ethernet       │
│  native Atlas driver: TCP 30201 → native ROS data   │
└────────────────────────────────────────────────────┘
```

**Expected sync accuracy at each stage** (host side — the sensor-side numbers are in Sections 4 and 5):

| Component | Accuracy | Notes |
|-----------|----------|-------|
| Atlas Duo internal GPS time | < 20 ns to UTC | Inside the device; surfaces in FusionEngine message timestamps |
| chrony (NMEA-disciplined `CLOCK_REALTIME`) | ~10–100 ms | NMEA alone, no PPS. See note below. |
| NIC PHC via `phc2sys` | tracks `CLOCK_REALTIME` ± 200 ns | Relative tracking is tight even if `CLOCK_REALTIME` itself drifts vs GPS |
| FusionEngine message `header.stamp` | < 20 ns to GPS | Built into each message by the Atlas — independent of host clock |

> **Why the host clock loosens.** The Atlas Duo's only time output is the NMEA sentence stream over TCP (the device hardware has no PPS BNC output and no USB serial output). NMEA is sentence-rate (1 Hz) with no sub-second edge, so chrony can discipline `CLOCK_REALTIME` to roughly ±10–100 ms — not the < 100 ns a hardware PPS edge would achieve. This is fine for ROS topic `header.stamp` because the recorder pulls Atlas timestamps from FusionEngine message headers (where they're still GPS-accurate at the device, independent of host-clock drift). Raw `tcpdump` packet kernel timestamps inherit the looser `CLOCK_REALTIME` accuracy — if you care about wire-level packet timing aligned to GPS sub-ms, post-correlate against FusionEngine messages in the same capture. PTP across sensors (host PHC ↔ LiDAR / camera slaves) is unaffected — relative cross-sensor sync stays sub-µs (hardware timestamping) or sub-50 µs (software).

### 3.1 Run the INS-to-PC sync script

```bash
chmod +x 3_setup_ins_to_pc_sync.sh
./3_setup_ins_to_pc_sync.sh
```

What it configures:

- **gpsd:** reads NMEA over TCP from the Atlas Duo at `tcp://192.168.1.30:30200`. Drops into shared memory for chrony. No PPS, no `/dev/ttyUSB0`, no `/dev/pps0`.
- **chrony:** disciplines `CLOCK_REALTIME` from the gpsd SHM (NMEA only). NTP pools are kept as a holdover fallback. Expected steady-state accuracy ~10–100 ms.
- **`ptp4l` grandmaster:** announces on the sensor NIC. Because the host clock is no longer disciplined to <100 ns GPS, the script defaults `clockClass` to **13** (application-specific, locked-to-internal-reference) instead of 6 (locked to primary GPS reference). Cross-sensor PTP sync remains tight; the change is just an honest advertisement to PTP slaves about the absolute-time pedigree.
- **`phc2sys`:** copies `CLOCK_REALTIME` into the NIC PHC (only relevant for hardware timestamping).
- **systemd:** enables and starts `gpsd`, `chrony`, and `ptp4l-grandmaster`; `phc2sys-grandmaster` is enabled only when the NIC supports hardware timestamping. The chain is live without a reboot.
- **Point One host tools:** `p1-host-tools/` cloned to `$HOME`, `pip install fusion-engine-client[all]`.

The script deliberately does **not** install Point One's public
`ros2-fusion-engine-driver`. At the pinned upstream revision it publishes
host-arrival-stamped `geometry_msgs/msg/PoseStamped`; Hitch's adapter requires
the deployment-provided `fusion_engine_msgs/msg/Pose` stream with device time,
solution type, and covariance. Start that native-message driver separately
before the adapter and recorder.

### 3.2 Manual verification

The script's self-test covers most of this, but here are the manual commands if you want to dig in:

```bash
# Atlas Duo reachability
ping -c 3 192.168.1.30
curl -s http://192.168.1.30/api/v1/device/status | python3 -m json.tool

# NMEA stream from Atlas Duo over TCP
nc -w 5 192.168.1.30 30200 | head -20
# Should show $GPGGA, $GPRMC sentences

# FusionEngine stream from Atlas Duo over TCP
nc -w 5 192.168.1.30 30201 | xxd | head -5
# Should show binary FusionEngine frames

# gpsd is consuming the TCP NMEA stream
gpsmon
# Should show satellites, fix type (3D), and time

# chrony — NMEA should be the chosen source
chronyc sources -v
chronyc tracking
# "Reference ID" should show NMEA, not an NTP server IP

# Setup-time PTP grandmaster diagnostic
sudo "$(command -v pmc)" -u -b 0 "GET PORT_DATA_SET"
# At least one local portState must be MASTER.

# PHC sync (hardware timestamping only)
sudo journalctl -u phc2sys-grandmaster -f
# offset values should be < 1000 ns

# Service status
sudo systemctl status gpsd chrony ptp4l-grandmaster
# Hardware timestamping only:
sudo systemctl status phc2sys-grandmaster
```

---

## Section 4: PTP Sync from PC to LiDARs

Configures one to three Seyond Robin W LiDARs as PTP slaves of the grandmaster
from Section 3 and installs the Seyond ROS 2 driver. Use `--ips` to name the
units fitted to this rig. GICP supports the front primary alone or with optional
rear units; GLIM requires all three. Assumes the **one-time network
provisioning** in [§2.7](#27-one-time-per-lidar-network-provisioning) has moved
each fitted Robin W from its factory IP to its assigned dome IP.

### 4.1 Run the LiDAR PTP-sync script

```bash
chmod +x 4_setup_lidar_ptp.sh
./4_setup_lidar_ptp.sh
```

What it does:

- Verifies the PTP grandmaster from Section 3 is running.
- Pings each LiDAR at its post-provisioning IP.
- Enables PTP on each Robin W via `innovusion_lidar_util` (standard IEEE 1588 mode, not automotive gPTP).
- Clones the Seyond ROS 2 driver and pins the reviewed commit. It removes the
  obsolete repository relative-time override if an older installation has it,
  then verifies that PointCloud2 publishes `timestamp/FLOAT64` numeric Unix
  seconds reconstructed as packet-start time plus the per-point offset.
- Builds with `./build.bash` inside `seyond_ros_driver/` (workspace-root
  `colcon build` does not work for this driver).
- Self-tests PTP slave sync status on each LiDAR.

Expected post-sync accuracy:

| Component | Accuracy (Tier 1 / RUTM50) | Accuracy (Tier 2 / Planet BC) |
|-----------|-----------------------------|-------------------------------|
| Robin W PTP slave to grandmaster | 5–50 µs | 300–800 ns |

---

## Section 5: PTP Sync from PC to Cameras

> **Optional, Tier 2 only.** If no cameras are installed, stop after Section 4:
> the recorder, GICP++, and GLIM++ are fully supported without this section.
> Installed cameras require PoE and benefit from the Planet
> WGS-6325-8UP2X boundary clock; see Section 2.2.

### 5.1 Run the camera PTP-sync script

```bash
chmod +x 5_setup_camera_ptp.sh
./5_setup_camera_ptp.sh
```

What it does:

- Verifies the PTP grandmaster from Section 3 is running.
- Installs the Aravis GigE Vision library and tools (`arv-tool-0.8`, `arv-viewer-0.8`).
- Installs ROS 2 camera packages (`image_transport`, `camera_info_manager`, etc.).
- Configures GigE Vision network settings: jumbo frames (MTU 9000) on the sensor NIC, large UDP receive buffers.
- Detects each RouteCAM on the network via Aravis discovery.
- Enables PTP on each camera through the GenICam `GevPTPMode = Slave` register.
- Self-tests PTP slave status on each camera and reports `Synchronized` / `Master` / `Listening`.

Expected post-sync accuracy through the Planet WGS-6325-8UP2X boundary clock:

| Component | Accuracy |
|-----------|----------|
| RouteCAM GigE Vision PTP slave | 1–10 µs |

---

## Section 6: Recording Data

The supported recorder is
[`../recording/sensor_recorder.py`](../recording/sensor_recorder.py). It
launches the configured ROS 2 LiDAR/camera drivers, verifies chrony plus the
actual sensor timestamps and Atlas adapter gate, records an MCAP rosbag, and
starts the Foxglove bridge. It runs unprivileged and does not call `pmc`. Its
hard runtime gate subscribes directly to every selected LiDAR/camera ROS topic
before bag creation and measures each `header.stamp` against host
`CLOCK_REALTIME`, including age stability and drift. The setup scripts still
use interactive `sudo pmc` only as an installation diagnostic.

Responsibility is intentionally one-way: **this `PTP_sync/` directory
configures and disciplines all clocks; the recorder only verifies the resulting
state.** A failed recording preflight never enables PTP, adjusts a clock, or
restarts a synchronization service. Correct the fault here, then rerun the
recorder.

For a production GLIM++ bag, start the deployment's native-message Atlas driver
first. It must publish `fusion_engine_msgs/msg/Pose` on
`/atlas/pose_filtered` and `sensor_msgs/msg/Imu` on
`/atlas/imu_calibrated`; the generic `fusion_engine_ros_driver` does not retain
the native solution-class/covariance contract required by the adapter. Then
start the adapter with exactly one deployment datum. The recorder launches
neither Atlas component:

```bash
# Terminal 1: launch the deployment's Atlas native-message ROS driver, then:
ros2 topic type /atlas/pose_filtered
ros2 topic type /atlas/imu_calibrated

# Terminal 2: normalized Atlas streams and authoritative Fixed-only odometry.
ros2 launch adapter adapter.launch.py \
  use_sim_time:=false \
  use_p1_imu_pcap:=false \
  pose_input_topic:=/atlas/pose_filtered \
  imu_input_topic:=/atlas/imu_calibrated \
  local_enu_origin:="<lat_deg>,<lon_deg>,<alt_m>"

# Terminal 3: enforce the production GLIM profile and record.
cd /path/to/Hitch_Sensor_Dome
python3 recording/sensor_recorder.py --profile glim
```

The output is `recording/data/session_<timestamp>/rosbag2/*.mcap` plus logs and
`session_metadata.json`. A GLIM-profile bag includes the three Robin W clouds,
`/gps_p1/imu`, `/gps_p1/fix`,
`/gps_p1/filtered_odom_rtk_fixed`, TF, and diagnostics when their publishers
are available. Camera streams are included only when cameras are detected and
selected. See [`../recording/README.md`](../recording/README.md) and
[`../recording/sensor_config.yaml`](../recording/sensor_config.yaml) for the
complete runtime and configuration reference.

---

## Section 7: Replay and Visualization

Open the MCAP directly in Foxglove Studio and import
`recording/foxglove/sensor_dome_layout.json`, or replay it into ROS 2:

```bash
cd /path/to/Hitch_Sensor_Dome
ros2 bag play recording/data/session_<timestamp>/rosbag2

# In another terminal:
rviz2
```

Keep `use_sim_time=false` unless the entire replay graph is deliberately
configured for `/clock`.

---

## Appendix

### A. Seyond Robin W default parameters

| Parameter | Factory default | Hitch dome (post-provisioning) | Notes |
|-----------|-----------------|--------------------------------|-------|
| IP Address | `172.168.1.10` | `192.168.1.10` / `.11` / `.12` | Factory IP per Seyond *Robin W1G User Manual* V2.2 §3.1. `provision_robin_w_multiunit.sh` renumbers each unit. |
| Data Port (UDP + TCP) | `8010` | `8337` / `8338` / `8339` | One port per unit. Rebased off Seyond's 8010/8020/8030 example to avoid Hadoop's 8020/8030 and the busy 8000-8099 dev-server range. |
| Coordinate Mode | `3` (forward/left/up) | same | Matches ROS REP-103 |
| PTP | Supported | Enabled (standard L3 IEEE 1588) | Enabled by `4_setup_lidar_ptp.sh` |
| FOV | 120° × 70° | same | |
| Points/sec | 1.28M | same | 192 scan lines |
| Range | 0.1–150 m | same | 70 m at 10% reflectivity |

### B. RouteCAM_P_CU25_CXLC_IP67 key specs

| Feature | Value |
|---------|-------|
| Sensor | AR0234 1/2.6" 2MP global shutter |
| Resolution | 1920 × 1200 |
| FOV | 158° DFOV, 134° HFOV, 73° VFOV |
| Interface | GigE Vision (M12 X-coded Ethernet) |
| Power | PoE (IEEE 802.3af) |
| Time Sync | IEEE 1588 PTP via GigE Vision |
| Protection | IP67 |
| Dome layout | Front stereo pair (110 mm baseline) + rear symmetric pair |

### C. NVIDIA GPU and RT kernel compatibility

The NVIDIA kernel module (`nvidia.ko`) cannot load on the PREEMPT_RT kernel. No CUDA, cuDNN, TensorRT, or GPU compute is available on the RT kernel.

| Kernel | CUDA Available | Best For |
|--------|---------------|----------|
| **Generic** | Yes | Training, simulation, GPU inference |
| **RT** | **No** | Deterministic sensor recording, real-time control |

**Recommended workflow:** record on the RT kernel (deterministic PTP timing, no GPU needed), then reboot into the generic kernel for post-processing with full CUDA access.

```
Field recording:   RT kernel → tcpdump + p1_runner (no GPU needed)
Post-processing:   Generic kernel → CUDA + PyTorch/TensorRT on recorded data
```

To select the kernel at boot, choose **Advanced options for Ubuntu** in the GRUB menu, then pick the `*-realtime` or `*-generic` entry.

### D. Troubleshooting

**PTP not syncing:** Verify the interface name in the `ptp4l` config, then run
`sudo "$(command -v pmc)" -u -b 0 "GET PORT_DATA_SET"` and require a local
`MASTER` state.
Confirm PTP is enabled on each Robin W and check the cable path.

**chronyc shows NTP as primary (not NMEA):** gpsd may not be reaching the Atlas Duo over TCP. Verify `ping 192.168.1.30` works, and that the Atlas Duo is set to "Start Navigating" in its web UI (without the navigation engine running, no NMEA is emitted). Run `sudo systemctl status gpsd` and `gpsmon`. To raw-test the NMEA stream independently of gpsd: `nc -w 5 192.168.1.30 30200 | head`.

**LiDAR packet drops:** Likely network bandwidth saturation. Check with `sudo ethtool -S eth0 | grep -i drop`. For 3 Robin W LiDARs + 4 cameras, a 10 GbE NIC is required.

**No GigE Vision cameras detected:** Check PoE switch power, verify cameras are on the same subnet, and try `arv-tool-0.8` to scan. Firewall rules may block GigE Vision discovery packets.

**Timestamps misaligned in rosbag:** Verify `use_sim_time` is `false` in all ROS 2 nodes. Check `chronyc tracking` to confirm GPS discipline is active.

**gpsd shows "NO FIX":** Ensure the Atlas Duo antenna has clear sky view. Cold start may take up to 30 minutes. Open the Atlas Duo web UI at `http://192.168.1.30` and confirm the navigation engine is running (Start button on the Map View).

**`<cstdint>` build errors:** GCC 14 on Ubuntu 24.04 is stricter. The `3_setup_ins_to_pc_sync.sh` and `4_setup_lidar_ptp.sh` scripts patch this automatically.

**Seyond build fails:** Use `./build.bash` inside `seyond_ros_driver/` (not `colcon build` from the workspace root). The script `4_setup_lidar_ptp.sh` handles this.
