# Robin W FW 2835 — Multi-Unit Provisioning Files

This folder holds the per-LiDAR `PCS_ENV` files Seyond ships for running three Robin W units on the same Ethernet segment. Each file assigns the receiving host's IP and a unit-specific UDP destination port so the three point-cloud streams don't collide when they arrive at the host PC.

> **Seyond factory default → must be reset.** Per Seyond's official [*Robin W1G LiDAR User Manual* V2.2 (2025-01-03)](https://www.seyond.com/wp-content/uploads/2025/03/Seyond-Robin-W1G-LiDAR_User-Manual_V2.2_EN_Public_20250103.pdf) §3.1, every Robin W ships with the default IP **`172.168.1.10`** (netmask `255.255.255.0`, gateway `172.168.1.1`). The Hitch Sensor Dome network runs on `192.168.1.0/24`, so **each Seyond LiDAR must be reset to the `192.168.1.x` subnet** — `192.168.1.10` / `.11` / `.12` per the mapping table below. This reset is the first step the provisioning script performs.

Upload one file to each LiDAR with [`provision_robin_w_multiunit.sh`](../provision_robin_w_multiunit.sh) (one-time per LiDAR). The script picks the right file based on the serial→position mapping below.

## Files in this folder

| File | Serial number | Target position | Target IP | UDP/TCP port (data + control) |
|---|---|---|---|---|
| `RW_FW2835_Allen_533192400101_unicast.env` | `533192400101` | `robin_w_front` | `192.168.1.10` | 8337 |
| `RW_FW2835_Allen_533262400110_unicast.env` | `533262400110` | `robin_w_rear_left` | `192.168.1.11` | 8338 |
| `RW_FW2835_Allen_533192400103_unicast.env` | `533192400103` | `robin_w_rear_right` | `192.168.1.12` | 8339 |

Position assignments preserve the natural ordering of Seyond's original 8010/8020/8030 triplet — we just rebased the range to 8337-8339 to dodge common conflicts (see "Port choice" below). The IP last octet matches the position pattern set in [`../../config/network_config.yaml`](../../config/network_config.yaml) (`.10` / `.11` / `.12`).

## Port choice — why 8337/8338/8339

Seyond's stock multi-unit example uses 8010/8020/8030. The Hitch dome rebases to **8337/8338/8339** to avoid two real conflict surfaces:

- **Apache Hadoop:** `8020` is the default HDFS NameNode IPC port, `8030` is the default YARN ResourceManager scheduler port. Any host that ever ran Cloudera / Hortonworks / vanilla Hadoop would have those ports squatted.
- **8000–8099 is dev-server territory:** Python `http.server` defaults to 8000, Django to 8000, Jupyter to 8888, Cypress to 8080, and so on. Even if those are TCP-only, the Seyond driver opens TCP + UDP on the same port (`TCP_SERVICE_PORT` and `UDP_PORT_*` are equal inside each PCS_ENV), so a TCP squatter blocks the LiDAR handshake.

`8337` and `8338` are IANA-registered to Konica Minolta PowerJet (printer-management services — vanishingly unlikely on a perception PC). `8339` is unassigned in IANA's registry. The triplet sits in a quiet stretch of the registered-port range and is contiguous, which keeps firewall and switch QoS rules simple.

If you change the ports, three files have to move together:
1. The three `RW_FW2835_Allen_*_unicast.env` files in this folder.
2. The `UNIT_TABLE` in [`../provision_robin_w_multiunit.sh`](../provision_robin_w_multiunit.sh).
3. The `lidars[*].port` entries in [`../../recording/sensor_config.yaml`](../../recording/sensor_config.yaml).

The provisioning script's `--help` output and the PCS_ENV files are the canonical source — keep the sensor_config.yaml in sync.

The receiving host's IP is set to **`192.168.1.5`** in every file — that is the project host PC's static IP per [`../../config/network_config.yaml`](../../config/network_config.yaml). Seyond's reference example used `172.168.1.100`; we replaced it because the dome network runs on `192.168.1.0/24`.

## Note on the Seyond factory default IP

A brand-new Robin W1G ships with **`172.168.1.10`** as its factory IP, per Seyond's official [*Robin W1G LiDAR User Manual* V2.2 (2025-01-03)](https://www.seyond.com/wp-content/uploads/2025/03/Seyond-Robin-W1G-LiDAR_User-Manual_V2.2_EN_Public_20250103.pdf) §3.1:

> *"The initial IP address of the LiDAR is 172.168.1.10. The initial subnet mask is 255.255.255.0. The initial gateway is 172.168.1.1."*

That subnet is publicly-routable IPv4 space (it is **not** inside RFC 1918's `172.16.0.0/12` private block, which ends at `172.31.x.x` — `172.168.x.x` is an entirely different range). For our purposes this is fine because the dome NIC is air-gapped from the public internet, but it does mean the host PC cannot ping a factory-state LiDAR from its normal `192.168.1.5` address — the two subnets don't share an L3 path.

`provision_robin_w_multiunit.sh` handles this automatically by adding a **temporary** `172.168.1.100/24` IP alias to the sensor NIC at the start of provisioning, then removing it on exit. The alias coexists with the regular `192.168.1.5/24` address (both work simultaneously at L2), so the post-provisioning ping at the new `192.168.1.X` address also succeeds inside the same script run. If you ever need to reach a factory LiDAR by hand:

```bash
# Add (run once at the start of a manual provisioning session):
sudo ip addr add 172.168.1.100/24 dev <your-sensor-nic>
# Verify reachability:
ping 172.168.1.10
# Remove when finished:
sudo ip addr del 172.168.1.100/24 dev <your-sensor-nic>
```

## Verifying the serial→position mapping on a new dome

Before the first multi-unit provisioning run, confirm that the serials in the table above match the physical units you actually have. Read the serial off each LiDAR's web UI (`http://172.168.1.10` while still at the factory IP — you'll need the temporary IP alias described below to reach it — and `http://192.168.1.X` after each unit is renumbered):

```bash
# At factory state (with the 172.168.1.100/24 alias up):
curl -s http://172.168.1.10/api/v1/static_info | python3 -m json.tool | grep -i serial

# After provisioning (host's normal 192.168.1.5 address suffices):
curl -s http://192.168.1.10/api/v1/static_info | python3 -m json.tool | grep -i serial
```

If a serial does not appear in the table above, the provisioning script will refuse to upload and ask you to update this README. That is intentional — a wrong `PCS_ENV` upload would silently route point clouds to the wrong destination port and the recorder would never know which LiDAR is which.

## File schema

```
TCP_SERVICE_PORT=833X       # TCP control port (matches UDP_PORT_*)
UDP_IP=192.168.1.5         # host PC destination
UDP_PORT_DATA=833X          # point cloud frames
UDP_PORT_MESSAGE=833X       # status messages
UDP_PORT_STATUS=833X        # health status
STATUS_INTERVAL_MS=50       # 20 Hz status reports
LOG_OPTION="..."            # /tmp/inno_pc_server.txt rotation rules
MIN_RUN_TIME=5              # min seconds before restart on failure
MIN_RUN_TIME_SLEEP=5        # sleep before restart
MAX_PACKET_SIZE=1450        # UDP MTU (Ethernet jumbo NOT required)
```

`833X` is `8337`, `8338`, or `8339` depending on which file. The same number is used for TCP control and all three UDP ports inside one file because the recipient demultiplexes on the *port*, not the *purpose* — different LiDARs send to different ports, while a single LiDAR may reuse the same port for data/message/status streams.

## Note on the `unicast` suffix

The filenames end in `_unicast.env` because each LiDAR is being told to send only to one IP (the host PC). The alternative — multicast — uses one shared destination but requires switch-side IGMP snooping to keep the traffic out of unrelated ports. Unicast is the safe default for a small fleet on a managed PoE switch (Planet WGS-6325-8UP2X per the network config). Switch to multicast only if you intend to run more than one consumer of the same LiDAR's stream.
