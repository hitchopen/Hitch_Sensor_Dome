# Robin W FW 2835 — Multi-Unit Provisioning Files

This folder holds the per-LiDAR `PCS_ENV` files Seyond ships for running three Robin W units on the same Ethernet segment. Each file assigns the receiving host's IP and a unit-specific UDP destination port so the three point-cloud streams don't collide when they arrive at the host PC.

Upload one file to each LiDAR with [`provision_robin_w_multiunit.sh`](../provision_robin_w_multiunit.sh) (one-time per LiDAR). The script picks the right file based on the serial→position mapping below.

## Files in this folder

| File | Serial number | Target position | Target IP | UDP ports (data/msg/status) |
|---|---|---|---|---|
| `RW_FW2835_Allen_533192400101_unicast.env` | `533192400101` | `robin_w_front` | `192.168.1.10` | 8010 |
| `RW_FW2835_Allen_533262400110_unicast.env` | `533262400110` | `robin_w_rear_left` | `192.168.1.11` | 8020 |
| `RW_FW2835_Allen_533192400103_unicast.env` | `533192400103` | `robin_w_rear_right` | `192.168.1.12` | 8030 |

Position assignments follow the natural ordering of the UDP ports Seyond pre-assigned in the files (8010 / 8020 / 8030 → front / rear-left / rear-right). The IP last octet matches the position pattern set in [`../../config/network_config.yaml`](../../config/network_config.yaml) (`.10` / `.11` / `.12`).

The receiving host's IP is set to **`192.168.1.40`** in every file — that is the project host PC's static IP per [`../../config/network_config.yaml`](../../config/network_config.yaml). Seyond's reference example used `172.168.1.100`; we replaced it because (a) the dome network uses `192.168.1.0/24`, and (b) `172.168.x.x` is publicly-routable address space, not an RFC 1918 private block.

## Verifying the serial→position mapping on a new dome

Before the first multi-unit provisioning run, confirm that the serials in the table above match the physical units you actually have. Read the serial off each LiDAR's web UI (`http://192.168.1.10` while still at the factory IP, then again after each one is renumbered):

```bash
curl -s http://192.168.1.10/api/v1/static_info | python3 -m json.tool | grep -i serial
```

If a serial does not appear in the table above, the provisioning script will refuse to upload and ask you to update this README. That is intentional — a wrong `PCS_ENV` upload would silently route point clouds to the wrong destination port and the recorder would never know which LiDAR is which.

## File schema

```
TCP_SERVICE_PORT=80X0       # TCP control port (matches UDP_PORT_*)
UDP_IP=192.168.1.40         # host PC destination
UDP_PORT_DATA=80X0          # point cloud frames
UDP_PORT_MESSAGE=80X0       # status messages
UDP_PORT_STATUS=80X0        # health status
STATUS_INTERVAL_MS=50       # 20 Hz status reports
LOG_OPTION="..."            # /tmp/inno_pc_server.txt rotation rules
MIN_RUN_TIME=5              # min seconds before restart on failure
MIN_RUN_TIME_SLEEP=5        # sleep before restart
MAX_PACKET_SIZE=1450        # UDP MTU (Ethernet jumbo NOT required)
```

`80X0` is `8010`, `8020`, or `8030` depending on which file. The same number is used for TCP control and all three UDP ports inside one file because the recipient demultiplexes on the *port*, not the *purpose* — different LiDARs send to different ports, while a single LiDAR may reuse the same port for data/message/status streams.

## Note on the `unicast` suffix

The filenames end in `_unicast.env` because each LiDAR is being told to send only to one IP (the host PC). The alternative — multicast — uses one shared destination but requires switch-side IGMP snooping to keep the traffic out of unrelated ports. Unicast is the safe default for a small fleet on a managed PoE switch (Planet WGS-6325-8UP2X per the network config). Switch to multicast only if you intend to run more than one consumer of the same LiDAR's stream.
