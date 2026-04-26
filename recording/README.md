# Recording — ROS 2 MCAP Capture + Foxglove Live Dashboard

A single Python orchestrator that activates every connected sensor on the dome, verifies the GPS → chrony → PTP clock chain, records GNSS / IMU / LiDAR / camera data into a Foxglove-native MCAP rosbag, and serves a live dashboard so you can watch the data being captured in real time.

This folder is the run-time companion to the one-time setup scripts in [`../PTP_sync/`](../PTP_sync/) and the static-TF definitions in [`../ROS2 config/sensor_dome_tf.yaml`](../ROS2%20config/sensor_dome_tf.yaml). Use `PTP_sync/` once at install time to bring up the synchronization plumbing; use this folder for every recording session afterward.

## Architecture

```
                       sensor_recorder.py
                              │
       ┌──────────────────────┼──────────────────────┐
       ▼                      ▼                      ▼
   detection             sync verify             session output
  ─────────────       ─────────────────        ───────────────
   probe sensors      chrony + ptp4l +          recording/data/
   show checklist     pmc, warn + prompt        session_<ts>/
       │                      │                      ▲
       └──── confirm ────────┘                       │
                              │                      │
                              ▼                      │
                   ┌──────── spawned ─────────┐      │
                   │  static_transform        │      │
                   │  fusion_engine_driver    │      │
                   │  seyond × N (one each)   ├──────┤
                   │  camera_aravis2 × M      │      │
                   │  rate_monitor.py         │      │
                   │  ros2 bag record -s mcap │──────┘
                   │  foxglove_bridge :8765   │
                   └──────────┬───────────────┘
                              │ ws://localhost:8765
                              ▼
                       Foxglove Studio
                  (live dashboard + replay)
```

Each driver is a separate process so a single crashed driver does not kill the bag recorder. All sensors share the GPS-disciplined PTP clock, so the MCAP timestamps line up without post-hoc correction.

## Running It

```bash
# Default: detect → confirm → verify sync → record to recording/data/
sudo python3 recording/sensor_recorder.py

# Headless: skip prompts, log sync warnings instead of blocking
sudo python3 recording/sensor_recorder.py --headless --yes

# Detect + sync-check only, no recording
python3 recording/sensor_recorder.py --dry-run
```

What happens on a normal run:

1. **Detect.** The script probes the Atlas Duo, Robin W LiDARs, and RouteCAM cameras listed in `sensor_config.yaml`, then prints a checklist tagged `[FOUND]` or `[MISSING]`. Press `ENTER` to record from everything found, or type space-separated indices to toggle items off.

2. **Verify sync.** The chrony reference, `ptp4l` master offset, and PTP slave count are checked against the tolerances in `sensor_config.yaml`. If anything is out of spec, you get a `Proceed anyway? [y/N]` prompt. (Switch to `--sync-mode hard` to refuse instead, or `--sync-mode log` to record-and-continue.)

3. **Record.** Drivers, bag recorder, rate monitor, and `foxglove_bridge` start up. The console shows their PIDs. Press `H` for health, `Q` (or Ctrl+C) to stop and flush.

4. **Tear down.** The bag is flushed first, then drivers exit. `session_metadata.json` is written with the sensors, sync results, and topic list.

## Foxglove

While the recorder is running:

1. Open Foxglove Studio.
2. **Open Connection → Foxglove WebSocket → `ws://localhost:8765`**.
3. **Layouts → Import from file → `recording/foxglove/sensor_dome_layout.json`**.

The layout shows the three Robin W point clouds superimposed in `imu_link` (resolved through `/tf_static` from `../ROS2 config/sensor_dome_tf.yaml`), the four camera views, a GNSS map that follows `/gps/fix`, an IMU plot, a fix-status indicator, and a Diagnostics panel bound to `/sensor_dome/rates` that lights up yellow / red when a sensor falls below its expected Hz.

For replay after the fact, open the recorded `.mcap` directly in Foxglove (`File → Open local file`) and apply the same layout — no bridge or recorder needed.

## Output

Sessions land in `recording/data/` by default (configurable via `--output` or `recording.output_dir` in the YAML):

```
recording/data/session_20260425_103022/
├── rosbag2/
│   └── rosbag2_0.mcap        # all sensors + /tf + /tf_static + /sensor_dome/rates
├── logs/                     # per-driver stdout/stderr
├── rosbag2.log
├── foxglove_bridge.log
└── session_metadata.json     # sensors, sync results, topics, foxglove URL
```

Sessions are gitignored; the `data/` folder itself is tracked so the path stays stable.

## See Also

- [`../PTP_sync/README.md`](../PTP_sync/README.md) — one-time install of `gpsd` + `chrony` + `ptp4l` + `phc2sys` and per-sensor PTP enablement
- [`../ROS2 config/sensor_dome_tf.yaml`](../ROS2%20config/sensor_dome_tf.yaml) — source of truth for every `imu_link → sensor_link` static transform
- [`../README.md`](../README.md) — top-level project description, sensor layout, and BOM
