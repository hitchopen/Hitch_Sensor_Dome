# Hitch Sensor Dome

A 3D-printable modular sensor dome for mounting a multi-sensor mapping rig on a vehicle roof via a suction cup camera mount.

## Sensors

- 3× Seyond Robin W LiDAR — 120° HFOV each, arranged at 120° intervals for full 360° surround coverage
- 1× Point-One Nav Atlas Duo INS — center-mounted, CoN at geometric origin
- 1× Survey-grade GNSS antenna — on commercial magnetic stand, centered above CoN
- 4× e-con RouteCAM_P_CU25_CXLC_IP67 cameras — front stereo pair (104 mm baseline) + rear symmetric pair

## Sensor Layout

The diagrams below (generated from the v17c SCAD model) label every sensor position relative to the Atlas Duo Center of Navigation (origin) in the ROS REP 103 vehicle frame (+X forward, +Y left, +Z up).

![Top-down sensor layout](3D%20files/sensor_dome_layout_top.jpg)

*Top-down view. LiDARs 1–3 face outward at 0° / 120° / 240°; cameras 1–2 form the 104 mm front stereo baseline flanking LiDAR 1; cameras 3–4 sit on the rear-left and rear-right hex faces.*

![Isometric sensor layout](3D%20files/sensor_dome_layout_iso.jpg)

*Isometric view showing the two-level dome: LiDARs hang from the underside of L2, cameras sit on top of L2, and the GNSS antenna rises above the plate center on its magnetic stand.*

## Repository Structure

```
3D files/           OpenSCAD model, READMEs, and exported STL files
  sensor_dome.scad   Parametric OpenSCAD source (v17c)
  README.md          Detailed design specifications (English)
  README_zh.md       Detailed design specifications (Chinese)
  *.stl              Exported print-ready meshes (L1, L2)

Documents/           Component datasheets
  Pointonenav-assembly-atlas-duo.pdf
  Seyond-Robin-W1G-Manual.pdf
  e-con_RouteCAM_CU25_IP67_Datasheet.pdf
  e-con_RouteCAM_CU25_IP67_Lens_Datasheet.pdf
  Datasheet_Magnetic_Stand_for_Survey_GNSS_Antenna.pdf

ROS2 config/         ROS2 integration files
  sensor_dome_tf.yaml  Static tf transforms (all sensors → imu_link)
```

## Quick Start

1. Install [OpenSCAD](https://openscad.org/)
2. Open `3D files/sensor_dome.scad`
3. Set `RENDER_MODE = 1` for Level 1, `RENDER_MODE = 2` for Level 2
4. Render (F6) and export STL (F7)
5. Print both parts on a 305 × 305 mm bed (PETG or ABS, 50–60% infill)

See [`3D files/README.md`](3D%20files/README.md) for full design specifications, BOM, and assembly instructions.

## Coordinate System (ROS REP 103)

- **+X** = Forward, **+Y** = Left, **+Z** = Up
- **Origin** = Atlas Duo Center of Navigation (CoN)

## Credits

This project was designed and is maintained by **Dr. Allen Y. Yang** (University of California, Berkeley).

Please cite or credit this repository when reusing any of the mechanical design, the ROS 2 TF configuration, or the PTP synchronization pipeline in derivative work:

> Yang, A. Y. *Hitch Sensor Dome: a 3D-printable modular multi-sensor mount for vehicle-roof mapping.* GitHub repository, 2026.

Thanks to the OpenSCAD, ROS 2, linuxptp, chrony, and Aravis communities whose open-source tooling this project builds on.

## License

See [LICENSE](LICENSE) for details.
