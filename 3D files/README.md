# 3D Mapping Sensor Dome Base

## Overview
A 3D-printable modular base for mounting 3× Seyond Robin W LiDARs, 1× Point-One Nav Atlas Duo INS, and 1× survey-grade GNSS antenna on a vehicle roof via a suction cup camera mount.

**Current source: v17r.** It retains the two-piece/unibody variants and the
inside-corner fillets, expands the V0/V1 and V4/V5 supports into 18 mm ×
140 mm side walls, extends the two Atlas-side supports to 55 mm, uses
10× M6×25 mm bracket bolts, and adds front-access chamfers. Camera positions,
the 80 mm LiDAR ring, and the cable opening below are the current SCAD values.

**v17b** updates the bracket design and L2 thickness. The **6 single-wall brackets** are now split between **4 hex-vertex brackets** (at V0, V1, V4, V5 along blank sides 2 and 6) and **2 new Atlas-side brackets** (flat walls running +X alongside the Atlas INS body at y = ±64 mm, x = −150 to −115 mm, 10 mm thick, 35 mm long). L2 now has the same **hex + rear flange shape as L1**. L2 thickness increases from 6 mm to **12 mm** for a deeper GNSS recess (8 mm depth, 4 mm floor). Hex-vertex brackets remain 12 mm thick, 35 mm long. Atlas-side brackets clear the Atlas body by 3–7 mm and LiDAR bodies by ≥10 mm.

**v15** eliminates the printed GNSS mast entirely. A commercial magnetic GNSS antenna stand (e.g. ArduSimple AS-ACC-SURVEYSTAND-01, with 5/8"-11 threaded pole) is glued into a circular recess on L2's top center. This reduces the assembly to just 2 printed parts (L1 + L2).

**v14** unified all inter-level bolts to M6, eliminating M5 entirely. Cable holes consolidated to 1× ø30 mm rear centerline.

**v13** simplified the design from 4 printed parts down to 3 by eliminating Level 3. LiDARs mount to the underside of L2 using the top mount bolt pattern.

## Design Specifications

### Coordinate System (ROS REP 103)
- **+X** = Forward (front of vehicle)
- **+Y** = Left
- **+Z** = Up
- **Origin** = Atlas Duo Center of Navigation (CoN), at the geometric center of the base

### Components Mounted
| Component | Qty | Mount Type |
|-----------|-----|------------|
| Seyond Robin W LiDAR | 3 | 4× M6 top-mount trapezoid (72×68×58 mm), from above through L2 |
| Point-One Nav Atlas Duo | 1 | 4× M4 rectangle (220×100 mm) |
| Survey-grade GNSS Antenna | 1 | Commercial magnetic stand glued into L2 recess |
| e-con RouteCAM_P_CU25_CXLC_IP67 | 4 | 4× M3 bottom-mount square (16.5 mm), on L2 top — front stereo pair + rear symmetric pair |
| Camera suction cup | 1 | 1/4"-20 UNC threaded hole (bottom center) |

### Hexagonal Plate Geometry (v17)
- **Regular hexagon**, flat side 1 facing +X (outward normal at 0°).
- Inradius (center to flat side): 130 mm.
- Circumradius (center to vertex): 150.1 mm.
- Vertex-to-vertex: 300.2 mm (fits 305 mm printer bed).
- Side numbering (outward normal angle): Side 1 (0°, LiDAR), Side 2 (60°, blank), Side 3 (120°, LiDAR), Side 4 (180°, Atlas axis), Side 5 (240°, LiDAR), Side 6 (300°, blank).
- Vertex numbering: V0 (30°), V1 (90°), V2 (150°), V3 (210°), V4 (270°), V5 (330°).

### LiDAR Arrangement
- 3× Robin W at **120° intervals** on hex sides 1, 3, 5 for full 360° surround coverage
- Mounted on the **underside of Level 2** using the LiDAR's top mount bolt pattern (M6×1.0 ▽5 threaded blind holes)
- M6×16 mm SHCS from L2 top surface down into LiDAR top mount holes (countersunk)
- Facing radially outward from center, hanging below L2
- Ring radius: 80 mm

### Atlas Duo Placement
- Mounted on Level 1 (base plate)
- CoN aligned to base geometric center
- Bolt pattern offset for asymmetric CoN position:
  - CoN is 68.7 mm from front datum, 47.8 mm from left datum
- L1 has a rectangular flange at hex side 4 extending to x = −150 mm to cover the rear bolt pair (at x = −141.3 mm)

### GNSS Antenna Mount (commercial stand, v15)
- A commercial magnetic GNSS antenna stand (e.g. ArduSimple AS-ACC-SURVEYSTAND-01) with a 5/8"-11 threaded metal pole is glued into a ø86 mm × 8 mm circular recess on L2's top center (leaves a 4 mm floor in the 12 mm plate).
- No printed thread, no bolts — the stand is bonded with epoxy or construction adhesive.
- The metal pole and 5/8"-11 thread are far stronger than printed plastic.
- **Measure your stand's base diameter** and adjust `gnss_stand_base_dia` in the SCAD file if it differs from the 88 mm default.

### RGB Camera Placement (RouteCAM_P_CU25_CXLC_IP67)
- **4 cameras** mounted on **L2 top surface** in two groups:
  - **Front stereo pair** at 0° (alongside LiDAR@0°): one camera at +Y offset, one at −Y offset. Stereo baseline = **110 mm**.
  - **Rear symmetric pair**: one camera at 120° (+Y), one at 240° (−Y). Mirrored about the global X axis at approximately (−108.1, ±47.3) mm.
- Each camera faces **radially outward**, parallel to its LiDAR's scanning direction for maximum FOV overlap.
- Front camera centers are **(75, ±55) mm**. Rear camera centers use
  **(95, ±70) mm** in their respective LiDAR-local frames.
- Bottom mount: **4× M3×0.5** threaded holes, **16.5 mm square** spacing, 5 mm thread depth.
- Camera body: **46 × 46 × 65.12 mm** (with lens). 2MP global shutter, IP67, PoE, M12 X-coded Ethernet.
- FOV: **158° DFOV**, 134° HFOV, 73° VFOV — overlaps the LiDAR's 120° HFOV on the same side.
- Key parameters: `rgb_cam_x_front`, `rgb_cam_y_front`,
  `rgb_cam_x_rear`, `rgb_cam_y_rear`, and `rgb_cam_layout`.

### Cable Opening (L2 side 4)
- Replaces the original ø30 mm cable hole. Carries **5 thick shielded cables** (4× PoE camera + 1× GNSS) from L2 down to L1.
- Shape: a closed teardrop from a **ø30 mm tip circle** at (−100, 0)
  to a 100 mm-wide base rectangle at x = [−140, −135], with a 10 mm rear wall.
- Tip-to-base-inner distance is **35 mm**; the straight tangent run is 30 mm
  after the tip radius. It leaves 14 mm to each Atlas-side support and at
  least 3.28 mm of plate around the nearest rear-camera M3 hole.
- **L2 only** — L1 plate remains solid under the brackets for structural support.
- Key parameters: `cable_opening_tip_x`, `cable_opening_tip_r`, `cable_opening_base_half_w`, `cable_opening_base_inner_x`.

### Supports (v17r)

**Hex-vertex brackets (4 total)**
- Located at **V0, V1, V4, V5** along blank **sides 2 and 6**.
- Each support forms part of an 18 mm-thick × 140 mm full side wall.
- Wall height is 133 mm above the 6 mm L1 plate; L2 begins at z = 139 mm.
- Symmetric 15 mm base/top fillets distribute load into both plates.
- 2 bolts per bracket, 8 total M6 BHCS.

**Atlas-side brackets (2 total)**
- Two flat-wall brackets running **+X** alongside the Atlas Duo INS body.
- Positioned at **y = +64 mm** and **y = −64 mm**, extending from **x = −150 mm to x = −95 mm** (55 mm long).
- Nominal thickness is 15 mm, clipped by the flange to approximately 11 mm.
- Clear the Atlas body by 3–7 mm and LiDAR bodies by ≥10 mm.
- **Inside-corner fillet** on 2 sides: +X face (10 mm) and Atlas-facing face (5 mm upper / 2 mm lower, preserving Atlas clearance).
- 1 centered bolt per support, 2 total M6 BHCS.

**Total: 6 supports, 10 M6×25 mm BHCS**

## Printed Parts

| Part | Qty | `RENDER_MODE` | Description |
|------|-----|---------------|-------------|
| Level 1 (Base) | 1 | `1` | Hex plate + Atlas rear flange, camera mount, 4 hex-vertex brackets + 2 Atlas-side brackets |
| Level 2 (Combined) | 1 | `2` | Hex + rear flange (same shape as L1) — LiDARs below, GNSS stand recess + cable hole + 4× RouteCAM mounts above |

All centers (GNSS antenna → CoN → camera mount) coaxial along Z at X = 0, Y = 0.

## Printer Compatibility
- **Printer:** Raise3D Pro2
- **Build volume:** 305 × 305 × 300 mm
- **L1 bounding box:** ~280 × 300 × 139 mm ✓
- **L2 bounding box:** ~280 × 300 × 12 mm ✓

## Recommended Print Settings
| Parameter | L1 / L2 |
|-----------|---------|
| Material | PETG or ABS (outdoor use) |
| Layer height | 0.2 mm |
| Wall perimeters | 3–4 |
| Top/bottom layers | 4–5 |
| **Infill** | **50–60%** |
| **Infill pattern** | **gyroid or cubic** |
| Supports | L1: none (flat wall extrusions, zero overhangs); L2: none |
| Bed adhesion | Brim recommended for L1 |

> Both plates are vehicle-roof-mounted and subject to wind load,
> vibration, and bolt clamping forces. Do not use infill below 50%.
> Use a 3D infill pattern (gyroid, cubic, or grid) rather than 2D
> (lines, rectilinear) for better multi-axis load resistance.

## Hardware BOM (non-printed)

### Complete Screws Shopping List
| Size | Qty | Used for |
|------|-----|----------|
| **M3 × 16 mm SHCS** | **16** | RGB cameras (4 per camera × 4 cameras, through L2 into camera bottom threads) |
| **M4 × 10 mm SHCS** | **4** | Atlas Duo INS → L1 plate |
| **M6 × 25 mm BHCS** | **10** | Supports through L2 into L1 |
| **M6 × 16 mm SHCS** | **12** | Robin W LiDARs (4 per LiDAR × 3 LiDARs, countersunk through L2) |
| **1/4"-20 UNC threaded insert** (heat-set or press-fit) | **1** | Camera suction cup mount |

> Three bolt sizes: M3 (cameras), M4 (Atlas), and M6 (everything else). 16× M3, 4× M4, 22× M6.
> M3 bolts pass through the 12 mm L2 from below and thread into the camera's bottom M3×0.5 holes (5 mm thread depth). M3×16 gives 4 mm thread engagement with ~1 mm bottom clearance.
> All M6 bolts self-tap into printed features — no hex nuts or inserts.
> **M6×25 BHCS:** clearance-only through 12 mm L2, 2 mm pilot clearance, and 11 mm engagement in 15 mm-deep L1 tap holes.
> **M6×16 SHCS:** through 12 mm L2 pass-through, 4 mm thread engagement in the LiDAR top-mount blind holes (▽5, per datasheet). Engagement is marginal for M6×1.0 (ISO recommends ≥ 1×D = 6 mm). If an M6×18 fits without bottoming in the 5 mm blind hole on your specific unit, prefer that; otherwise budget for thread-locker and lower clamp load.

### Detailed Fastener Breakdown
| Item | Qty | Location |
|------|-----|----------|
| M3 × 16 mm SHCS | 16 | RGB cameras (4 per camera × 4 cameras, through L2 from below) |
| M4 × 10 mm SHCS | 4 | Atlas Duo mounting (220 × 100 mm pattern on L1) |
| M6 × 25 mm BHCS | 10 | L1 ↔ L2: full-side supports (8) + Atlas-side supports (2) |
| M6 × 16 mm SHCS | 12 | Robin W top mount (4 per LiDAR × 3 LiDARs, through L2) |
| 1/4"-20 UNC insert | 1 | Camera suction cup (bottom of L1, center) |

### Other
| Item | Qty | Purpose |
|------|-----|---------|
| TELESIN triple suction cup mount | 1 | Vehicle roof attachment |
| e-con RouteCAM_P_CU25_CXLC_IP67 | 4 | RGB cameras (158° DFOV, IP67, PoE, 2MP) |
| Survey-grade GNSS antenna | 1 | 5/8"-11 thread mount |
| Commercial magnetic GNSS stand | 1 | e.g. ArduSimple AS-ACC-SURVEYSTAND-01 |
| Epoxy or construction adhesive | — | Bond stand base into L2 recess |

## How to Export STL Files

1. Install [OpenSCAD](https://openscad.org/)
2. Open `sensor_dome.scad`
3. Set `RENDER_MODE` at the top of the file:
   - `RENDER_MODE = 0;` → Full assembly preview (F5, do NOT export)
   - `RENDER_MODE = 1;` → Level 1 (F6 render, F7 export as `sensor_dome_level_1.stl`)
   - `RENDER_MODE = 2;` → Level 2 (F6 render, F7 export as `sensor_dome_level_2.stl`)
4. Print one of each (2 parts total).

## Assembly Order
1. Print both parts (1× Level 1, 1× Level 2)
2. Install the 1/4"-20 threaded insert into L1 bottom center
3. Place L2 onto L1's supports and secure with 10× M6×25 mm BHCS (self-tapping — run screws in slowly the first time)
4. Mount Atlas Duo to L1 top surface using 4× M4 bolts
5. Attach Robin W LiDARs to L2 underside using 12× M6×16 mm SHCS from L2 top surface (4 per LiDAR, countersunk, threading down into LiDAR top mount holes)
6. Mount 4× RouteCAM cameras on L2 top surface using 16× M3×16 mm SHCS from L2 underside (4 per camera). Front stereo pair flanks LiDAR@0° on both sides; rear pair at 120°/240° is symmetric about the X axis.
7. Glue the commercial GNSS magnetic stand base into the L2 center recess using epoxy or construction adhesive. Ensure the stand is centered and level; allow adhesive to cure fully before use.
8. Thread the GNSS antenna onto the stand's 5/8"-11 metal stud
9. Attach assembly to TELESIN suction cup via 1/4"-20

## Customization
All dimensions are parametric in `sensor_dome.scad`. Key parameters:
- `hex_inradius` — hexagon center-to-flat-side distance (default 130 mm)
- `lidar_ring_radius` — how far LiDARs sit from center (default 80 mm)
- `gnss_stand_base_dia` — **measure your GNSS stand** and set this value (default 88 mm)
- `gnss_stand_recess_depth` — recess depth (default 8 mm, leaves 4 mm floor)
- `bracket_wall_t` / `bracket_leg_len` — full-side support dimensions (18 mm × 140 mm)
- `bracket_fillet_r` — fillet radius at support-to-plate junction (default 15 mm)
- `atlas_fillet_inner_upper` / `atlas_fillet_inner_lower` — Atlas-facing fillet radius (default 5 / 2 mm)
- `atlas_bracket_wall_t` / `atlas_bracket_leg_len` — Atlas-side support dimensions (nominal 15 mm × 55 mm)
- `atlas_bracket_y_upper` / `atlas_bracket_y_lower` — Atlas-side bracket Y positions (default ±64 mm)
- `rgb_cam_x_front` / `rgb_cam_y_front` — front camera offsets (default 75 / 55 mm)
- `rgb_cam_x_rear` / `rgb_cam_y_rear` — rear camera offsets (default 95 / 70 mm)
- `rgb_cam_mount_spacing` — M3 bolt pattern square spacing (default 16.5 mm)
- `rgb_cam_layout` — camera positions: [angle, y_sign] pairs (default: front stereo + rear symmetric)
- `plate_thickness` — L1 structural plate stiffness (default 6 mm)
- `L2_thickness` — L2 plate thickness (default 12 mm, derived as `2 * plate_thickness`)
- `atlas_flange_extent` — L1 flange rear extent (default 150 mm)
- `lidar_angles` — LiDAR angular positions (default [0, 120, 240])

## Notes
- Atlas Duo CoN is precisely at the base center for simple lever arm calibration.
- GNSS antenna sits directly above CoN for optimal positioning.
- 1/4"-20 camera mount directly below CoN for balanced loading.
- All three axes (GNSS → CoN → camera) coaxial through Z.
- Hex-vertex brackets clear LiDAR bodies by ≥ 10 mm.
- Atlas-side brackets clear the Atlas body by 3–7 mm and LiDAR bodies by ≥ 10 mm.
- 4 RGB cameras: front stereo pair (110 mm baseline) flanking LiDAR@0°, rear pair at 120°/240° symmetric about X axis. Each camera's 158° DFOV (134° HFOV) overlaps its LiDAR's 120° HFOV.
