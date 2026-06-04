// ============================================================
// 3D Mapping Sensor Dome — v17j (FINAL)
//
// CHANGES FROM v17i (Option B from the 360°-coverage analysis):
//   - Rear-camera aim REVERTED from "outward along radial-through-
//     center" (atan2(y, x) = ±156.4°) to the LiDAR-frame radial
//     direction (literal 120° / 240°). This was Option B from the
//     coverage-vs-axis-through-center trade-off:
//       - The v17h "axis-through-center, INWARD" experiment left
//         the entire rear half of the world uncovered (175° gap).
//       - The v17i "axis-through-center, OUTWARD" fix put the lens
//         on the correct side of the dome, but with the rear cams
//         positioned near the cable cutout, the radial aim came
//         out at ±156°, leaving 22° gaps at each broadside.
//       - v17j gives up the axis-through-center property and
//         restores the v17g LiDAR-radial yaws. Full 360° coverage
//         comes back with healthy overlap (front 134° + rear-left
//         53°–187° + rear-right 173°–307°). The optical-axis line
//         now crosses the X axis at x ≈ −62.5 mm (about 6 cm
//         behind the imu_link origin) rather than at the origin —
//         acceptable for surround perception.
//   - All v17h/v17i POSITION changes are KEPT:
//       Front cameras at (85, ±73) — close to V0/V1 / V4/V5 edges,
//         146 mm stereo baseline, 26 mm wall to front-LiDAR M6.
//       Rear cameras at global (-108.1, ±47.3) — close to the
//         cable cutout, 19 mm wall to rear-LiDAR M6.
//   - aim_away_from_origin() helper deleted (no longer used).
//
// CHANGES FROM v17h (camera-placement corrections):
//   - Rear-camera aim flipped from "inward (look at center)" to
//     "outward along the radial through the center". The optical-
//     axis LINE still passes through the dome center axis (X=Y=0)
//     as required, but the LENS now points away from the center,
//     so each rear camera's field of view covers the world OUTSIDE
//     the dome rather than looking back across L2 at itself.
//     Yaw change:
//       cam_rear_left   -23.616°  →  +156.384°
//       cam_rear_right  +23.616°  →  -156.384°
//     The 180° flip leaves the M3 bolt pattern in the same holes
//     (4-bolt 16.5×16.5 square is invariant under 180° rotation),
//     so no L2 drill positions change.
//   - Front cameras pushed further toward V0/V1 and V4/V5 edges:
//       (rgb_cam_x_front, rgb_cam_y_front)
//       v17g: (105, 52)  →  v17h: (105, 65)  →  v17i: (85, 73)
//     The camera moves 20 mm rearward AND 8 mm further laterally.
//     Net wall-to-side-2 from each upper M3 hole edge is now
//     2.65 mm (printable); net wall-to-front-LiDAR-M6 grows to
//     25.6 mm (was 14 mm in v17h, 1.6 mm in v17g). Side-effect:
//     front-stereo baseline widens to 146 mm (was 130 in v17h,
//     104 in v17a–g) — even better depth-resolution geometry.
//
// CHANGES FROM v17g (camera placement vs LiDAR mount interference):
//   - v17g flagged that the front-stereo cameras at (105, ±52)
//     and the rear cameras at the same local (105, 52) rotated
//     by 120°/240° both sit within ~9 mm of the nearest Robin W
//     M6 top-mount countersink edge. With a ø11 countersink and
//     ø3.4 M3 camera bolt, the structural wall between them was
//     ~1.6 mm — a printable but uncomfortable margin.
//   - v17h moves both pairs into the empty edges next to them:
//
//     Front pair (cameras at angle 0°):
//       (rgb_cam_x_front, rgb_cam_y_front)
//       OLD: (105, ±52)  →  NEW: (105, ±65)
//       The lateral move pushes the M3 mount holes toward the
//       V0–V1 (left, 60° side) and V4–V5 (right, 300° side)
//       edges of the hex — both of which were unused. The front
//       LiDAR bolts at (109, ±36) / (51, ±36) now sit 21 mm
//       away from the nearest camera M3, giving a 14 mm wall.
//       Side-effect: front-stereo baseline grows 104 → 130 mm,
//       slightly better triangulation for depth resolution.
//
//     Rear pair (cameras at angles 120° / 240°):
//       (rgb_cam_x_rear, rgb_cam_y_rear)
//       OLD: (105, ±52)  →  NEW: (95, ±70)
//       Each rear camera moves rearward and outward in its
//       local frame, which in the rotated global frame pulls
//       the camera closer to the cable-cutout teardrop on side
//       4 (rear flange) and away from the rear LiDARs. Rear-
//       left camera moves from (-97.5, +64.9) to (-108.1, +47.3),
//       gaining 27 mm of vertical separation from the cable-
//       cutout upper diagonal and pushing the camera ~27 mm away
//       from the nearest rear-left LiDAR M6 — 20 mm wall.
//
//     *** REAR-CAMERA AIM RE-POINTED AT DOME CENTER (v17h) ***
//     In v17g the rear cameras pointed radially OUTWARD (along
//     the 120° / 240° rays), which placed their optical axes
//     skew to the dome's vertical Z axis. v17h rotates each
//     rear camera about its own Z axis so its optical axis
//     (camera local +X) passes through the dome center (X=0,
//     Y=0). The aim angle is computed by `aim_away_from_origin()` as
//     atan2(-gy, -gx) for global position (gx, gy):
//       rear-left  aim = -23.62°  (global, was +120°)
//       rear-right aim = +23.62°  (global, was +240°/-120°)
//     The L2 M3 bolt pattern rotates with the camera body, so
//     the holes drilled in L2 stay aligned with the re-aimed
//     mount. Verified clearances: M3 ↔ LiDAR M6 wall = 20 mm,
//     all four M3 bolts inside the v17g irregular hex outline,
//     6.7 mm minimum margin to the cable-cutout upper diagonal.
//
//     Front cosmetic note: the front-stereo body upper-right
//     corner at (131.5, 88) overhangs the new front edge at
//     X=121.32 by 10 mm in X and the side-2 diagonal by ~14 mm
//     visually. M3 bolts all stay inside the plate (top-right
//     M3 at (113.25, 73.25) is 1.5 mm inside side 2 — tight).
//     Rear cosmetic note: each rear-camera body overhangs the
//     cable-cutout teardrop boundary by a few mm; bolts stay
//     in the plate.
//
//   - rgb_cam_layout now carries per-row local (x, y) so front
//     and rear can be tuned independently. If you ever want to
//     restore v17g coincident positions, set
//       rgb_cam_x_front = rgb_cam_x_rear = 105
//       rgb_cam_y_front = rgb_cam_y_rear = 52
//     and the layout stays valid.
//
//   - config/sensor_dome_tf.yaml updated to match:
//       cam_front_*  y: ±0.052 → ±0.065
//       cam_rear_*   (x, |y|): (-0.0975, 0.0649) → (-0.10812, 0.04727)
//
// CHANGES FROM v17f (further compaction — irregular hex outline):
//   - Hex vertices V0 (30°), V1 (90°), V4 (270°), V5 (330°) are
//     pulled radially inward by hex_vertex_pullback = 10 mm.
//     V2 (150°) and V3 (210°) — the two rear vertices on either
//     side of the Atlas flange — are unchanged.
//     The plate outline is no longer a regular hexagon; it
//     becomes an irregular 6-gon, still mirror-symmetric about
//     the X axis (so the dome stays left/right symmetric).
//   - All 4 hex-vertex support columns (V0/V1/V4/V5 brackets)
//     move inward with their vertices, so each column still
//     connects flush to the L1 and L2 edges. Bracket bolt
//     positions, fillets, and tap holes all follow.
//   - The 2 Atlas-side brackets and the rear flange (anchored
//     by V2/V3 and the rear of the Atlas Duo body) are
//     unchanged from v17f.
//   - New vertex positions:
//       V0: (130, 75.05)  →  (121.33, 70.05)
//       V1: (0,  150.10)  →  (0,      140.10)
//       V4: (0, -150.10)  →  (0,     -140.10)
//       V5: (130,-75.05)  →  (121.33,-70.05)
//     Side 1 (front edge, between V5 & V0) shrinks from
//     X=130, Y=±75.05 to X=121.33, Y=±70.05 — about 8.7 mm
//     closer to the dome center and ~5 mm shorter on each
//     side. Total L2 plate area drops by ~6%.
//
//   *** OVERHANG TRADE-OFF — front of dome ***
//   Pulling V0/V5 inward also moves side 1 from X=130 to X=121.3,
//   which RE-INTRODUCES some of the front-LiDAR overhang that
//   v17f had just eliminated. With ring radius = 80 and Robin W
//   body depth 106.7:
//        front LiDAR body max X      = 80 + 53.35 = 133.35 mm
//        new side 1 front edge       =                121.32 mm
//        new front overhang          =                 12.03 mm
//      (was 3.35 mm at v17f side 1 = 130 mm)
//   The overhang is structurally harmless — the LiDAR top-mount
//   M6 countersinks reach only r=114.5, which is still 6.8 mm
//   inside the new front edge. Bracket bolts and camera M3 bolts
//   are also all inside the new outline. The overhang is purely
//   visual: the Robin W body sticks out past the L2 hex edge by
//   ~12 mm at the front when viewed from above. If that visual
//   matters to you, options are (a) reduce lidar_ring_radius
//   further (80 → 71 zeroes the overhang at the new edge), or
//   (b) restore some of the pullback (lower hex_vertex_pullback
//   from 10 to 5 puts overhang back to ~7 mm). The same pullback
//   also makes the FRONT-STEREO CAMERAS at (105, ±52) overhang
//   the new front-edge Y-span by ~5 mm cosmetically — M3 bolt
//   holes at |Y|≤60.25 are still safely inside the plate, so
//   the structural mount is unaffected.
//
// CHANGES FROM v17e (post-print compaction & tolerance fixes):
//   - LiDAR ring radius shortened: 90 mm → 80 mm (−10 mm).
//     The Robin W bodies stuck out past the L2 hex side edges
//     by ~13 mm at angle 0° in v17e; reducing the ring radius
//     by 10 mm pulls every body back to ~3 mm overhang (purely
//     cosmetic) while keeping the M6 top-mount bolt clearance
//     to the hex edge ample (~15.5 mm wall, up from ~5.5 mm).
//     Cascade: the LiDAR TF entries in config/sensor_dome_tf.yaml
//     get updated to match (lidar_front x: 0.090 → 0.080;
//     rear pair x: -0.045 → -0.040, |y|: 0.077942 → 0.069282).
//     *** CLEARANCE NOTE — repeat from v17e ***
//     The Z gap between the LiDAR body bottoms (z=54) and the
//     Atlas Duo body top (z=53.8) is still only 0.2 mm. Pulling
//     the LiDAR ring inward GROWS the XY overlap between each
//     LiDAR body and the Atlas footprint, so any sag or print
//     warp now has more area in which to cause physical contact.
//     If you have not already, consider raising L1_pillar_height
//     by a few mm before printing v17f.
//   - GNSS stand recess enlarged: ø86 → ø88 (+1 mm radius).
//     Post-print shrinkage made the ø86 pocket too tight against
//     the nominal ø88 magnetic base; ø88 lets the stand seat
//     without forcing. Floor thickness drops from 5 mm to 4 mm,
//     wall to nearest LiDAR countersink grows because the LiDAR
//     ring moved inward (see above).
//   - Atlas Duo center-pair bolt holes are now SLOTS, not round.
//     The two FRONT bolt holes (at x = +78.7 mm, closer to the
//     dome center) become slotted ±2 mm in the X direction so
//     the screws can shift fwd/aft to compensate for print
//     tolerance vs the Atlas's nominal hole pattern. The two
//     REAR bolts (x = −141.3 mm, on the flange) stay round —
//     once the center pair is in, they pin the rotation and the
//     rear pair lines up naturally.
//
// CHANGES FROM v17d:
//   - L1_pillar_height nudged 130mm → 133mm (+3mm) to remove the
//     LiDAR-vs-Atlas-body interference that v17d introduced.
//     New LiDAR-bottom = z=54mm vs Atlas-top = z=53.8mm.
//     Z-gap = 0.2mm — geometrically non-overlapping, but extremely
//     tight: it leaves NO room for cable routing or print-tolerance
//     between the LiDAR housings and the Atlas body. Consider
//     raising to 138–145mm (5–12mm gap) if you want practical
//     cable clearance and a printable margin.
//     Net change vs original v17c: pillar is 27mm shorter
//     (160 → 133), L2 sits at z=139 (was z=166).
//     M6×20mm bracket BHCS still works (12mm L2 + 8mm tap = 20mm).
//
// CHANGES FROM v17c:
//   - L1_pillar_height SHORTENED from 160mm → 130mm (−30mm) in v17d.
//     Caused a 2.8mm LiDAR-vs-Atlas-body collision; corrected
//     in v17e by adding 3mm back (see above).
//
// CHANGES FROM v17b:
//   - INSIDE-CORNER FILLETS on ALL bracket-to-plate junctions.
//     Every inside corner (where plate extends beyond the bracket
//     wall) gets a smooth 10mm hull()-based fillet transition.
//   - HEX-VERTEX BRACKETS: fillet on 3 sides (inner face + both
//     short ends). Outer face stays flush with hex side edge.
//   - ATLAS-SIDE BRACKETS: fillet on 2 sides (+X toward hex center,
//     and Atlas-facing with reduced radius: 5mm upper / 2mm lower
//     to preserve Atlas body clearance).
//   - New params: bracket_fillet_r=10, atlas_fillet_inner_upper=5,
//     atlas_fillet_inner_lower=2.
//
// CHANGES FROM v17:
//   - V2/V3 hex-vertex brackets REPLACED by two "Atlas-side"
//     brackets: flat walls running in the +X direction alongside
//     the Atlas INS body (one above, one below in Y).
//     Positioned at y=±64mm (Atlas body Y range: -61 to +56.8),
//     from x=-150 to x=-115 (35mm long, 10mm thick).
//     Clears Atlas body by 3–7mm and LiDAR bodies by ≥10mm.
//   - L2 plate now has the SAME flange as L1 (hex + side-4 flange)
//     so both plates cover the Atlas-side brackets.
//   - L2 plate thickness DOUBLED from 6mm to 12mm, giving a deeper
//     GNSS antenna stand recess (8mm depth, 4mm floor).
//   - Bolt lengths updated for thicker L2:
//       Bracket BHCS: M6×20mm (12mm plate + 8mm tap engagement)
//       LiDAR SHCS:   M6×16mm (8mm pass-through + 8mm into LiDAR)
//
// CHANGES FROM v16 (carried forward from v17):
//   - Both plates changed to REGULAR HEXAGON shape (inradius 130mm).
//     LiDARs mount on alternating sides 1, 3, 5 (at 0°, 120°, 240°).
//     Atlas Duo oriented along sides 1–4 axis (+X to -X).
//   - LiDAR ring radius increased from 65mm to 90mm.
//     Body-to-body gap at center: 51mm (was 8mm). Much more space.
//   - Both plates = hex + rectangular flange at side 4.
//   - 6 brackets total: 4 single-wall at hex vertices V0,V1,V4,V5
//     (along blank sides 2 and 6) + 2 Atlas-side brackets running
//     in +X alongside the Atlas body.
//   - Bracket bolts: M6 button-head (BHCS), clearance-only
//     through L2 (no countersink, head sits on L2 top surface).
//     12 total (2 per bracket × 6 brackets).
//   - L2 cable opening: teardrop cutout on side 4 (L2 only),
//     tip circle ø30mm at (-100,0), base ±60mm between brackets.
//   - GNSS recess has 12.5mm wall to nearest LiDAR countersink.
//   - BOM: 24× M6 (12 bracket BHCS + 12 LiDAR SHCS), 4× M4, etc.
//
// Prior changes still in effect:
//   v15: GNSS mast eliminated, commercial magnetic stand in recess.
//   v14: All inter-level bolts M6. Single ø30mm cable hole.
//   v13: L3 eliminated. LiDARs top-mount below L2. 2 printed parts.
//
// Level 1 (Base): Atlas Duo (CoN up), 1/4"-20 camera mount below
//   → 4 hex-vertex brackets + 2 Atlas-side brackets UP to Level 2
//   → Hex plate + rear flange for Atlas bolts
// Level 2 (Combined):
//   Bottom: 3× Robin W at 120° (top mount, hanging below)
//   Top: GNSS stand recess (center), cable hole,
//        4× RouteCAM_P_CU25_CXLC_IP67 (front stereo pair + rear pair)
//   → Hex plate + rear flange (same shape as L1)
//
// Hex side numbering (outward normal angle):
//   Side 1: 0°   (LiDAR@0°)      Side 2: 60°  (blank)
//   Side 3: 120° (LiDAR@120°)    Side 4: 180° (Atlas long axis, -X)
//   Side 5: 240° (LiDAR@240°)    Side 6: 300° (blank)
//
// Vertex numbering (between adjacent sides):
//   V0: 30° (1-2)    V1: 90° (2-3)    V2: 150° (3-4)
//   V3: 210° (4-5)   V4: 270° (5-6)   V5: 330° (6-1)
//
// All centers (GNSS, LiDAR ring, Atlas CoN, camera) coaxial at X=0, Y=0.
//
// RENDER_MODE: 0=assembly (default), 1=level1, 2=level2
// ============================================================

RENDER_MODE = 0;

// v17h: when unibody_mode = true, the bolt holes that would otherwise
// be drilled at each bracket top (L1 tap pockets) AND through L2
// (clearance through-holes) are SKIPPED entirely. Use this when
// printing the dome as a single unibody part — there are no bolts to
// install through those holes, so they're just stress concentrators.
// The two-piece bolted assembly (RENDER_MODE 1 + RENDER_MODE 2)
// keeps the default unibody_mode = false and drills the holes.
//
// The unibody wrapper (3D files/sensor_dome_unibody.scad) sets this
// to true after `include <sensor_dome.scad>`.
unibody_mode = false;

// ===================== PARAMETERS ============================

// --- Atlas Duo ---
atlas_bolt_width_pattern = 220;
atlas_bolt_height_pattern = 100;
atlas_bolt_dia = 4.5;
atlas_bolt_csink_dia = 8.0;
atlas_bolt_csink_depth = 4.0;

atlas_con_x_from_datum = 68.7;
atlas_con_y_from_datum = 47.8;
atlas_front_bolt_x = 10;
atlas_rear_bolt_x = 210;

datum_x = atlas_con_x_from_datum;
datum_y = atlas_con_y_from_datum;

atlas_bolts = [
    [ datum_x + atlas_front_bolt_x,   datum_y],
    [ datum_x + atlas_front_bolt_x,  -(atlas_bolt_height_pattern - datum_y)],
    [ datum_x - atlas_rear_bolt_x,    datum_y],
    [ datum_x - atlas_rear_bolt_x,   -(atlas_bolt_height_pattern - datum_y)]
];

// v17f: print-tolerance slots for the FRONT pair (closer to dome
// center). Each front bolt hole is a slot running ±atlas_slot_play
// mm in X, so the M4 screw can shift fwd/aft to land in the Atlas
// thread despite ~0.1–0.3 mm dimensional drift typical of FDM PETG.
// The REAR pair stays round and pins rotation once the front pair
// is engaged.
atlas_slot_play = 2.0;          // mm of fwd/aft play in the slot

atlas_body_length = 235.3;
atlas_body_width = 117.8;
atlas_body_height = 47.8;

// --- Robin W: TOP mount pattern (for under-L2 mounting) ---
// From Seyond Robin W1G Manual §2.1 top view drawing.
// 4× M6×1.0 ▽5 in isosceles trapezoid.
// "Front" = window/scanning side (wider), "Back" = connector side (narrower).
robin_top_front_width   = 72;   // front bolt pair spacing (window side)
robin_top_back_width    = 68;   // back bolt pair spacing (connector side)
robin_top_height_spacing = 58;  // front-to-back bolt row spacing
robin_top_bolt_dia      = 6.5;  // M6 clearance hole
robin_top_csink_dia     = 11.0; // M6 SHCS head clearance
robin_top_csink_depth   = 4.0;  // countersink depth from L2 top

robin_top_bolts = [
    [ robin_top_height_spacing/2,  robin_top_front_width/2],
    [ robin_top_height_spacing/2, -robin_top_front_width/2],
    [-robin_top_height_spacing/2,  robin_top_back_width/2],
    [-robin_top_height_spacing/2, -robin_top_back_width/2]
];

// Robin W body dimensions (for ghost and clearance)
robin_body_w = 104.8;   // width (Y in LiDAR local frame)
robin_body_h = 85;       // height (Z when upright)
robin_body_d = 106.7;   // depth (X, scanning direction)

// v17f: 80 mm (was 90 mm in v17–v17e). Pulls LiDAR bodies back
// inside the L2 hex outline; outermost bolt countersink at r=114.5
// (was r=124.5) leaves a comfortable 15.5 mm wall to the hex edge.
lidar_ring_radius = 80;
lidar_angles = [0, 120, 240];

// --- Hexagonal Plate (v17) ---
plate_thickness = 6;
corner_r = 10;

// Regular hexagon geometry. Side 1 faces +X (outward normal at 0°).
// Orientation: rotate standard circle($fn=6) by 30° so flat side
// faces +X and vertices are at 30°, 90°, 150°, 210°, 270°, 330°.
hex_inradius = 130;                              // center to flat side
hex_circumradius = hex_inradius / cos(30);       // center to vertex ≈ 150.1

// Atlas rear bolt flange (L1 AND L2 in v17b).
// Atlas rear bolts at x=-141.3 are 11.3mm beyond hex side 4 (at x=-130).
// Flange extends side 4 rearward to cover those bolts and the
// Atlas-side brackets. Both plates share this flange shape.
// Width matches hex side 4 span (vertex Y = ±R/2 ≈ ±75.1mm).
atlas_flange_extent = 150;                        // -X extent of flange
atlas_flange_half_width = hex_circumradius / 2;   // ≈ 75.1mm

// --- Inter-Level Bolts (M6) ---
interlevel_bolt_dia = 6.5;     // M6 clearance hole
interlevel_csink_dia = 11.0;   // M6 SHCS head clearance (LiDAR bolts)
interlevel_csink_depth = 4.0;
interlevel_tap_dia = 5.5;      // M6 self-tap pilot in printed plastic
interlevel_tap_depth = 10;

// --- Hex Vertex Brackets (4 of 6 — V0, V1, V4, V5) ---
// Single-wall brackets at hex vertices along blank sides 2 and 6.
// Wall is 12mm thick × 35mm long, extruded 160mm tall.
//
// Even vertices keep wall toward NEXT vertex (d = a+120).
// Odd vertices keep wall toward PREV vertex (d = a-120).
// V0, V1 → walls along blank side 2.
// V4, V5 → walls along blank side 6.
//
// V2 and V3 are NOT used (they would collide with Atlas body on
// side 4 or with LiDAR bodies on sides 3/5). Instead, two custom
// "Atlas-side brackets" replace them (see below).

bracket_wall_t   = 12;    // wall thickness for hex vertex brackets
bracket_leg_len  = 35;    // wall length along hex side
bracket_fillet_r = 10;    // fillet radius at bracket-to-plate junction
// Atlas-side brackets: fillet toward Atlas body is limited by clearance.
// Upper bracket clearance: 64 - 56.8 = 7.2mm → use 5mm.
// Lower bracket clearance: 64 - 61.0 = 3.0mm → use 2mm.
atlas_fillet_inner_upper = 5;   // Atlas-facing fillet, upper bracket
atlas_fillet_inner_lower = 2;   // Atlas-facing fillet, lower bracket

hex_vertex_angles = [30, 90, 150, 210, 270, 330];

// Indices of hex vertices that get brackets (V0, V1, V4, V5 only)
hex_bracket_vertices = [0, 1, 4, 5];

// v17g: irregular-hex compaction. The four vertices that carry
// brackets (V0/V1/V4/V5 — see hex_bracket_vertices) are pulled
// radially inward by hex_vertex_pullback mm. V2 and V3 (the two
// vertices on either side of the rear flange) stay at the original
// circumradius so the flange geometry is unchanged.
hex_vertex_pullback = 10;

function is_pulled_vertex(i) =
    (i == 0 || i == 1 || i == 4 || i == 5);

function hex_vertex_xy(i) =
    let(a = hex_vertex_angles[i],
        r = is_pulled_vertex(i)
              ? hex_circumradius - hex_vertex_pullback
              : hex_circumradius)
    [r * cos(a), r * sin(a)];

// Bracket wall direction and inward normal (simple even/odd rule).
// Note: only the VERTEX POSITION moves with the pullback; the
// direction vectors stay tied to the original 30°/90°/270°/330°
// rays, so the bracket walls point the same way as in v17e/v17f.
function bracket_wall_dir(i) =
    let(a = hex_vertex_angles[i])
    (i % 2 == 0) ? a + 120 : a - 120;

function bracket_wall_norm(i) =
    let(a = hex_vertex_angles[i])
    (i % 2 == 0) ? a - 150 : a + 150;

// Bolt positions: 2 per bracket (at 30% and 70% along wall length).
// v17g: vertex anchor now comes from hex_vertex_xy(i) so brackets
// at V0/V1/V4/V5 move inward with their pulled vertices.
function bracket_bolt_pos(i, j) =
    let(
        v  = hex_vertex_xy(i),
        vx = v[0],
        vy = v[1],
        d  = bracket_wall_dir(i),
        n  = bracket_wall_norm(i),
        frac = (j == 0) ? 0.3 : 0.7
    )
    [vx + bracket_leg_len * frac * cos(d) + bracket_wall_t/2 * cos(n),
     vy + bracket_leg_len * frac * sin(d) + bracket_wall_t/2 * sin(n)];

// --- Atlas-Side Brackets (2 of 6 — replacing V2/V3) ---
// Two flat walls running in the +X direction alongside the Atlas
// INS body, one above (y=+64) and one below (y=-64).
// These sit on the hex+flange plate area and clear both the Atlas
// body (gap ≥3mm) and the rear LiDAR bodies (gap ≥10mm).
// Wall is 10mm thick × 35mm long, extruded 160mm tall.
// Thickness extends AWAY from Atlas (+Y for upper, -Y for lower).

atlas_bracket_x_start   = -150;   // starts at flange rear edge
atlas_bracket_leg_len   = 35;     // runs +X for 35mm to x=-115
atlas_bracket_wall_t    = 10;     // slightly thinner than hex brackets
atlas_bracket_y_upper   = 64;     // upper wall inner edge Y
atlas_bracket_y_lower   = -64;    // lower wall inner edge Y

// Atlas-side bracket bolt positions (2 per bracket × 2 brackets = 4)
function atlas_bracket_bolt_pos(side, j) =
    let(
        frac = (j == 0) ? 0.3 : 0.7,
        bx = atlas_bracket_x_start + atlas_bracket_leg_len * frac,
        y_base = (side == 0) ? atlas_bracket_y_upper : atlas_bracket_y_lower,
        y_sign = (side == 0) ? 1 : -1,
        by = y_base + y_sign * atlas_bracket_wall_t / 2
    )
    [bx, by];

// Combined bolt positions for all 6 brackets (8 hex-vertex + 4 atlas-side = 12)
hex_bracket_bolt_positions = [
    bracket_bolt_pos(0, 0), bracket_bolt_pos(0, 1),   // V0 (30°)
    bracket_bolt_pos(1, 0), bracket_bolt_pos(1, 1),   // V1 (90°)
    bracket_bolt_pos(4, 0), bracket_bolt_pos(4, 1),   // V4 (270°)
    bracket_bolt_pos(5, 0), bracket_bolt_pos(5, 1),   // V5 (330°)
    atlas_bracket_bolt_pos(0, 0), atlas_bracket_bolt_pos(0, 1),  // upper Atlas-side
    atlas_bracket_bolt_pos(1, 0), atlas_bracket_bolt_pos(1, 1)   // lower Atlas-side
];

// Camera mount
camera_thread_dia = 6.5;
camera_insert_depth = 5;

// --- Level 1 ---
L1_thickness = plate_thickness;

// Pillar height: must clear Atlas body + cables + LiDAR bodies below L2.
// Atlas top = L1_thickness + atlas_body_height = 53.8mm.
// LiDAR bottom = L2_z_bottom - robin_body_h.
// v17e: L1_pillar_height = 133 (was 130 in v17d, 160 in v17c).
//   LiDAR_bottom = 139-85 = 54mm vs Atlas_top = 53.8mm.
//   Gap = 0.2mm — non-overlapping but very tight (see header).
L1_pillar_height = 133;

// --- Level 2 ---
L2_z_bottom = L1_thickness + L1_pillar_height;   // 166
L2_thickness = 2 * plate_thickness;                // 12  (v17b: doubled for deeper GNSS recess)
L2_z_top = L2_z_bottom + L2_thickness;            // 178 (was 172 in v17)

// --- GNSS Magnetic Stand Recess (v15) ---
// Commercial magnetic GNSS stand (e.g. ArduSimple AS-ACC-SURVEYSTAND-01)
// glued into a circular centering recess on L2 top surface.
// *** MEASURE YOUR STAND AND ADJUST gnss_stand_base_dia IF NEEDED ***
gnss_stand_base_dia    = 88;    // magnetic base outer diameter (MEASURE!)
// v17f: 88 (was 86 in v17-v17e). +1 mm radius accommodates post-print
// shrinkage; the ø88 nominal base now seats without forcing.
gnss_stand_recess_dia  = 88;    // centering pocket
gnss_stand_recess_depth = 8;    // recess depth (leaves 4mm floor in 12mm plate)

// Cable opening — teardrop cutout on side 4 (L2 ONLY)
// Replaces old ø30mm cable hole.
// Carries 4× shielded PoE camera cables + 1× GNSS cable.
//
// Shape: hull() of a tip circle and a base rectangle.
//   Tip circle: ø30mm at (-100, 0) — round termination.
//   Base rectangle: x = [-150, -140] (half the flange length),
//     y = ±60 mm (fits between Atlas-side brackets).
//   The hull creates smooth tangent lines from the circle to the
//   base, giving a teardrop that widens gradually from the tip.
//
// Triangle height (tip to base inner edge): 40 mm (doubled from v17b).
// Base half-width 60 mm leaves 4 mm wall to bracket inner edge (y=±64).
// Bracket bolt holes at y=±69 have ≥6.6 mm clearance.
// GNSS recess clearance: 42 mm. Camera hole clearance: ≥37 mm.
//
// NOT applied to L1 — L1 plate remains solid under the brackets.
cable_opening_tip_x        = -100;  // tip circle center X
cable_opening_tip_r        = 15;    // tip circle radius (ø30mm)
cable_opening_base_half_w  = 60;    // base half-width (between brackets)
cable_opening_base_outer_x = -150;  // base outer edge (flange edge)
cable_opening_base_inner_x = -140;  // base inner edge (half flange)

// --- RGB Camera: RouteCAM_P_CU25_CXLC_IP67 ---
// Body: 46 × 46 × 52.95 mm (without lens), 65.12 mm total height with lens
// FOV: 158° DFOV, 134° HFOV, 73° VFOV (with 1/2.6" AR0234 sensor)
// Bottom mount: 4× M3×0.5 threaded holes, 5 mm deep, 16.5 mm square spacing
// Side/top mount: 4× M2×0.4 threaded holes (not used here)
// IP67, PoE, 2MP global shutter, M12 X-coded Ethernet
//
// Placement: 4 cameras in two groups:
//   Front stereo pair — two cameras at 0° (alongside LiDAR@0°),
//     offset ±Y for a stereo baseline of 2×52 = 104 mm.
//   Rear symmetric pair — one camera at 120° (+Y), one at 240° (−Y),
//     mirrored about the global X axis for symmetric coverage.
// All cameras face radially outward, parallel to their LiDAR's scanning
// direction, for maximum FOV overlap.
rgb_cam_body_w    = 46;       // body width and depth (square cross-section)
rgb_cam_body_d    = 52.95;    // body depth without lens
rgb_cam_body_h_total = 65.12; // total height with lens protrusion
rgb_cam_mount_spacing = 16.5; // M3 bolt pattern square spacing
rgb_cam_bolt_dia  = 3.4;      // M3 clearance hole diameter

// v17h: front and rear cameras now carry their own (x, y) so each pair
// can dodge the LiDAR top-mount bolts independently. See header.
//
//   FRONT pair (cameras facing 0°) — pushed laterally toward the unused
//   V0–V1 and V4–V5 hex edges (sides 2 and 6). The local frame here is
//   the un-rotated camera frame: +X = camera optical axis (= forward),
//   +Y = camera's left.
//   v17i: pushed even further toward V0/V1 and V4/V5, with the camera
//   body pulled rearward to keep a comfortable wall to side 2. New
//   numbers leave a 2.6 mm wall from each upper-Y M3 hole edge to side
//   2 and a 26 mm wall to the nearest front-LiDAR M6 countersink.
//   Side-effect: front-stereo baseline widens 130 → 146 mm.
rgb_cam_x_front   = 85;       // forward distance (was 105 v17a–h)
rgb_cam_y_front   = 73;       // lateral offset (was 52 v17a–g, 65 v17h)

//   REAR pair (cameras facing 120° / 240°) — pulled rearward in the
//   LiDAR-relative local frame, which in global coords pulls them
//   closer to the cable-cutout teardrop on the rear flange and away
//   from the rear LiDAR M6 mounts.
rgb_cam_x_rear    = 95;       // local radial distance (was 105)
rgb_cam_y_rear    = 70;       // local lateral offset (was 52)

// 2D rotation helper used to derive rear-camera global positions.
function rot2d(p, deg) =
    let(c = cos(deg), s = sin(deg))
    [p[0]*c - p[1]*s, p[0]*s + p[1]*c];

// Rear-camera GLOBAL positions are obtained by rotating their
// LiDAR-relative local placement by the dome angle (120° / 240°).
// This keeps the v17g semantic — "rear cameras sit alongside the
// rear LiDARs" — while letting us re-aim them independently below.
rear_left_global  = rot2d([rgb_cam_x_rear,  rgb_cam_y_rear], 120);
rear_right_global = rot2d([rgb_cam_x_rear, -rgb_cam_y_rear], 240);

// v17j FINAL — rear-camera aim is the LiDAR-frame radial direction
// (120° / 240°), matching the LiDARs they sit alongside. This is
// "Option B" from the 360°-coverage analysis: full horizontal
// coverage is restored at the cost of the v17h/v17i axis-through-
// center property. Each rear camera's optical axis no longer
// passes through (X=0, Y=0) — instead it crosses the X axis at
// approximately x = -62.5 mm — but the lens still points outward
// and the 4-camera union covers the full 360° horizon with overlap.
//
// History of rear-camera aim:
//   v17a–v17g : aim = LiDAR angle (120° / 240°)         → 360° ✓
//   v17h      : aim = atan2(-y, -x) (INWARD at center) → 175° gap (broken)
//   v17i      : aim = atan2( y,  x) (outward thru ctr) → two 22° gaps
//   v17j      : aim = 120° / 240°                       → 360° ✓ (final)
//
// Camera layout — one row per camera, with EXPLICIT (global_x,
// global_y, aim_deg). Front cameras face +X (aim 0°). Rear cameras
// face the LiDAR-frame radial-outward direction (aim 120° / 240°).
//
//   row format: [global_x, global_y, aim_deg]
rgb_cam_layout = [
    [ rgb_cam_x_front,  rgb_cam_y_front, 0],                  // front +Y
    [ rgb_cam_x_front, -rgb_cam_y_front, 0],                  // front -Y
    [ rear_left_global[0],  rear_left_global[1],  120],       // rear-left
    [ rear_right_global[0], rear_right_global[1], 240],       // rear-right
];

// Camera M3 bolt positions in local frame (bottom mount, 4 holes)
rgb_cam_bolt_offsets = [
    [ rgb_cam_mount_spacing/2,  rgb_cam_mount_spacing/2],
    [ rgb_cam_mount_spacing/2, -rgb_cam_mount_spacing/2],
    [-rgb_cam_mount_spacing/2,  rgb_cam_mount_spacing/2],
    [-rgb_cam_mount_spacing/2, -rgb_cam_mount_spacing/2]
];

// ===================== MODULES ===============================

module rounded_rect(w, h, r) {
    offset(r) offset(-r) square([w, h], center=true);
}

// v17f — slot 2D outline used by the front-pair Atlas bolt holes.
// Slot's long axis is the X axis, length = d + 2*atlas_slot_play.
// hull() of two end circles produces clean rounded ends and a
// printable, watertight 2D contour.
module atlas_slot_2d(d) {
    hull() {
        translate([-atlas_slot_play, 0]) circle(d=d, $fn=32);
        translate([ atlas_slot_play, 0]) circle(d=d, $fn=32);
    }
}

// v17g — irregular hexagon plate outline (L2 and base shape for L1).
// Vertices at 30°, 90°, 150°, 210°, 270°, 330°. V0/V1/V4/V5 are
// pulled inward by hex_vertex_pullback; V2/V3 stay at the original
// circumradius. corner_r rounding is preserved via offset+offset.
module hex_plate_outline() {
    offset(r=corner_r) offset(r=-corner_r)
        polygon([for (i = [0:5]) hex_vertex_xy(i)]);
}

// L1 plate: irregular hex + rectangular flange at side 4 for Atlas
// rear bolts. v17g: the hex is now the irregular outline (V0/V1/V4/V5
// pulled inward); side 4 (between V2 and V3) is unchanged, so the
// flange Y-span ±atlas_flange_half_width still matches V2/V3 cleanly.
module L1_plate_outline() {
    offset(r=corner_r) offset(r=-corner_r)
    union() {
        polygon([for (i = [0:5]) hex_vertex_xy(i)]);
        // Flange: rectangle extending side 4 in -X direction
        translate([-atlas_flange_extent, -atlas_flange_half_width])
            square([atlas_flange_extent - hex_inradius + 1,
                    2 * atlas_flange_half_width]);
    }
}

// Through-bolt + countersink holes (for LiDAR SHCS bolts)
module bolt_csink_at(positions, thickness) {
    for (pos = positions) {
        translate([pos[0], pos[1], -1])
            cylinder(d=interlevel_bolt_dia, h=thickness+2, $fn=32);
        translate([pos[0], pos[1], thickness-interlevel_csink_depth])
            cylinder(d=interlevel_csink_dia, h=interlevel_csink_depth+1, $fn=32);
    }
}

// Through-bolt only (for bracket BHCS bolts — no countersink,
// bolt head sits on L2 top surface)
module bolt_clearance_at(positions, thickness) {
    for (pos = positions) {
        translate([pos[0], pos[1], -1])
            cylinder(d=interlevel_bolt_dia, h=thickness+2, $fn=32);
    }
}

// =============== HEX VERTEX BRACKETS (V0, V1, V4, V5) ==========
// Each bracket is a single rectangular wall at a hex vertex,
// extending along the adjacent blank side (2 or 6).
// 2D profile: rectangle along the wall direction, thickness inward.

module hex_bracket_wall_profile(vertex_index) {
    a = hex_vertex_angles[vertex_index];
    d = bracket_wall_dir(vertex_index);
    n = bracket_wall_norm(vertex_index);

    // Rectangle: from vertex along direction d for leg_len,
    // with thickness wall_t in direction n (toward hex interior).
    polygon([
        [0, 0],
        [bracket_leg_len * cos(d), bracket_leg_len * sin(d)],
        [bracket_leg_len * cos(d) + bracket_wall_t * cos(n),
         bracket_leg_len * sin(d) + bracket_wall_t * sin(n)],
        [bracket_wall_t * cos(n), bracket_wall_t * sin(n)]
    ]);
}

// Expanded hex bracket profile for fillet base — grows on 3 sides:
//   - INWARD (along normal n, toward hex center) by fr
//   - BOTH SHORT ENDS (along wall direction d) by fr
// The OUTER face (along the hex side edge) stays flush — there is no
// inside corner on that side since the plate edge is there.
// Expansion kept small (fillet_r/5) so the profile stays within L1 plate.
module hex_bracket_fillet_profile(vertex_index) {
    a = hex_vertex_angles[vertex_index];
    d = bracket_wall_dir(vertex_index);
    n = bracket_wall_norm(vertex_index);
    fr = bracket_fillet_r / 5;        // reduced expansion to stay within plate
    t_exp = bracket_wall_t + fr;      // inward expansion

    polygon([
        // Outer edge: expanded along d at both ends but NOT along n
        [-fr * cos(d),
         -fr * sin(d)],
        [(bracket_leg_len + fr) * cos(d),
         (bracket_leg_len + fr) * sin(d)],
        // Inner edge: expanded along d at both ends AND along n
        [(bracket_leg_len + fr) * cos(d) + t_exp * cos(n),
         (bracket_leg_len + fr) * sin(d) + t_exp * sin(n)],
        [-fr * cos(d) + t_exp * cos(n),
         -fr * sin(d) + t_exp * sin(n)]
    ]);
}

// Place 4 hex-vertex brackets on L1 (V0, V1, V4, V5 only)
// Each bracket has:
//   - A fillet at the plate junction: hull from 3-side-expanded profile
//     at plate_thickness to normal profile at plate_thickness + fillet_r.
//     This smooths the inside vertical corners on the inner face and
//     both short ends where the wall meets the plate.
//   - The main wall above the fillet up to full pillar height.
module hex_brackets() {
    for (i = hex_bracket_vertices) {
        a = hex_vertex_angles[i];
        // v17g: vertex anchor comes from hex_vertex_xy(i) so the
        // bracket moves inward with the pulled vertex (V0/V1/V4/V5).
        v  = hex_vertex_xy(i);
        vx = v[0];
        vy = v[1];

        // Fillet: smooth inside-corner transition from plate to wall.
        // Clipped to L1 plate outline so nothing extends beyond the
        // rounded hex + flange boundary.
        intersection() {
            // Clip volume: L1 plate outline extruded through fillet zone
            translate([0, 0, plate_thickness])
                linear_extrude(bracket_fillet_r + 0.01)
                    L1_plate_outline();
            // Fillet hull: expanded base tapers to normal profile
            translate([vx, vy, plate_thickness])
                hull() {
                    linear_extrude(0.01)
                        hex_bracket_fillet_profile(i);
                    translate([0, 0, bracket_fillet_r])
                        linear_extrude(0.01)
                            hex_bracket_wall_profile(i);
                }
        }

        // Main wall: from fillet top to full pillar height.
        // Also clipped to plate outline so no part of the wall
        // hangs beyond the rounded plate corners.
        intersection() {
            translate([0, 0, plate_thickness + bracket_fillet_r])
                linear_extrude(L1_pillar_height - bracket_fillet_r + 0.01)
                    L1_plate_outline();
            translate([vx, vy, plate_thickness + bracket_fillet_r])
                linear_extrude(L1_pillar_height - bracket_fillet_r)
                    hex_bracket_wall_profile(i);
        }
    }
}

// =============== ATLAS-SIDE BRACKETS (replacing V2/V3) ==========
// Two flat walls running +X alongside the Atlas INS body.
// Upper wall: y = [64, 74], extending +Y from inner edge.
// Lower wall: y = [-74, -64], extending -Y from inner edge.
// Both from x = -150 to x = -115 (on flange + hex overlap).

module atlas_side_brackets() {
    // Atlas-side bracket fillet expansion:
    //   +X face (toward hex center): bracket_fillet_r (10mm) — ample room
    //   -X face (x=-150, flange edge): 0 — no plate beyond, no inside corner
    //   Atlas-facing face: limited by Atlas body clearance (5mm upper, 2mm lower)
    //   Outward face (away from Atlas): 0 — at/near plate edge, no inside corner
    //
    // All geometry (fillet + wall) is intersection()-clipped to
    // L1_plate_outline() so nothing hangs beyond the rounded flange corners.

    fr = bracket_fillet_r;

    // ---- Upper bracket (+Y side of Atlas) ----
    // Atlas-facing = y=0 edge in local coords (global y=64)
    // Outward = y=wall_t edge (global y=74)

    // Fillet — clipped to plate outline
    intersection() {
        translate([0, 0, plate_thickness])
            linear_extrude(fr + 0.01)
                L1_plate_outline();
        translate([atlas_bracket_x_start, atlas_bracket_y_upper, plate_thickness])
            hull() {
                linear_extrude(0.01)
                    translate([0, -atlas_fillet_inner_upper])
                        square([atlas_bracket_leg_len + fr,
                                atlas_bracket_wall_t + atlas_fillet_inner_upper]);
                translate([0, 0, fr])
                    linear_extrude(0.01)
                        square([atlas_bracket_leg_len, atlas_bracket_wall_t]);
            }
    }
    // Main wall above fillet — clipped to plate outline
    intersection() {
        translate([0, 0, plate_thickness + fr])
            linear_extrude(L1_pillar_height - fr + 0.01)
                L1_plate_outline();
        translate([atlas_bracket_x_start, atlas_bracket_y_upper,
                   plate_thickness + fr])
            linear_extrude(L1_pillar_height - fr)
                square([atlas_bracket_leg_len, atlas_bracket_wall_t]);
    }

    // ---- Lower bracket (-Y side of Atlas) ----
    // Atlas-facing = y=wall_t edge in local coords (global y=-64)
    // Outward = y=0 edge (global y=-74)

    // Fillet — clipped to plate outline
    intersection() {
        translate([0, 0, plate_thickness])
            linear_extrude(fr + 0.01)
                L1_plate_outline();
        translate([atlas_bracket_x_start,
                   atlas_bracket_y_lower - atlas_bracket_wall_t,
                   plate_thickness])
            hull() {
                linear_extrude(0.01)
                    square([atlas_bracket_leg_len + fr,
                            atlas_bracket_wall_t + atlas_fillet_inner_lower]);
                translate([0, 0, fr])
                    linear_extrude(0.01)
                        square([atlas_bracket_leg_len, atlas_bracket_wall_t]);
            }
    }
    // Main wall above fillet — clipped to plate outline
    intersection() {
        translate([0, 0, plate_thickness + fr])
            linear_extrude(L1_pillar_height - fr + 0.01)
                L1_plate_outline();
        translate([atlas_bracket_x_start,
                   atlas_bracket_y_lower - atlas_bracket_wall_t,
                   plate_thickness + fr])
            linear_extrude(L1_pillar_height - fr)
                square([atlas_bracket_leg_len, atlas_bracket_wall_t]);
    }
}

// =============== TAP HOLES (all 12 brackets) ====================
// Tap holes at bracket tops for all 6 brackets (12 bolts total)
module all_bracket_tap_holes() {
    for (pos = hex_bracket_bolt_positions) {
        translate([pos[0], pos[1],
                   plate_thickness + L1_pillar_height - interlevel_tap_depth])
            cylinder(d=interlevel_tap_dia, h=interlevel_tap_depth + 1, $fn=32);
    }
}

// =============== CABLE OPENING (triangle cutout on side 4) ========
// 2D polygon for the cable opening. Oversized triangle — the plate
// outline clips it automatically via difference().

module cable_opening_2d() {
    // Teardrop: hull of tip circle + base rectangle.
    // The hull produces smooth tangent lines from the circle
    // to the base corners, with a round termination at the tip.
    // The base rectangle extends past the plate boundary so the
    // plate outline clips it to the correct shape via difference().
    far_x = -300;  // well beyond plate/flange boundary
    hw = cable_opening_base_half_w;
    hull() {
        // Tip circle
        translate([cable_opening_tip_x, 0])
            circle(r=cable_opening_tip_r, $fn=64);
        // Base rectangle (extends to plate edge and beyond)
        translate([far_x, -hw])
            square([abs(far_x) - abs(cable_opening_base_inner_x), 2*hw]);
    }
}

// =============== AXIS ETCHING (1mm deep on plate top) ========

etch_depth = 1;
etch_text_size = 8;
etch_line_w = 1.5;
etch_arrow_len = 25;
etch_head_len = 5;
etch_head_w = 4;

module arrow_2d() {
    translate([0, -etch_line_w/2])
        square([etch_arrow_len - etch_head_len, etch_line_w]);
    translate([etch_arrow_len - etch_head_len, 0])
        polygon([
            [0, -etch_head_w],
            [etch_head_len, 0],
            [0, etch_head_w]
        ]);
}

module x_axis_etch_2d() {
    translate([5, -25]) {
        text("+X", size=etch_text_size, halign="right", valign="center");
        translate([2, 0]) arrow_2d();
    }
}

module y_axis_etch_2d() {
    translate([-30, 5]) {
        translate([0, -2])
            text("+Y", size=etch_text_size, halign="center", valign="top");
        translate([0, 0]) rotate([0, 0, 90]) arrow_2d();
    }
}

module axis_etching(z_top) {
    translate([0, 0, z_top - etch_depth])
        linear_extrude(etch_depth + 0.1) {
            x_axis_etch_2d();
            y_axis_etch_2d();
        }
}

// =============== LEVEL 1 =====================================

module level1() {
    difference() {
        union() {
            // Hex plate + Atlas rear flange
            linear_extrude(L1_thickness) L1_plate_outline();
            // 4 hex-vertex brackets (V0, V1, V4, V5)
            hex_brackets();
            // 2 Atlas-side brackets (replacing V2, V3)
            atlas_side_brackets();
        }

        // Atlas M4 holes + counterbore from bottom.
        // v17f: the FRONT pair (bolt[0] > 0, closer to the dome
        // center) are slots running ±atlas_slot_play mm in X to
        // absorb print-tolerance error. The REAR pair (bolt[0] < 0,
        // on the rear flange) stays as round holes — they pin the
        // rotation once the front pair is in place.
        for (bolt = atlas_bolts) {
            if (bolt[0] > 0) {
                // Front pair → slots
                translate([bolt[0], bolt[1], -1])
                    linear_extrude(L1_thickness + 2)
                        atlas_slot_2d(atlas_bolt_dia);
                translate([bolt[0], bolt[1], -1])
                    linear_extrude(atlas_bolt_csink_depth + 1)
                        atlas_slot_2d(atlas_bolt_csink_dia);
            } else {
                // Rear pair → round (unchanged from v17e)
                translate([bolt[0], bolt[1], -1])
                    cylinder(d=atlas_bolt_dia, h=L1_thickness+2, $fn=32);
                translate([bolt[0], bolt[1], -1])
                    cylinder(d=atlas_bolt_csink_dia, h=atlas_bolt_csink_depth+1, $fn=32);
            }
        }

        // Camera mount 1/4"-20 insert pocket
        translate([0, 0, -1])
            cylinder(d=camera_thread_dia, h=camera_insert_depth+1, $fn=32);
        translate([0, 0, -1])
            cylinder(d=camera_thread_dia+2, h=2.5, $fn=32);

        // Bracket tap holes (12 total, 2 per bracket × 6 brackets).
        // v17h: suppressed entirely in unibody mode — when the dome
        // prints as one piece, there are no bolts to install through
        // the bracket tops, so the holes would just be stress risers.
        if (!unibody_mode) all_bracket_tap_holes();

        // Wiring slots
        for (angle = [0, 90, 180, 270]) {
            rotate([0, 0, angle])
                translate([15, 0, -1])
                    linear_extrude(L1_thickness+2)
                        rounded_rect(10, 14, 3);
        }

        // Axis etching on top surface
        axis_etching(L1_thickness);
    }
}

// =============== LEVEL 2 (LiDARs below, GNSS stand recess + top above) ===

module level2() {
    difference() {
        // Hex plate + Atlas rear flange (same shape as L1, v17b)
        linear_extrude(L2_thickness)
            L1_plate_outline();

        // === TOP SIDE FEATURES ===

        // GNSS magnetic stand recess — centering pocket at center (v15)
        translate([0, 0, L2_thickness - gnss_stand_recess_depth])
            cylinder(d=gnss_stand_recess_dia,
                     h=gnss_stand_recess_depth + 1, $fn=64);

        // Cable opening — triangular cutout on side 4
        // (replaces old ø30mm cable hole; carries 4× PoE + 1× GNSS cables)
        translate([0, 0, -1])
            linear_extrude(L2_thickness + 2)
                cable_opening_2d();

        // RGB camera mount holes (4 cameras × 4 M3 holes each = 16 holes)
        // RouteCAM_P_CU25_CXLC_IP67 bottom mount: 4× M3, 16.5 mm square.
        // v17h: each row is [global_x, global_y, aim_deg]. The mount
        // bolt pattern rotates with the camera body, so for the rear
        // pair (aim_deg ≠ 0) the holes in L2 align with the re-aimed
        // (look-at-center) camera orientation.
        for (cam = rgb_cam_layout) {
            translate([cam[0], cam[1], -1])
                rotate([0, 0, cam[2]])
                    for (off = rgb_cam_bolt_offsets) {
                        translate([off[0], off[1], 0])
                            cylinder(d=rgb_cam_bolt_dia,
                                     h=L2_thickness+2, $fn=32);
                    }
        }

        // Bracket bolt holes (12 total, clearance only — BHCS head on surface).
        // v17h: suppressed entirely in unibody mode (see level1() note).
        if (!unibody_mode)
            bolt_clearance_at(hex_bracket_bolt_positions, L2_thickness);

        // Axis etching on top surface
        axis_etching(L2_thickness);

        // === BOTTOM SIDE FEATURES ===

        // Robin W top mount M6 bolts (through plate, countersunk from TOP)
        for (angle = lidar_angles) {
            rotate([0, 0, angle]) {
                for (bolt = robin_top_bolts) {
                    // Through-hole
                    translate([lidar_ring_radius + bolt[0], bolt[1], -1])
                        cylinder(d=robin_top_bolt_dia, h=L2_thickness+2, $fn=32);
                    // Countersink from top
                    translate([lidar_ring_radius + bolt[0], bolt[1],
                               L2_thickness - robin_top_csink_depth])
                        cylinder(d=robin_top_csink_dia,
                                 h=robin_top_csink_depth+1, $fn=32);
                }
            }
        }
    }
}

// ===================== GHOST COMPONENTS ======================

module atlas_duo_ghost() {
    color("DarkBlue", 0.4)
        translate([datum_x+atlas_front_bolt_x - atlas_body_length+8,
                   datum_y - atlas_body_width+9,
                   L1_thickness])
            cube([atlas_body_length, atlas_body_width, atlas_body_height]);
}

module robin_w_ghost() {
    color("DarkGray", 0.3)
        translate([0, 0, -robin_body_h/2])
            cube([robin_body_d, robin_body_w, robin_body_h], center=true);
}

// Commercial magnetic GNSS stand ghost (base + pole + antenna)
gnss_stand_base_h     = 18;
gnss_stand_pole_dia   = 16;
gnss_stand_pole_h     = 100;

module gnss_stand_ghost() {
    color("DimGray", 0.6)
        translate([0, 0, L2_z_top - gnss_stand_recess_depth])
            cylinder(d=gnss_stand_base_dia, h=gnss_stand_base_h, $fn=64);
    color("Silver", 0.5)
        translate([0, 0, L2_z_top - gnss_stand_recess_depth + gnss_stand_base_h])
            cylinder(d=gnss_stand_pole_dia, h=gnss_stand_pole_h, $fn=32);
}

module gnss_antenna_ghost() {
    z = L2_z_top - gnss_stand_recess_depth + gnss_stand_base_h + gnss_stand_pole_h;
    color("White", 0.4) translate([0,0,z]) {
        cylinder(d1=30, d2=150, h=20, $fn=64);
        translate([0,0,20]) cylinder(d=150, h=50, $fn=64);
        translate([0,0,70]) cylinder(d1=150, d2=140, h=10, $fn=64);
    }
}

module camera_mount_ghost() {
    color("Gray", 0.3) translate([0,0,-45]) cylinder(d=35, h=45, $fn=32);
}

// RouteCAM_P_CU25_CXLC_IP67 ghost (body + lens protrusion)
// Camera sits on L2 top surface, facing radially outward (+X in local frame).
// Body is 46×46 cross-section, 52.95 depth (X). Lens adds ~12mm to front.
module rgb_camera_ghost() {
    // Camera body (facing +X, centered at mount bolt center)
    color("DarkGreen", 0.5) {
        // Main body: 52.95 deep × 46 wide × 46 tall
        translate([-rgb_cam_body_d/2, -rgb_cam_body_w/2, 0])
            cube([rgb_cam_body_d, rgb_cam_body_w, rgb_cam_body_w]);
        // Lens protrusion (cylindrical, front face)
        translate([rgb_cam_body_d/2, 0, rgb_cam_body_w/2])
            rotate([0, 90, 0])
                cylinder(d=25, h=rgb_cam_body_h_total - rgb_cam_body_d, $fn=32);
    }
}

// ===================== ASSEMBLY ==============================

module full_assembly() {
    // Level 1 + Atlas + camera
    color("RoyalBlue", 0.8) level1();
    atlas_duo_ghost();
    camera_mount_ghost();

    // Level 2 (translated to its Z position)
    translate([0, 0, L2_z_bottom]) {
        color("SteelBlue", 0.8) level2();

        // LiDARs hanging below L2 (top face at L2 bottom = z=0 in local)
        for (angle = lidar_angles)
            rotate([0, 0, angle])
                translate([lidar_ring_radius, 0, 0])
                    robin_w_ghost();
    }

    // RGB cameras on L2 top (4 cameras: front stereo pair + rear symmetric pair).
    // v17h: per-camera (global_x, global_y, aim_deg) from rgb_cam_layout.
    // Rear cameras get aim_away_from_origin so their lenses point back at the
    // dome center axis (X=Y=0). Front cameras keep aim = 0° (face +X).
    for (cam = rgb_cam_layout)
        translate([cam[0], cam[1], L2_z_top])
            rotate([0, 0, cam[2]])
                rgb_camera_ghost();

    // Commercial GNSS magnetic stand (glued into L2 recess)
    gnss_stand_ghost();
    gnss_antenna_ghost();
}

// ===================== OUTPUT ================================

if (RENDER_MODE == 0) full_assembly();
if (RENDER_MODE == 1) level1();
if (RENDER_MODE == 2) level2();

// ===================== VERIFICATION ==========================
//
// HEXAGONAL PLATE GEOMETRY (v17b):
//   Inradius (center to flat side): 130mm.
//   Circumradius (center to vertex): 150.1mm.
//   Vertex-to-vertex: 300.2mm < 305mm printer bed.              ✓
//   LiDARs on sides 1 (0°), 3 (120°), 5 (240°).
//   Blank sides 2 (60°), 4 (180°), 6 (300°).
//   Orientation: rotate(30) circle($fn=6) → flat side at 0°.    ✓
//
// L1 AND L2 PLATES (hex + flange, v17b):
//   Both plates share the same outline: hex + side-4 flange.
//   Hex: 260 × 300.2mm (flat-to-flat × vertex-to-vertex).
//   Flange at side 4: extends from x=-130 to x=-150, Y=±75.1mm.
//   Total bounding box: 280 × 300.2mm. Fits 305mm bed.          ✓
//   L1 thickness: 6mm.  L2 thickness: 12mm (doubled for GNSS).
//
// COAXIAL ALIGNMENT (all at X=0, Y=0):
//   Camera 1/4"-20:      (0, 0, bottom of L1)                   ✓
//   Atlas CoN:           (0, 0, 6mm)                             ✓
//   LiDAR ring center:   (0, 0, 139mm = L2_z_bottom)            ✓ (v17e)
//   GNSS stand recess:   (0, 0, L2_z_top - 8mm = 143mm)         ✓ (v17e)
//
// VERTICAL CLEARANCE (LiDARs below L2) — v17e:
//   LiDAR body bottom = L2_z_bottom - robin_body_h = 139 - 85 = 54mm
//   Atlas body top    = L1_thickness + atlas_body_height = 53.8mm
//   Gap               = 54 - 53.8 = 0.2mm  (no collision, tight) ✓
//   (v17d at pillar=130 gave −2.8mm overlap; v17c at 160 = 27.2mm.)
//
// BRACKET GEOMETRY (v17b — 4 hex-vertex + 2 Atlas-side):
//   HEX-VERTEX BRACKETS (V0, V1, V4, V5):
//     Wall 12mm thick × 35mm long along blank sides 2 and 6.
//     Height 160mm, flat extrusion, no overhangs.                 ✓
//     M6 tap wall: (12-5.5)/2 = 3.25mm each side.                ✓
//     LiDAR body clearance: ≥12.3mm.                              ✓
//   ATLAS-SIDE BRACKETS (replacing V2, V3):
//     Wall 10mm thick × 35mm long, running +X from x=-150 to -115.
//     Upper: y=[64, 74].  Lower: y=[-74, -64].                   ✓
//     Atlas body clearance: upper 7.2mm, lower 3.0mm.            ✓
//     LiDAR@120°/240° clearance: ≥10.2mm.                        ✓
//     M6 tap wall: (10-5.5)/2 = 2.25mm each side.                ✓
//     Height 160mm, flat extrusion, no overhangs.                 ✓
//   Total: 6 brackets, 12 bolts.                                  ✓
//
// BOLT ACCESS (v17b — 12mm L2):
//   Bracket ↔ L2: 6 brackets × 2 bolts = 12× M6×20mm BHCS.
//     Clearance-only through L2 (no countersink), head on surface.
//     M6×20mm: 12mm plate + 8mm engagement in 10mm tap holes.    ✓
//   LiDAR mount: 3 LiDARs × 4 bolts = 12× M6×16mm SHCS.
//     Countersunk through L2 into LiDAR top mount holes.
//     12mm plate - 4mm csink = 8mm pass-through + 8mm thread.    ✓
//   Atlas: 4× M4×10mm, front pair on hex, rear pair on flange.   ✓
//   GNSS: glued, no bolts.                                        ✓
//
// GNSS STAND RECESS (v17b — deeper recess):
//   ø86mm × 8mm centering pocket at L2 center (top side).
//   Floor thickness: 12 - 8 = 4mm.                               ✓
//   Nearest LiDAR countersink: wall = 12.5mm.                    ✓
//
// LIDAR RING SPACING:
//   Ring radius: 90mm. Body-to-body gap: 51.1mm.                 ✓
//   Outermost bolt at r=119mm. Countersink wall to hex: 5.5mm.   ✓
//
// CABLE OPENING (L2 only — teardrop, replacing ø30mm hole):
//   Hull of tip circle ø30mm at (-100, 0) + base rect at x=[-150,-140].
//   Base half-width ±60mm (half flange length, between brackets).
//   Triangle height 40mm (doubled from original 20mm).
//   Fits between Atlas-side brackets (inner edges at y=±64mm).
//   Wall to bracket inner edge: 4mm. Bracket bolts: ≥6.6mm gap.    ✓
//   GNSS recess clearance: 42mm. Camera holes: ≥37mm.              ✓
//   Carries 5 cables (4× PoE + 1× GNSS).
//   Width at tip circle: ø30mm, at x=-120: ~52mm, at x=-140: 120mm.✓
//   L1 plate remains SOLID — no cable opening on L1.                ✓
//
// RGB CAMERA PLACEMENT (RouteCAM_P_CU25_CXLC_IP67):
//   4 cameras total:
//     Front stereo pair — 2 cameras at 0° (alongside LiDAR@0°),
//       one at +Y (105, +52) and one at −Y (105, −52).
//       Stereo baseline = 2×52 = 104mm.
//     Rear symmetric pair — 1 camera at 120° +Y, 1 at 240° −Y.
//       Global positions: (−97.5, +64.9) and (−97.5, −64.9),
//       mirrored about the global X axis.
//   Bottom mount: 4× M3, 16.5mm square pattern (16 holes total).
//   M3 hole centers (local): (96.75,±43.75)…(113.25,±60.25).
//   Outermost hole at r=113.25mm. Wall to hex side: ≥15.1mm.      ✓
//   Nearest LiDAR countersink: 9.65mm c-t-c > 7.2mm required.    ✓
//   Nearest to GNSS recess (r=43mm): 94.2mm >> 43mm.              ✓
//   All 16 M3 holes inside hex plate at all positions.              ✓
//   Front stereo pair inter-camera clearance: 87.5mm c-t-c.        ✓
//   Camera body 46×46mm overhangs hex edge by ~2mm at angle=0°
//     (front face at r=128 vs hex at r=130) — cosmetic only,
//     mount holes have >15mm structural wall to hex edge.           ✓
//   FOV: 158° DFOV, 134° HFOV parallel to LiDAR 120° HFOV.
//   Camera and LiDAR on same side → maximum FOV overlap.           ✓
//   Camera on L2 top, LiDAR below L2 — no Z-axis conflict.        ✓
//
// FOV CLEARANCE:
//   V0, V1, V4, V5 brackets on blank sides 2, 6 — no occlusion.  ✓
//   Atlas-side brackets at x=[-150,-115] are behind LiDARs@120°
//   and @240°, in the rear zone between the two LiDAR bodies.
//   Not in any LiDAR's primary scanning direction.                 ✓
//
// PRINT NOTES (v17e):
//   L1: 280×300×139mm (hex+flange, 4 vertex + 2 atlas-side
//       bracket walls up — all flat extrusions, no support)        ✓
//       (v17c=166mm, v17d=136mm, v17e=139mm.)
//   L2: 280×300×12mm  (hex+flange plate — trivial)                ✓
//   2 printed parts. Both fit Raise3D Pro2 (305×305×300mm).       ✓
//
// BOM (v17b):
//  12× M6×20mm BHCS — bracket ↔ L2 (clearance, head on surface)
//  12× M6×16mm SHCS — LiDAR top mount (3 LiDARs × 4 bolts)
//   4× M4×10mm      — Atlas Duo ↔ L1
//  16× M3×12mm SHCS — RGB cameras (4 per camera × 4 cameras,
//                      through 12mm L2 into camera bottom threads)
//   1× 1/4"-20 brass insert — camera mount
//   1× commercial magnetic GNSS stand
//   Epoxy or construction adhesive for GNSS stand bond.
//   Total M6: 24 (12× BHCS + 12× SHCS).  Total M4: 4.  Total M3: 16.
//
// BRACKET INSIDE-CORNER FILLETS (v17c fix):
//   Problem: sharp 90° vertical corners where bracket walls meet
//     the plate. Need smooth transitions on ALL inside corners.
//   Fix — hull()-based fillet at each bracket base. Fillet base
//     profile is expanded on every side that has an inside corner
//     (i.e. where the plate extends beyond the bracket wall).
//   HEX-VERTEX BRACKETS: expanded on 3 sides (fillet_r/5 = 2mm):
//     - Inner face (toward hex center): +2mm along normal.
//     - Both short ends (along wall direction): +2mm each.
//     - Outer face (along hex side): NO expansion (at plate edge).
//     Both fillet and wall are intersection()-clipped to L1_plate_outline
//     so nothing hangs beyond the rounded hex + flange boundary.    ✓
//   ATLAS-SIDE BRACKETS: expanded on 2 sides:
//     - +X face (toward hex center): +10mm. x=-115 → x=-105.
//     - Atlas-facing face: limited expansion to preserve clearance.
//       Upper bracket: +5mm (leaves 2.2mm to Atlas body).           ✓
//       Lower bracket: +2mm (leaves 1.0mm to Atlas body).           ✓
//     - -X face (x=-150, flange edge): 0 — no plate beyond.
//     - Outward face: 0 — at/near plate edge.
//     Both fillet and wall are intersection()-clipped to L1_plate_outline
//     so nothing hangs beyond the rounded flange corners.           ✓
//   No effect on bracket bolt positions (at 30%/70% along wall,
//     well above fillet height). No effect on tap hole depth.       ✓
// =============================================================
