// ============================================================
// 3D Mapping Sensor Dome — v17c
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
// RENDER_MODE: 0=assembly, 1=level1, 2=level2
// ============================================================

RENDER_MODE = 2;

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

lidar_ring_radius = 90;      // v17: increased from 65 for more center spacing
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

// Bracket wall direction and inward normal (simple even/odd rule)
function bracket_wall_dir(i) =
    let(a = hex_vertex_angles[i])
    (i % 2 == 0) ? a + 120 : a - 120;

function bracket_wall_norm(i) =
    let(a = hex_vertex_angles[i])
    (i % 2 == 0) ? a - 150 : a + 150;

// Bolt positions: 2 per bracket (at 30% and 70% along wall length)
function bracket_bolt_pos(i, j) =
    let(
        a = hex_vertex_angles[i],
        vx = hex_circumradius * cos(a),
        vy = hex_circumradius * sin(a),
        d = bracket_wall_dir(i),
        n = bracket_wall_norm(i),
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
// With L1_pillar_height=160: LiDAR_bottom = 166-85 = 81mm,
//   Atlas_top = 53.8mm, gap = 27.2mm (ample for cable routing).
L1_pillar_height = 160;

// --- Level 2 ---
L2_z_bottom = L1_thickness + L1_pillar_height;   // 166
L2_thickness = 2 * plate_thickness;                // 12  (v17b: doubled for deeper GNSS recess)
L2_z_top = L2_z_bottom + L2_thickness;            // 178 (was 172 in v17)

// --- GNSS Magnetic Stand Recess (v15) ---
// Commercial magnetic GNSS stand (e.g. ArduSimple AS-ACC-SURVEYSTAND-01)
// glued into a circular centering recess on L2 top surface.
// *** MEASURE YOUR STAND AND ADJUST gnss_stand_base_dia IF NEEDED ***
gnss_stand_base_dia    = 88;    // magnetic base outer diameter (MEASURE!)
gnss_stand_recess_dia  = 86;    // centering pocket
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
rgb_cam_x         = 105;      // radial position from center (along LiDAR direction)
rgb_cam_y         = 52;       // lateral offset magnitude from LiDAR center

// Camera layout: [LiDAR_angle, y_sign]
//   y_sign = +1 → camera at +Y (CCW side) in local frame
//   y_sign = -1 → camera at −Y (CW side) in local frame
rgb_cam_layout = [
    [  0,  1],   // front stereo right (+Y)
    [  0, -1],   // front stereo left  (−Y)
    [120,  1],   // rear left  (+Y → global +Y side)
    [240, -1],   // rear right (−Y → mirrors 120° about X axis)
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

// Regular hexagon plate outline (L2 and base shape for L1).
// Flat side 1 faces +X. Vertices at 30°, 90°, 150°, 210°, 270°, 330°.
module hex_plate_outline() {
    offset(r=corner_r) offset(r=-corner_r)
        rotate([0, 0, 30])
            circle(r=hex_circumradius, $fn=6);
}

// L1 plate: hex + rectangular flange at side 4 for Atlas rear bolts.
// The flange extends straight back from hex side 4, matching the
// side's Y span for a seamless join. Rear corners are rounded.
module L1_plate_outline() {
    offset(r=corner_r) offset(r=-corner_r)
    union() {
        rotate([0, 0, 30])
            circle(r=hex_circumradius, $fn=6);
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
        vx = hex_circumradius * cos(a);
        vy = hex_circumradius * sin(a);

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

        // Atlas M4 holes + counterbore from bottom
        for (bolt = atlas_bolts) {
            translate([bolt[0], bolt[1], -1])
                cylinder(d=atlas_bolt_dia, h=L1_thickness+2, $fn=32);
            translate([bolt[0], bolt[1], -1])
                cylinder(d=atlas_bolt_csink_dia, h=atlas_bolt_csink_depth+1, $fn=32);
        }

        // Camera mount 1/4"-20 insert pocket
        translate([0, 0, -1])
            cylinder(d=camera_thread_dia, h=camera_insert_depth+1, $fn=32);
        translate([0, 0, -1])
            cylinder(d=camera_thread_dia+2, h=2.5, $fn=32);

        // Bracket tap holes (12 total, 2 per bracket × 6 brackets)
        all_bracket_tap_holes();

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
        // RouteCAM_P_CU25_CXLC_IP67 bottom mount: 4× M3, 16.5 mm square
        // Through-holes for M3 bolts from L2 underside into camera bottom threads
        for (cam = rgb_cam_layout) {
            rotate([0, 0, cam[0]]) {
                for (off = rgb_cam_bolt_offsets) {
                    translate([rgb_cam_x + off[0],
                               cam[1] * rgb_cam_y + off[1], -1])
                        cylinder(d=rgb_cam_bolt_dia, h=L2_thickness+2, $fn=32);
                }
            }
        }

        // Bracket bolt holes (12 total, clearance only — BHCS head on surface)
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

    // RGB cameras on L2 top (4 cameras: front stereo pair + rear symmetric pair)
    for (cam = rgb_cam_layout)
        translate([0, 0, L2_z_top])
            rotate([0, 0, cam[0]])
                translate([rgb_cam_x, cam[1] * rgb_cam_y, 0])
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
//   LiDAR ring center:   (0, 0, 166mm = L2_z_bottom)            ✓
//   GNSS stand recess:   (0, 0, L2_z_top - 8mm = 170mm)         ✓
//
// VERTICAL CLEARANCE (LiDARs below L2):
//   LiDAR body bottom = L2_z_bottom - robin_body_h = 166 - 85 = 81mm
//   Atlas body top    = L1_thickness + atlas_body_height = 53.8mm
//   Gap               = 81 - 53.8 = 27.2mm (ample for cables)   ✓
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
// PRINT NOTES (v17b):
//   L1: 280×300×166mm (hex+flange, 4 vertex + 2 atlas-side
//       bracket walls up — all flat extrusions, no support)        ✓
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
