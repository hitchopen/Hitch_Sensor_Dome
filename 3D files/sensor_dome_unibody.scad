// =============================================================
// sensor_dome_unibody.scad
//
// SINGLE-PIECE (unibody) VARIANT of the Hitch Sensor Dome.
//
// Use this file when you want to print L1, the 6 brackets, and
// L2 as ONE FDM part, instead of the default two-piece bolted
// assembly produced by sensor_dome.scad.
//
// -------------------------------------------------------------
// HOW IT WORKS
// -------------------------------------------------------------
// This file `include`s sensor_dome.scad — every parameter,
// module, sensor mount pattern, fillet, and clearance comes
// from there. The wrapper just:
//   1. Suppresses the parent's two-piece render switch.
//   2. Adds a `unibody()` module that does
//          union() { level1(); translate(...) level2(); }
//      Because the brackets in level1() extrude to exactly
//      z = L2_z_bottom (= L1_thickness + L1_pillar_height), the
//      bracket tops meet L2's underside seamlessly and the
//      union fuses into one watertight body.
//
// To change pillar height, plate thickness, sensor positions,
// fillets, etc., EDIT sensor_dome.scad and re-render this file.
// Do not redeclare those constants here — there is no copy to
// keep in sync.
//
// -------------------------------------------------------------
// LEFTOVER BRACKET BOLT HOLES
// -------------------------------------------------------------
// The two-piece design needs 12× M6×20 BHCS bolts to join L2
// to the bracket tops, so sensor_dome.scad cuts:
//   - 12 tap pockets (ø5.5 × 10 mm deep) at the top of each
//     bracket — see all_bracket_tap_holes() in level1().
//   - 12 clearance through-holes (ø6.5 × 12 mm) through L2 —
//     see bolt_clearance_at(...) in level2().
//
// When you merge the two parts, those holes remain in place
// and align into a ~22 mm vertical channel at each junction.
// They are:
//   - HARMLESS to strength. Each hole is ≤ 4% of the 12×35 mm
//     bracket wall cross-section.
//   - USEFUL as vent holes. They let trapped air escape during
//     printing (avoids blistering between L2 underside and
//     bracket-top print interface).
//   - HIDDEN once you set a GNSS stand and cameras on L2.
//
// If you'd rather have a completely solid join, set
// `fill_bracket_bolt_holes = true` below — that turns on a
// back-fill cylinder set that plugs every hole with PETG.
//
// -------------------------------------------------------------
// PRINT NOTES
// -------------------------------------------------------------
// Bounding box: 280 × 300 × 151 mm. Fits Raise3D Pro2 (305^3)
// in the default L1-down orientation.
//
// In L1-down orientation the brackets are vertical walls
// (good for FDM, no overhangs on them). The L2 plate (12 mm
// slab at z = 139–151) cantilevers between bracket tops over
// a mostly-empty 280 × 300 area and REQUIRES SUPPORT MATERIAL
// under every region of L2 that is not directly above a
// bracket.
//
// Recommended slicer settings:
//   - Tree / organic supports (Bambu, PrusaSlicer 2.6+, Cura
//     "Tree Support"). Avoid grid supports — wasteful here.
//   - Support interface layers (top z-distance ~0.2 mm) so
//     L2's bottom face is presentable.
//   - Support roof only above L2's underside; no support
//     between bracket walls below L2.
//   - Expect ~1.5–2 kg of support material in PETG.
// Print time ~28–36 h depending on nozzle / layer / infill.
// Soluble PVA/HIPS is impractical at this height.
//
// Alternatives:
//   - Print L2-down (upside-down). Same support problem flipped,
//     but the GNSS recess pocket becomes a hat that prints
//     poorly. Not recommended.
//   - Diagonal print orientation (e.g. 30°). Reduces L2 support
//     volume but adds bracket overhang. Print at your own risk.
//
// -------------------------------------------------------------
// BOM DELTA vs two-piece (v17e)
// -------------------------------------------------------------
//   Removed:  12× M6×20 mm BHCS  (bracket-to-L2 bolts)
//   Unchanged:
//     12× M6×16 mm SHCS  — LiDAR top mount (3 LiDARs × 4)
//      4× M4×10 mm        — Atlas Duo to L1
//     16× M3×12 mm SHCS  — RGB cameras (4 × 4)
//      1× 1/4"-20 brass insert — camera mount under L1
//      1× commercial magnetic GNSS stand
//     Epoxy / construction adhesive for GNSS stand bond.
// =============================================================

include <sensor_dome.scad>;

// Override the parent file's RENDER_MODE so its
//     if (RENDER_MODE == 0) full_assembly();
//     if (RENDER_MODE == 1) level1();
//     if (RENDER_MODE == 2) level2();
// switch produces nothing — we render our own thing below.
// (OpenSCAD top-level variable assignments are late-bound,
// so this override takes effect inside the parent's if-blocks
// even though the assignment appears after `include`.)
RENDER_MODE = -1;

// -------------------------------------------------------------
// Local toggle: solid-fill the 12 leftover bracket bolt holes.
// Off by default — see "LEFTOVER BRACKET BOLT HOLES" above.
// -------------------------------------------------------------
fill_bracket_bolt_holes = false;

// Back-fill cylinders sized slightly larger than the original
// bolt holes (0.3 mm radial overlap) so the boolean union
// closes cleanly with no z-fighting at the slicer.
module _bracket_bolt_back_fills() {
    for (pos = hex_bracket_bolt_positions) {
        // Fill L1 tap pocket at bracket top (ø5.5 × 10 mm).
        translate([pos[0], pos[1],
                   plate_thickness + L1_pillar_height
                       - interlevel_tap_depth - 0.1])
            cylinder(d = interlevel_tap_dia + 0.3,
                     h = interlevel_tap_depth + 0.2,
                     $fn = 32);
        // Fill L2 clearance through-hole (ø6.5 × 12 mm).
        translate([pos[0], pos[1], L2_z_bottom - 0.1])
            cylinder(d = interlevel_bolt_dia + 0.3,
                     h = L2_thickness + 0.2,
                     $fn = 32);
    }
}

// -------------------------------------------------------------
// The unibody. This is the model you export as STL.
// -------------------------------------------------------------
module unibody() {
    union() {
        // L1 plate + 6 brackets (4 hex-vertex + 2 atlas-side).
        // The brackets in level1() extrude up to exactly
        // z = plate_thickness + L1_pillar_height = L2_z_bottom,
        // so their tops sit at L2's underside.
        level1();

        // L2 plate, translated up onto the bracket tops.
        translate([0, 0, L2_z_bottom])
            level2();

        // Optional: plug the 12 leftover bracket bolt holes.
        if (fill_bracket_bolt_holes)
            _bracket_bolt_back_fills();
    }
}

// =============================================================
// OUTPUT
// =============================================================
unibody();
