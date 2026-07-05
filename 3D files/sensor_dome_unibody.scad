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
//   1. Sets `unibody_mode = true` so the parent's level1() /
//      level2() SKIP drilling the 12 bracket bolt holes that
//      only exist to bolt the two pieces together. In the
//      unibody print there are no bolts to install at those
//      positions, so the holes would just be stress risers.
//   2. Suppresses the parent's two-piece render switch.
//   3. Adds a `unibody()` module that does
//          union() { level1(); translate(...) level2(); }
//      Because the brackets in level1() extrude to exactly
//      z = L2_z_bottom (= L1_thickness + L1_pillar_height), the
//      bracket tops meet L2's underside seamlessly and the
//      union fuses into one watertight body. With unibody_mode
//      = true the tops are solid (no tap pockets) and the L2
//      plate is solid above them (no clearance through-holes)
//      so the join is fully sealed plastic.
//
// To change pillar height, plate thickness, sensor positions,
// fillets, etc., EDIT sensor_dome.scad and re-render this file.
// Do not redeclare those constants here — there is no copy to
// keep in sync.
//
// -------------------------------------------------------------
// WHAT'S DIFFERENT FROM THE TWO-PIECE PRINT
// -------------------------------------------------------------
// Beyond fusing L1 and L2, the unibody also LACKS:
//   - 10 × ø5.5 tap pockets (15 mm deep) at the top of the brackets
//     (v17q: was 12 × 10 mm-deep pockets at v17e–v17p). In two-piece
//     mode the M6 BHCS bolts thread into these. In unibody mode there
//     are no bolts → no pockets.
//   - 10 × ø6.5 clearance through-holes in L2 above the
//     bracket tops. In two-piece mode the M6 BHCS heads pass
//     through these. In unibody mode → no holes.
//
// Net BOM reduction vs two-piece (v17q):
//   Removed:  10× M6×25 mm BHCS  (bracket-to-L2 bolts)
//             — 4 along the V0-V1 side wall, 4 along the V4-V5 side
//               wall, 1 on the V2 column (Atlas-upper), 1 on the V3
//               column (Atlas-lower). Was 12× M6×20 at v17b–v17p.
//   Unchanged:
//     12× M6×16 mm SHCS  — LiDAR top mount (3 LiDARs × 4)
//      4× M4×10 mm        — Atlas Duo to L1 (front pair use slots)
//     16× M3×12 mm SHCS  — RGB cameras (4 × 4)
//      1× 1/4"-20 brass insert — camera mount under L1
//      1× commercial magnetic GNSS stand
//     Epoxy / construction adhesive for GNSS stand bond.
//
// -------------------------------------------------------------
// PRINT NOTES
// -------------------------------------------------------------
// Bounding box: 280 × 300 × 151 mm. Fits Raise3D Pro2.
//
// In L1-down orientation the brackets are vertical walls (good
// for FDM, no overhangs on them). The L2 plate (12 mm slab at
// z = 139–151) cantilevers between bracket tops over a mostly-
// empty 280 × 300 area and REQUIRES SUPPORT MATERIAL under
// every region of L2 that is not directly above a bracket.
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
// =============================================================

include <sensor_dome.scad>;

// v17h: tell the parent's level1()/level2() to skip the bracket
// bolt holes entirely (no tap pockets in L1, no clearance holes
// in L2). With this true, the L1 bracket tops fuse to the L2
// underside as fully solid plastic — no leftover vent holes.
unibody_mode = true;

// Override the parent file's RENDER_MODE so its
//     if (RENDER_MODE == 0) full_assembly();
//     if (RENDER_MODE == 1) level1();
//     if (RENDER_MODE == 2) level2();
// switch produces nothing — we render our own thing below.
// (OpenSCAD top-level variable assignments are late-bound, so
// this override takes effect inside the parent's if-blocks even
// though the assignment appears after `include`.)
RENDER_MODE = -1;

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
    }
}

// =============================================================
// OUTPUT
// =============================================================
unibody();
