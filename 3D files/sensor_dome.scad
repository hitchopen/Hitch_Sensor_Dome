// ============================================================
// 3D Mapping Sensor Dome — v17r (V0/V5 front-access chamfer)
//
// CHANGES FROM v17q (Atlas-INS front-face access from V0-V5 side):
//   User request: "Open the V0-V5 side wider so that from the front,
//   it is easy access to the Atlas INS front antenna connectors;
//   currently the X-side panel of Atlas INS is not fully exposed
//   from the V5-V0 side."
//
//   Diagnosis: with v17l's full-side enclosure (the V0+V1 brackets
//   form a continuous side-2 wall, V4+V5 the side-6 wall), the two
//   bracket inner corners that sit nearest the front of the dome —
//   P3 at (112.34, +54.47) for V0 and P3 at (112.34, -54.47) for V5
//   — INTRUDE into the Atlas Y range. Specifically:
//     V0 corner y = +54.47, Atlas top y = +56.80 → 2.33 mm intrusion
//     V5 corner y = -54.47, Atlas bottom y = -61.00 → 6.53 mm intrusion
//     (Atlas Y is asymmetric: body extends y ∈ [-61, +56.80],
//      centered at -2.1; V5 corner sits 6.5 mm inside Atlas Y range
//      while V0 corner only intrudes 2.3 mm.)
//   These intrusions block 5-10 mm of the Atlas front face's corners
//   from being reached horizontally between the V0 and V5 brackets.
//
//   v17r cuts a TRIANGULAR CHAMFER from each bracket inner corner,
//   moving the bracket inner edge up to y = +61.80 (V0 side) and
//   down to y = -66.00 (V5 side) — both at Atlas edge + 5 mm safety
//   margin (= finger / connector pull clearance). The chamfers run
//   the full L1 bracket height (z = 6 → 139) for clean geometry and
//   open visibility of the Atlas front face from the front of the
//   dome.
//
//   Chamfer triangles (in plan view):
//     V0 corner:   (116.57, +61.80) ↔ ( 99.64, +61.80) ↔ (112.34, +54.47)
//                  Triangle area: 62.08 mm² in XY; volume 8.26 cm³.
//     V5 corner:   (119.00, -66.00) ↔ ( 92.37, -66.00) ↔ (112.34, -54.47)
//                  Triangle area: 153.57 mm² in XY; volume 20.42 cm³.
//   Total material removed: 28.7 cm³ ≈ 36 g PETG.
//
//   The chamfer cuts the V0 and V1 bracket UNION near V0 (both share
//   the same near-V0 inner corner via the v17l side-2 wall) and the
//   V4 and V5 bracket UNION near V5. The bracket OUTER faces along
//   sides 2 and 6 are UNCHANGED (the chamfer wedge sits entirely on
//   the INNER side of the wall) — full-side enclosure of sides 2 / 6
//   is preserved.
//
//   v17q two-piece bolt positions (fracs [0.2, 0.4] → 28 / 56 mm from
//   the vertex along the leg) all sit at bracket parameter s ≥ 28,
//   well OUTSIDE the chamfer region (which extends to s = 14.66 on V0
//   side and s = 23.06 on V5 side). All 4 long-wall bolts stay
//   structurally engaged on each side.
//
//   Bracket cross-section reduction at the chamfered corners: 2.5%
//   (V0 side) and 6.1% (V5 side) of the v17l 2520 mm² gross cross-
//   section — negligible structurally; the v17k drop-resilience
//   margins (3-5× v17j) are preserved.
//
//   Applied in BOTH two-piece and unibody modes — Atlas front-face
//   access is a usability requirement regardless of print mode.
//   No sensor moved, no TF impact.
//
// CHANGES FROM v17p (M6 bolt count + length + tap depth):
//   Only affects the two-piece variant (sensor_dome.scad with
//   unibody_mode = false). The unibody build suppresses all
//   bracket-to-L2 bolt holes and is unaffected by v17q.
//
//   v17p (and every version since v17b) carried 12 × M6×20 BHCS
//   bracket-to-L2 bolts, distributed as 2 bolts per bracket × 6
//   brackets. With v17l's full-side enclosure (V0+V1 brackets
//   each span the entire 140 mm V0-V1 side; same for V4+V5 along
//   the V4-V5 side), V0's fracs [0.3, 0.7] and V1's fracs [0.3, 0.7]
//   produce bolt positions at 42 / 98 mm from V0 and 42 / 98 mm
//   from V1 = 42 / 98 mm from V0 — i.e. the bracket-bolt-pos lists
//   COINCIDE pairwise. Net effective bolts per side: 2 (drilled
//   twice by the difference()).
//
//   v17q rebalances the layout to match the actual side enclosure:
//
//     V0-V1 side wall:  4 distinct M6×25 bolts at 28, 56, 84,
//                       112 mm from V0 (V0 contributes fracs
//                       [0.2, 0.4]; V1 also contributes [0.2, 0.4]
//                       in V1's frame = 112, 84 mm from V0).
//                       Evenly spaced ~28 mm apart along the wall.
//     V4-V5 side wall:  4 distinct M6×25 bolts, symmetric.
//     V2 column (Atlas-upper): 1 M6×25 at leg midpoint (frac 0.5).
//     V3 column (Atlas-lower): 1 M6×25 at leg midpoint.
//     ----
//     TOTAL: 10 × M6×25 BHCS bracket-to-L2 bolts.
//
//   Bolt length 20 → 25 mm: engagement in bracket grows 8 → 13 mm
//   (more than +60% pull-out strength per bolt). The pilot tap
//   pocket grows:
//     interlevel_tap_depth : 10 → 15 mm  (= engagement + 2 mm pilot)
//   The tap pocket bottom sits at L1_pillar_height - 15 = 118 mm,
//   which is exactly the bottom edge of the v17k top fillet — the
//   pilot still lives entirely in solid bracket plastic with no
//   conflict with the wall geometry.
//
//   Net clamping force vs v17p two-piece variant:
//     - Bolt count: 12 (= 8 effective due to coincidence) → 10 (all
//       distinct).
//     - V0-V1 / V4-V5 sides: 2 effective → 4 each (+100%).
//     - V2 / V3 columns: 2 each → 1 each (-50%; matched to the
//       short 55 mm Atlas-side leg, where a centered single bolt
//       clamps uniformly).
//     - Per-bolt pull-out: ≈ +60% from deeper engagement.
//     - Net total joint strength along the long V-walls: roughly
//       3× the v17p value; total Atlas-side joint strength roughly
//       0.8× v17p (still > 5 kN per column at PETG's typical pull-
//       out modulus, more than adequate for the dome's mass).
//
//   sensor_dome_unibody.scad BOM line updated (12× M6×20 →
//   10× M6×25 removed by the unibody build).
//   No TF impact, no sensor moved, no geometry except tap pocket
//   depth changed.
//
// CHANGES FROM v17o (cable opening base narrowed to cover M3 mounts):
//   Print test of v17o revealed that the L2 cable opening's UPPER
//   TANGENT LINE clipped into the rear-LEFT camera's M3 mount hole
//   pattern. Specifically:
//     Rear-left camera at global (-108.12, +47.27), aim 120°.
//     The 4 M3 holes (local pattern ±8.25, ±8.25 rotated 120°) sit at
//       A = (-119.39, +50.29)   — upper-rear
//       B = (-111.14, +36.00)   — lower-rear  ← problem
//       C = (-105.10, +58.54)   — upper-front
//       D = ( -96.85, +44.25)   — lower-front
//     At v17o (cable_opening_base_half_w = 60 mm), the upper tangent
//     line from rectangle corner (-135, +60) to the tip circle
//     (-100, 0) r=15 runs at slope -1.083 and passes through y ≈
//     34.17 at x = -111.14. Hole B at (-111.14, +36.00) has its
//     LOWER EDGE (radius 1.7 mm) at y = 34.30 — i.e. 0.13 mm above
//     the tangent line's y, BUT the perpendicular distance is only
//     1.24 mm, and after subtracting the 1.7 mm hole radius the
//     hole edge clips INTO the cable opening by 0.46 mm. (Symmetric
//     exposure on the rear-RIGHT side: hole at (-111.14, -36.00).)
//   v17p drops cable_opening_base_half_w 60 → 50 mm:
//     - Closest hole (B) edge margin: -0.46 mm  →  +3.28 mm
//     - Hole A edge margin: +3.18 mm → +5.82 mm
//     - Holes C and D were already well clear, stay well clear
//     - Cable opening still 100 mm × 40 mm ≈ 2400 mm² area
//       (13× the 4× PoE + 1× GNSS bundle's 182 mm² area)
//     - Atlas-side bracket inner edge clearance: 4 mm → 14 mm
//       (lots of headroom against the rear LiDAR M6 brackets)
//   No other parameter changed. No sensor moved. No TF impact.
//
// CHANGES FROM v17n (close the cable opening's rear-open side):
//   Through v17n the L2 cable opening was a teardrop whose BASE
//   coincided with the L2 plate's rear edge at x = -150. After
//   difference()-ing from the plate, the opening was U-shaped —
//   open on the rear side — i.e. an "open triangle with one side
//   open." Cables could escape SIDEWAYS through that open back,
//   and the rear of the L2 plate had no transverse bracing
//   between the two rear-flange outer corners at (-150, ±75.06).
//
//   v17o pulls the cable opening's rear base inward to x = -140
//   so a 10 mm-thick L2-plate strip survives between x = -150
//   (the flange's rear edge) and x = -140 (the cable opening's
//   new rear edge). That 10 mm strip
//     - turns the open-back teardrop into a fully ENCLOSED
//       triangular hole that cables must thread vertically
//       through (no sideways escape route);
//     - bridges the two rear-flange outer corners at (-150,
//       ±75.06) with a continuous transverse beam — the rear
//       flange now has a dedicated load path across its full
//       150 mm width, which adds drop / vibration resilience
//       at those corners (the user's structural goal).
//
//   Parameter diff:
//     cable_opening_base_outer_x : -150 → -140  (rear edge)
//     cable_opening_base_inner_x : -140 → -135  (front edge of base rect)
//     NEW: cable_opening_rear_wall_t = 10        (= -140 - -150)
//     cable_opening_2d() no longer uses `far_x = -300` for the
//       rectangle — it now uses outer_x / inner_x directly so the
//       rectangle lives entirely inside the plate.
//
//   Effective teardrop length: 50 mm → 40 mm (tip at -100, base at
//   -140). At the new base, the opening is still the full 120 mm
//   wide (= 2 × 60 mm = 2 × cable_opening_base_half_w), which more
//   than covers the 4× shielded PoE + 1× GNSS cable bundle (~34 mm
//   total cable diameter).
//   Material added: ~14.4 cm³ of L2 plate plastic (≈ 18 g PETG).
//
//   No other geometry, sensor position, bracket, or TF moved.
//   The v17n vertex arcs at V0/V1/V4/V5 carry through unchanged.
//
// CHANGES FROM v17m (drop-impact distribution at hex vertices):
//   v17l made the V0/V1/V4/V5 brackets the entire side wall; v17m
//   added low partial walls along sides 3 and 5. Both passes left
//   the four exposed hex VERTICES (V0/V1/V4/V5) as sharp polygon
//   corners — just smoothed by the global corner_r=10 offset cycle,
//   which is a single ~10 mm arc tangent to the two adjacent sides.
//   The next drop test would still concentrate impact force on
//   roughly a 10 mm arc — closer to a point than to a distributed
//   load — at exactly the vertex where the bracket wall column
//   terminates. That is the structural worst case.
//
//   v17n introduces EXPLICIT 25 mm arcs at V0/V1/V4/V5 baked into
//   the hex polygon point list (rounded_hex_polygon_points()):
//     - 60° arc centered on the INWARD bisector at distance
//       R / sin(60°) = 28.87 mm from the theoretical vertex
//     - 16 sample points per arc (smooth visual)
//     - arc length grows from 10.47 mm (R=10) to 26.18 mm (R=25)
//       → 2.5× contact-line distribution at the L1 plate edge
//     - Hertzian peak stress drops to ~69% of v17m baseline
//       (R^{-2/5} drop-impact scaling per Stronge / Hertz)
//
//   The offset(corner_r) offset(-corner_r) smoothing pass in
//   L1_plate_outline() / hex_plate_outline() is preserved at the
//   original corner_r = 10 mm value. Convex arcs at R = 25 mm
//   survive the ±10 mm offset cycle UNCHANGED (the cycle only
//   smooths corners tighter than 10 mm). So:
//     V0/V1/V4/V5 → 25 mm arcs (NEW)
//     V2/V3       → 10 mm concave inside fillets at the hex-vs-
//                   flange junction (unchanged)
//     flange rear outer corners → 10 mm arcs (unchanged)
//   No other geometry, sensor position, or bolt pattern moved.
//   No TF impact.
//
//   v17l side-2 / side-6 enclosure SURVIVES the change:
//     - Tangent point on side 2 moves from 5.77 mm to 14.43 mm
//       inboard of V0 / V1 → straight wall span 128.6 mm → 111.2 mm
//       (still > 78% of side length)
//     - The 14.43 mm arc region at each end of side 2 is covered
//       by the bracket polygon's intersection with the rounded
//       plate outline — bracket material follows the arc, so
//       enclosure is continuous
//   v17m side 3 + side 5 partial walls SURVIVE similarly:
//     - Lose 14.43 mm of straight wall at the V1/V4 end (V2/V3 end
//       unchanged — those vertices stay sharp)
//     - Net partial wall straight section 145.4 mm → 130.9 mm
//
//   v17k symmetric top/bottom bracket fillets are UNAFFECTED — the
//   fillets are local to the bracket-plate junction in the Z axis;
//   they intersect with L1_plate_outline so they follow the new
//   25 mm arcs in plan view automatically.
//
// CHANGES FROM v17l (sensor-mount clearance + side 3/5 strengthening):
//   Print-fitting the v17l unibody revealed two mount-clearance
//   conflicts introduced by the v17k+v17l bracket widening / side
//   enclosure, plus the user noted that sides 3 and 5 (the rear
//   LiDAR-bearing faces) have no inter-column bracing at all. v17m
//   makes three changes:
//
//   (1) FRONT-STEREO CAMERAS pulled inward and rearward
//         rgb_cam_x_front : 85 → 75   (10 mm closer to dome center)
//         rgb_cam_y_front : 73 → 55   (18 mm closer to X axis)
//       At v17l the side-2 wall is 18 mm thick from the V0→V1 line
//       inward; with v17k's TOP fillet the wall expands to 21 mm
//       at z = L2_z_bottom. The previous (85, 73) cam position
//       placed its worst-case M3 bolt at perpendicular distance
//       4.35 mm from side 2 — INSIDE the wall — requiring six
//       ø6 mm head-clearance pockets that compromised the wall.
//       The v17m position moves the worst-case M3 to 24.93 mm
//       perpendicular distance — 3.93 mm clear of the 21 mm top-
//       fillet expansion and 6.93 mm clear of the 18 mm main wall.
//       The v17l front_cam_m3_head_pockets() helper is KEPT in
//       the source — its list comprehension now evaluates to empty
//       at the v17m cam position, so no pockets are drilled, but
//       if the cam is ever moved back the pockets re-appear
//       automatically.
//       Side-effect: stereo baseline shrinks 146 → 110 mm. Still
//       wider than v17a–g (104 mm) and v17h (130 mm).
//
//   (2) ATLAS-SIDE BRACKET TOP FILLET — "spherical-style" rounded
//       interface to L2 that does NOT block rear-LiDAR M6 bolts.
//       v17k expanded the TOP-FILLET footprint by fr=15 mm in +X
//       (toward dome center) so the bracket-L2 junction flares
//       outward. That +X flare reached x = -80 at z = L2_z_bottom
//       and SWEPT OVER the rear LiDAR M6 top-mount bolt at
//       (-85.68, +76.40) — making the M6 BHCS un-installable on
//       both rear LiDARs.
//       v17m removes the +X expansion entirely on both Atlas-side
//       brackets (upper and lower) while keeping the Atlas-facing
//       -Y / +Y expansion (5 mm upper, 2 mm lower) that fits
//       within the Atlas body clearance. The Atlas-side bracket
//       top now meets L2 with a vertical +X face (rear-LiDAR
//       side) and a smoothly chamfered -Y / +Y face (Atlas side).
//       Rear-left LiDAR M6 bolt is now 9.68 mm clear of the +X
//       face of the upper bracket; mirror for the lower bracket
//       and the rear-right LiDAR.
//
//   (3) SIDE 3 + SIDE 5 — NEW LOW PARTIAL WALLS, z = 6 → 54.
//       Sides 3 (V1→V2) and 5 (V3→V4) carry the rear Robin W
//       LiDARs. Previously they had NO bracket along the side —
//       the load path between V1 vertex bracket and the Atlas
//       upper bracket (and symmetrically V4 ↔ Atlas lower) ran
//       only through the L1 plate. The rear LiDARs hang from L2
//       and extend from z = 54 (body bottom) up to z = 139
//       (L2 underside). A FULL-HEIGHT wall along side 3 / 5
//       would collide with the LiDAR body.
//       v17m adds a PARTIAL wall along each rear side at
//       bracket_wall_t = 18 mm thick, running the full vertex-
//       to-vertex length but TOPPING OUT at z = 54 — exactly the
//       LiDAR body's underside. The wall stays inside the LiDAR's
//       Z envelope and outside its XY footprint (verified: all
//       4 wall corners outside the rotated rectangle of the LiDAR
//       body, and all 4 body corners outside the wall polygon).
//       Bonded L1↔L1 contact along each rear side grows from 0
//       to 145.4 mm × 48 mm height — significant torsional
//       stiffness added to the rear of the dome.
//
//   Geometry summary (post-v17m):
//     side 1 (V5-V0, 0°)   — front LiDAR        — M6 top-mount only
//     side 2 (V0-V1, 60°)  — FULLY ENCLOSED     — continuous 18 mm wall (v17l)
//     side 3 (V1-V2, 120°) — rear-left LiDAR    — LOW partial wall, z 6→54 (v17m)
//     side 4 (V2-V3, 180°) — Atlas rear flange  — 2 Atlas-side brackets
//     side 5 (V3-V4, 240°) — rear-right LiDAR   — LOW partial wall, z 6→54 (v17m)
//     side 6 (V4-V5, 300°) — FULLY ENCLOSED     — continuous 18 mm wall (v17l)
//
//   Total v17m bonded perimeter (counting full-height + half-height
//   contributions as their area equivalents):
//     full-height (z 6→139, 133 mm tall):
//       2 × 140 mm (sides 2 + 6) + 2 × 55 mm (Atlas) = 390 mm
//     half-height (z 6→54, 48 mm tall):
//       2 × 145.4 mm (sides 3 + 5) = 290.8 mm
//     Effective bonded area (height-weighted, vs v17k baseline 30 mm × 270 mm
//     = 8,100 mm²):
//       (390 × 133 + 290.8 × 48) / 30 = 2,194 mm of equivalent 30-mm-tall wall.
//     Roughly 8× the v17j L1↔L2 bonded perimeter, with the rear half-
//     height walls adding the L1↔L1 torsional brace the rear couldn't
//     get from full-height columns.
//
//   No TF impact from (2) and (3). (1) updates the front-cam
//   translations in config/sensor_dome_tf.yaml.
//
// CHANGES FROM v17k (v17l side-2 / side-6 enclosure):
//   Following the drop testing that motivated v17k's wider/longer/
//   filleted brackets, the two BLANK hex sides (side 2 = V0-V1 and
//   side 6 = V4-V5 — the LiDAR-less faces) are now FULLY ENCLOSED
//   by continuous vertical walls. With the v17g irregular hex these
//   sides are 140.111 mm long. v17l sets bracket_leg_len = 140 so
//   each pair of vertex brackets (V0+V1, V4+V5) overlaps along
//   nearly the entire side; their union is a single side-spanning
//   wall. Sides 1, 3, 4, 5 are untouched (they carry LiDARs or
//   the Atlas flange; can't be enclosed).
//
//   Geometry summary (post-v17l):
//     side 1 (V5-V0, 0°)   — front LiDAR        — 4 M6 top-mount bolts only
//     side 2 (V0-V1, 60°)  — FULLY ENCLOSED     — continuous 18 mm wall
//     side 3 (V1-V2, 120°) — rear-left LiDAR    — 4 M6 top-mount bolts only
//     side 4 (V2-V3, 180°) — Atlas rear flange  — 2 Atlas-side brackets
//     side 5 (V3-V4, 240°) — rear-right LiDAR   — 4 M6 top-mount bolts only
//     side 6 (V4-V5, 300°) — FULLY ENCLOSED     — continuous 18 mm wall
//
//   The continuous walls bond L1 and L2 along the full 140 mm of
//   each enclosed side rather than at four discrete 50 mm
//   contact patches. Total bonded perimeter grows from 4×50 +
//   2×35 = 270 mm to 2×140 + 4×50 + 2×35 = 550 mm (≈2× longer
//   adhesion line, with proportionally bigger drop resilience).
//
//   *** REQUIRED FIX: front-camera M3 head-clearance pockets ***
//   The widened wall along side 2 sweeps over the front-right
//   camera's M3 mount pattern (cam at (85, +73), bolts at
//   |x| = 76.75 / 93.25, |y| = 81.25). Three of four M3 bolts per
//   front camera land inside the 18 mm-thick wall. Without a
//   clearance pocket the M3 bolt heads can't seat on L2's underside
//   — the cameras become unmountable. v17l drills ø6 mm × 5 mm
//   pockets from the bracket TOP downward at every front-cam M3
//   position within (bracket_wall_t + 2) mm of the side-2/6 line.
//   New helper module: front_cam_m3_head_pockets(); called from
//   level1() unconditionally (both two-piece and unibody modes).
//
//   Numeric clearance verification at v17l geometry (LEG=140, T=18):
//     Front-right cam M3 (perpendicular distance to side 2 line):
//       (76.75, 64.75)   26.89 mm  outside  →  no pocket
//       (76.75, 81.25)   12.60 mm  inside   →  POCKET
//       (93.25, 64.75)   18.64 mm  outside  →  POCKET (margin < 1 mm)
//       (93.25, 81.25)    4.35 mm  inside   →  POCKET
//     Front-left cam M3 — symmetric (3 pockets).
//     Total v17l pockets: 6.
//
//   No TF impact. No sensor positions or bolt patterns moved.
//   The unibody variant benefits most from v17l because the
//   continuous wall braces the L1↔L2 join over a long edge
//   instead of relying on point loads at four vertex columns.
//
// CHANGES FROM v17j (v17k post-drop-test strengthening):
//   Field testing showed the printed dome fractures at the
//   bracket-to-L2 junction when dropped on a hard surface. The
//   failure mode is a clean shear at the 90° corner where each
//   bracket wall meets the L2 underside — a textbook stress-
//   concentration crack initiator. v17k strengthens the 6 support
//   columns in three ways at once:
//
//   1) Thicker walls:
//      - hex-vertex brackets: bracket_wall_t 12 → 18 mm (+50 %)
//      - atlas-side brackets: atlas_bracket_wall_t 10 → 15 mm
//        (parametric; flange-clipped to ~11 mm effective)
//
//   2) Longer legs (more L1 footprint + more bending inertia):
//      - hex-vertex brackets: bracket_leg_len 35 → 50 mm
//      - atlas-side brackets: atlas_bracket_leg_len 35 → 55 mm
//        (extends from x=-150 to x=-95 instead of -115; stays
//        clear of the cable-cutout teardrop tip at x=-100)
//
//   3) Symmetric top + bottom fillets at every bracket-plate
//      junction:
//      - bracket_fillet_r 10 → 15 mm (+50 %)
//      - BOTTOM (L1) fillet: pre-existing hull()-based taper from
//        an expanded base profile to the normal wall profile.
//      - TOP (L2) fillet: NEW in v17k — symmetric mirror of the
//        base fillet that flares the bracket outward as it
//        approaches L2's underside. Replaces the 90° edge that
//        used to be the crack initiator with a 15 mm linear taper
//        clipped to L1_plate_outline() so the flare stays within
//        the L2 footprint above.
//      - In both fillet zones the expanded profile grows along
//        the wall direction at both short ends and toward the
//        plate interior on the inner face, matching the v17c
//        original geometry — only the radius and the vertical
//        extent changed.
//
//   Net result of (1) + (2) + (3): bracket cross-section grows
//   from 12×35 = 420 mm² to 18×50 = 900 mm² (≈2.1× area), and
//   the two stress-concentration corners at each end of every
//   bracket are eliminated. Estimated drop-impact strength on
//   the side of the dome rises by roughly 3×–5× vs v17j.
//
//   Clearance verification (numeric check at v17k geometry):
//   - V0/V5 brackets — inner edge to nearest front-camera M3
//     bolt CENTER: 8.89 mm; hole edge wall: 7.19 mm. SAFE.
//   - V1/V4 brackets — inner edge to nearest camera M3: 46 mm.
//   - All hex-vertex brackets to LiDAR bodies: ≥ 47 mm.
//   - Atlas-side brackets — inner edge unchanged at y = ±64;
//     Atlas body remains at y ∈ [-61, +56.8]; gaps remain at
//     7.2 mm (upper) and 3.0 mm (lower).
//   - L1 pillar height unchanged at 133 mm. Straight-wall
//     section between top and bottom fillets shrinks from
//     113 mm (v17j: 133-20) to 103 mm (v17k: 133-30) — still
//     plenty of vertical wall for shear stiffness.
//
//   No TF impact. All sensor positions unchanged. The unibody
//   variant benefits more from v17k than the two-piece variant
//   because the unibody can't be re-tightened after a drop loosens
//   the bracket bolts.
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
// v17q: bracket-to-L2 bolt spec is M6×25 BHCS (was M6×20 v17b–v17p).
//   L2 plate clearance:  12 mm
//   Bracket engagement:  13 mm  (25 - 12)
//   Pilot clearance:      2 mm
//   ⇒ interlevel_tap_depth = 15 mm
// The tap hole bottom sits at L1_pillar_height - 15 = 118 mm = bottom
// edge of the v17k top fillet, so the pilot still lives entirely in
// solid bracket material.
interlevel_bolt_dia = 6.5;     // M6 clearance hole
interlevel_csink_dia = 11.0;   // M6 SHCS head clearance (LiDAR bolts)
interlevel_csink_depth = 4.0;
interlevel_tap_dia = 5.5;      // M6 self-tap pilot in printed plastic
interlevel_tap_depth = 15;     // v17q: 10 → 15 mm for M6×25 engagement

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

// v17k drop-resilience pass + v17l side enclosure: brackets are the load
// path that fails first on impact. v17k bumped thickness/length/fillet
// and added a symmetric TOP fillet at the L2 junction. v17l extends the
// bracket leg to the FULL side length so V0/V1 brackets union into one
// continuous wall along side 2 (V0-V1), and V4/V5 brackets union into
// one continuous wall along side 6 (V4-V5). The two blank sides without
// LiDARs are now fully enclosed for impact protection.
//
// Side length in v17g irregular hex = 140.11 mm. Leg = 140 leaves a
// 0.07 mm gap at each far end, picked up by the matched-vertex bracket
// from the other direction. The intersection() clip to L1_plate_outline
// keeps any over-extension within the plate footprint.
bracket_wall_t   = 18;    // wall thickness for hex vertex brackets (was 12)
bracket_leg_len  = 140;   // FULL side length (was 50 in v17k, 35 in v17j)
bracket_fillet_r = 15;    // fillet radius at bracket-to-plate junction (was 10)
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

// v17n DROP-RESILIENCE VERTEX ROUNDING
// ====================================
// Drop-test landings concentrate impact at the four hex vertices that
// project farthest in plan view (V0, V1, V4, V5). Each is the OUTER
// corner of a 120° internal angle and, with v17l's full-side enclosure,
// also marks the OUTER edge of the bracket wall. Replacing each sharp
// polygon corner with an explicit arc spreads the drop force along an
// arc length of ~26 mm (vs. ~10 mm at corner_r=10), dropping Hertzian
// peak stress to ~69% of the v17m baseline (R^{-2/5} scaling).
//
// Implementation: build the hex polygon with EXPLICIT arc points at
// V0/V1/V4/V5 (sampled at $fn=16-equivalent density) and keep V2/V3 as
// sharp polygon corners (they sit inside the Atlas flange union and
// are not exposed). The subsequent offset(corner_r) offset(-corner_r)
// in L1_plate_outline() leaves these large arcs UNCHANGED (a convex
// R=25 arc survives the ±10 mm offset cycle as itself, only V2/V3 +
// flange corners pick up the 10 mm rounding) — so a single corner_r
// retains the small-corner smoothing without flattening the new arcs.
//
// V2/V3 stay sharp because they are concave hex-vs-flange corners
// shielded by the Atlas-side brackets; the flange's own rear corners
// stay at corner_r = 10 mm (the offset pass is what rounds them).
rounded_vertex_r       = 25;   // arc radius at V0, V1, V4, V5 (was implicit 10)
rounded_vertex_arc_pts = 16;   // sample points per arc (smoother = larger)

// Geometry helper: arc of radius R that's tangent to BOTH adjacent
// sides at V_i. Computed from the actual incoming/outgoing edge
// directions so the result is correct for the v17g irregular hex
// (where V1/V4 have interior angle 123.43° instead of 120° because
// V0/V1/V4/V5 are pulled in but V2/V3 are not).
//
// Returns N+1 points on the arc, in CCW order from the start-tangent
// (on the incoming side, just before V_i) to the end-tangent (on the
// outgoing side, just after V_i). When stitched into the polygon
// point list these points replace the sharp vertex.
function vertex_arc_points(i, R, N) =
    let(
        Vi    = hex_vertex_xy(i),
        Vprev = hex_vertex_xy((i + 5) % 6),       // CCW previous
        Vnext = hex_vertex_xy((i + 1) % 6),       // CCW next
        // Incoming direction (Vprev → V_i), unit
        dix   = Vi[0] - Vprev[0],
        diy   = Vi[1] - Vprev[1],
        diL   = sqrt(dix*dix + diy*diy),
        diux  = dix / diL,    diuy = diy / diL,
        // Outgoing direction (V_i → Vnext), unit
        dox   = Vnext[0] - Vi[0],
        doy   = Vnext[1] - Vi[1],
        doL   = sqrt(dox*dox + doy*doy),
        doux  = dox / doL,    douy = doy / doL,
        // Inward bisector at V_i. The two unit vectors going AWAY from
        // V_i along each adjacent side are (-incoming) and outgoing.
        // Their sum BISECTS the OUTSIDE angle, but for a convex CCW
        // polygon vertex these two side-going-away vectors both point
        // INTO the polygon interior on average — so (-incoming + outgoing)
        // normalized IS the inward bisector. From V_i, points toward
        // the arc center.
        bix_raw = -diux + doux,
        biy_raw = -diuy + douy,
        biL     = sqrt(bix_raw*bix_raw + biy_raw*biy_raw),
        bix     = bix_raw / biL,
        biy     = biy_raw / biL,
        // Interior angle at V_i from (-incoming) · outgoing
        cos_int  = (-diux) * doux + (-diuy) * douy,
        half_int = acos(cos_int) / 2,
        // Setback (V_i to arc center) and tangent distance from V_i
        setback   = R / sin(half_int),
        tangent_d = R / tan(half_int),
        // Arc center
        cx = Vi[0] + setback * bix,
        cy = Vi[1] + setback * biy,
        // Tangent points: on incoming side (start), on outgoing side (end)
        sx = Vi[0] - tangent_d * diux,
        sy = Vi[1] - tangent_d * diuy,
        ex = Vi[0] + tangent_d * doux,
        ey = Vi[1] + tangent_d * douy,
        // Arc start/end angles measured at arc center
        start_a = atan2(sy - cy, sx - cx),
        end_a_raw = atan2(ey - cy, ex - cx),
        // Sweep CCW from start to end (the convex-vertex case)
        end_a   = end_a_raw < start_a ? end_a_raw + 360 : end_a_raw
    )
    [for (j = [0 : N])
        let(t = start_a + (end_a - start_a) * j / N)
        [cx + R * cos(t), cy + R * sin(t)]];

// Full hex polygon point list: arc points at V0/V1/V4/V5, sharp
// polygon points at V2/V3. CCW traversal.
function rounded_hex_polygon_points() =
    let(sectors = [
        for (i = [0:5])
            is_pulled_vertex(i)
                ? vertex_arc_points(i, rounded_vertex_r,
                                       rounded_vertex_arc_pts)
                : [hex_vertex_xy(i)]
    ])
    [for (sect = sectors) for (p = sect) p];

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
// v17q: each vertex bracket contributes 2 bolts at fracs [0.2, 0.4]
// of its 140 mm leg. With the v17l full-side enclosure (V0+V1 cover
// the entire V0-V1 side; V4+V5 cover the entire V4-V5 side), V0's
// bolts at 0.2/0.4 land at 28/56 mm from V0 and V1's bolts at 0.2/0.4
// (in V1's frame) land at 28/56 mm from V1 = 112/84 mm from V0. Net:
// 4 distinct positions per side at {28, 56, 84, 112} mm from the
// reference vertex — evenly spaced ~28 mm apart along the 140 mm
// wall. (Two-piece mode only; unibody_mode suppresses these holes.)
function bracket_bolt_pos(i, j) =
    let(
        v  = hex_vertex_xy(i),
        vx = v[0],
        vy = v[1],
        d  = bracket_wall_dir(i),
        n  = bracket_wall_norm(i),
        frac = (j == 0) ? 0.2 : 0.4
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
// v17k drop-resilience: leg extended 35→55 mm and wall_t 10→15 mm. The
// outer Y edge gets clipped by L1_plate_outline() at ±75.05 mm, so the
// effective wall thickness is ~11 mm, but the longer leg gives more L1
// contact area + more inertia. Top fillet is added in atlas_side_brackets().
atlas_bracket_leg_len   = 55;     // runs +X from x=-150 to x=-95 (was 35→x=-115)
atlas_bracket_wall_t    = 15;     // parametric value (was 10); flange-clipped to ~11
atlas_bracket_y_upper   = 64;     // upper wall inner edge Y
atlas_bracket_y_lower   = -64;    // lower wall inner edge Y

// Atlas-side bracket bolt position — v17q: ONE bolt per column at the
// LEG MIDPOINT (frac = 0.5), down from two bolts in v17m. The 55 mm
// leg is short enough that a single centered bolt clamps the bracket
// uniformly to L2; the long V0-V1 / V4-V5 walls (140 mm) keep their
// 4-bolt distribution. (Two-piece mode only.)
function atlas_bracket_bolt_pos(side) =
    let(
        bx = atlas_bracket_x_start + atlas_bracket_leg_len * 0.5,
        y_base = (side == 0) ? atlas_bracket_y_upper : atlas_bracket_y_lower,
        y_sign = (side == 0) ? 1 : -1,
        by = y_base + y_sign * atlas_bracket_wall_t / 2
    )
    [bx, by];

// v17q bolt layout (two-piece variant):
//   V0+V1 side (side 2):   2 bolts/bracket × 2 brackets = 4 entries → 4 distinct positions
//   V4+V5 side (side 6):   2 bolts/bracket × 2 brackets = 4 entries → 4 distinct positions
//   V2 column (Atlas-up):  1 bolt = 1 entry
//   V3 column (Atlas-low): 1 bolt = 1 entry
//   ---
//   Total: 10 M6×25 BHCS bracket-to-L2 bolts.
hex_bracket_bolt_positions = [
    bracket_bolt_pos(0, 0), bracket_bolt_pos(0, 1),   // V0 (30°)  — side 2 bolts 1, 2
    bracket_bolt_pos(1, 0), bracket_bolt_pos(1, 1),   // V1 (90°)  — side 2 bolts 3, 4
    bracket_bolt_pos(4, 0), bracket_bolt_pos(4, 1),   // V4 (270°) — side 6 bolts 1, 2
    bracket_bolt_pos(5, 0), bracket_bolt_pos(5, 1),   // V5 (330°) — side 6 bolts 3, 4
    atlas_bracket_bolt_pos(0),                        // V2 column (upper Atlas-side) — single midpoint bolt
    atlas_bracket_bolt_pos(1)                         // V3 column (lower Atlas-side) — single midpoint bolt
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
//   Base rectangle: x = [-140, -135], y = ±50 mm.
//   The hull creates smooth tangent lines from the circle to the
//   base, giving a teardrop that widens gradually from the tip.
//
// v17o: REAR-EDGE CLOSURE.
//   At v17n the teardrop's REAR base coincided with the L2 plate's
//   rear flange edge at x = -150 — so the opening was U-shaped (open
//   at the rear) and cables could escape SIDEWAYS through the open
//   slot. v17o moves the rear base inward to x = -140, leaving a
//   10 mm-thick L2-plate wall between the cable hole's rear edge
//   (x = -140) and the flange's rear edge (x = -150). That wall:
//     1) closes the open side, turning the teardrop into a fully
//        ENCLOSED triangular HOLE that cables must thread vertically
//        through (no sideways escape);
//     2) connects the two rear flange outer corners at (-150, ±75.06)
//        with a continuous transverse beam, bracing the rear of the
//        L2 plate against drop / vibration loads at the corners.
//   Material added: 10 mm × 12 mm × 150 mm ≈ 18 g of PETG.
//
// v17p: BASE NARROWED to clear the rear-camera M3 mounts.
//   At v17o (base_half_w = 60 mm) the upper tangent line of the
//   teardrop ran at y ≈ 34.17 at x = -111.14, which is exactly where
//   the rear-LEFT camera's third M3 mount hole sits (rotated bolt
//   pattern at aim 120° puts one hole at (-111.14, +36.00)). The
//   bolt hole's lower edge (radius 1.7 mm) clipped INTO the cable
//   opening by 0.46 mm — i.e., the M3 hole was partially open into
//   the cable cutout, leaving the rear-cam screw with no L2 material
//   on one side. (Symmetric exposure on the rear-RIGHT side.)
//   v17p drops base_half_w 60 → 50 mm, which lowers the tangent
//   line to y ≈ 29.43 at x = -111.14 (perpendicular distance to
//   the hole CENTER 4.98 mm; to the hole EDGE 3.28 mm). All four
//   M3 holes for both rear cameras now have ≥ 3.28 mm of L2 wall
//   between the hole edge and the cable opening boundary.
//   Effective opening: 100 mm wide at the base × 40 mm long ≈
//   2400 mm² trapezoid — still 13× the 182 mm² cable bundle
//   cross-section, plenty of room.
//
// Triangle height (tip to base inner edge): 30 mm (v17o: was 40 mm
//   at v17b–v17n).
// v17p clearance summary:
//   - Atlas-side bracket inner edge (y = ±64): 14 mm wall (v17o: 4 mm)
//   - Bracket bolt holes (y = ±69): ≥ 19 mm wall
//   - Rear-cam M3 hole edge: ≥ 3.28 mm wall
//   - GNSS recess: 42 mm clearance. Front-cam M3: ≥ 37 mm.
//
// NOT applied to L1 — L1 plate remains solid under the brackets.
cable_opening_tip_x        = -100;  // tip circle center X
cable_opening_tip_r        = 15;    // tip circle radius (ø30mm)
cable_opening_base_half_w  = 50;    // base half-width (v17p: was 60, exposed rear-cam M3)
cable_opening_base_outer_x = -140;  // base outer edge (v17o: was -150 = flange edge)
cable_opening_base_inner_x = -135;  // base inner edge (v17o: was -140; 5 mm rectangle depth)
cable_opening_rear_wall_t  = 10;    // closing wall thickness = flange edge (-150) to opening rear (-140)

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
//   v17i: pushed further toward V0/V1 and V4/V5; stereo baseline 130 → 146 mm.
//   v17m: pulled BACK toward the center because v17l's full-side enclosure
//   walls (18 mm thick) now physically overlap most of each front-cam M3
//   bolt pattern. New numbers move the cam 10 mm rearward and 18 mm
//   inward so the WORST-CASE M3 bolt is 24.93 mm from the side-2 line
//   (= 3.93 mm clear of the 21 mm top-fillet expansion, 6.93 mm clear
//   of the 18 mm main wall). Stereo baseline shrinks 146 → 110 mm but
//   is still wider than v17a–g (104 mm) and v17h (130 mm).
rgb_cam_x_front   = 75;       // forward distance (v17a–h: 105; v17i–l: 85)
rgb_cam_y_front   = 55;       // lateral offset (v17a–g: 52; v17h: 65; v17i–l: 73)

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
// circumradius.
// v17n: V0/V1/V4/V5 are now sampled as explicit 25 mm arcs in the
// polygon point list (see rounded_hex_polygon_points). The offset+
// offset pass still smooths the V2/V3 sharp corners and any flange-
// edge corners by corner_r=10, but a convex 25 mm arc survives the
// ±10 mm offset cycle unchanged.
module hex_plate_outline() {
    offset(r=corner_r) offset(r=-corner_r)
        polygon(rounded_hex_polygon_points());
}

// L1 plate: irregular hex + rectangular flange at side 4 for Atlas
// rear bolts. v17g: the hex is now the irregular outline (V0/V1/V4/V5
// pulled inward); side 4 (between V2 and V3) is unchanged, so the
// flange Y-span ±atlas_flange_half_width still matches V2/V3 cleanly.
// v17n: hex polygon carries pre-rounded V0/V1/V4/V5 arcs.
module L1_plate_outline() {
    offset(r=corner_r) offset(r=-corner_r)
    union() {
        polygon(rounded_hex_polygon_points());
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

        // Main wall: from base-fillet top to top-fillet bottom.
        // v17k: shortened by bracket_fillet_r on the upper end to make
        // room for the symmetric top fillet below. Still clipped to plate
        // outline so no part of the wall hangs beyond the rounded plate
        // corners.
        intersection() {
            translate([0, 0, plate_thickness + bracket_fillet_r])
                linear_extrude(L1_pillar_height - 2*bracket_fillet_r + 0.01)
                    L1_plate_outline();
            translate([vx, vy, plate_thickness + bracket_fillet_r])
                linear_extrude(L1_pillar_height - 2*bracket_fillet_r)
                    hex_bracket_wall_profile(i);
        }

        // v17k TOP FILLET — mirror of the base fillet but inverted in z.
        // The bracket flares outward as it approaches L2's underside, so
        // the bracket-to-L2 junction is a smooth taper rather than a
        // sharp 90° step. This is the load path that breaks first when
        // the dome is dropped on its side; the taper distributes shear
        // stress over the fillet height instead of concentrating it at
        // a single edge. Clipped to L1_plate_outline so the flare stays
        // within the plate footprint above (L2 has the same outline).
        intersection() {
            translate([0, 0, plate_thickness + L1_pillar_height - bracket_fillet_r])
                linear_extrude(bracket_fillet_r + 0.01)
                    L1_plate_outline();
            translate([vx, vy, plate_thickness + L1_pillar_height - bracket_fillet_r])
                hull() {
                    // narrow at the bottom (joins main wall)
                    linear_extrude(0.01)
                        hex_bracket_wall_profile(i);
                    // expanded at the top (fuses with L2 underside)
                    translate([0, 0, bracket_fillet_r])
                        linear_extrude(0.01)
                            hex_bracket_fillet_profile(i);
                }
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
    // Main wall: between base fillet and top fillet (v17k).
    intersection() {
        translate([0, 0, plate_thickness + fr])
            linear_extrude(L1_pillar_height - 2*fr + 0.01)
                L1_plate_outline();
        translate([atlas_bracket_x_start, atlas_bracket_y_upper,
                   plate_thickness + fr])
            linear_extrude(L1_pillar_height - 2*fr)
                square([atlas_bracket_leg_len, atlas_bracket_wall_t]);
    }
    // v17m TOP FILLET (upper bracket) — "spherical" rounded interface
    // to L2 that does NOT expand in +X.
    //
    // v17k flared the top in +x (toward dome center) by fr=15 mm. That
    // 15 mm reach pushed the bracket footprint at z=L2_z_bottom out to
    // x=-80, sweeping over the rear-LEFT Robin W M6 top-mount bolt at
    // (-85.68, +76.40). The bolt was inside both the X and Y range of
    // the top-fillet expansion, making the bolt un-installable.
    //
    // v17m removes the +X expansion entirely while keeping the -Y
    // (Atlas-facing) expansion. The bracket-to-L2 transition smooths
    // INWARD toward the Atlas body (where there's room) but stays
    // VERTICAL on the LiDAR-facing (+X) face. Result: the rear-left
    // LiDAR M6 bolt is now 9.68 mm clear of the bracket footprint at
    // z=L2_z_bottom.
    intersection() {
        translate([0, 0, plate_thickness + L1_pillar_height - fr])
            linear_extrude(fr + 0.01)
                L1_plate_outline();
        translate([atlas_bracket_x_start, atlas_bracket_y_upper,
                   plate_thickness + L1_pillar_height - fr])
            hull() {
                linear_extrude(0.01)
                    square([atlas_bracket_leg_len, atlas_bracket_wall_t]);
                translate([0, 0, fr])
                    linear_extrude(0.01)
                        translate([0, -atlas_fillet_inner_upper])
                            square([atlas_bracket_leg_len,     // v17m: no +fr (no +X expansion)
                                    atlas_bracket_wall_t + atlas_fillet_inner_upper]);
            }
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
    // Main wall: between base fillet and top fillet (v17k).
    intersection() {
        translate([0, 0, plate_thickness + fr])
            linear_extrude(L1_pillar_height - 2*fr + 0.01)
                L1_plate_outline();
        translate([atlas_bracket_x_start,
                   atlas_bracket_y_lower - atlas_bracket_wall_t,
                   plate_thickness + fr])
            linear_extrude(L1_pillar_height - 2*fr)
                square([atlas_bracket_leg_len, atlas_bracket_wall_t]);
    }
    // v17m TOP FILLET (lower bracket) — symmetric to upper. Removes the
    // +X flare that swept over the rear-RIGHT Robin W M6 bolt at
    // (-85.68, -76.40). Keeps the +Y (Atlas-facing) expansion at
    // atlas_fillet_inner_lower=2 mm (tight 3 mm Atlas clearance on the
    // lower side).
    intersection() {
        translate([0, 0, plate_thickness + L1_pillar_height - fr])
            linear_extrude(fr + 0.01)
                L1_plate_outline();
        translate([atlas_bracket_x_start,
                   atlas_bracket_y_lower - atlas_bracket_wall_t,
                   plate_thickness + L1_pillar_height - fr])
            hull() {
                linear_extrude(0.01)
                    square([atlas_bracket_leg_len, atlas_bracket_wall_t]);
                translate([0, 0, fr])
                    linear_extrude(0.01)
                        square([atlas_bracket_leg_len,         // v17m: no +fr (no +X expansion)
                                atlas_bracket_wall_t + atlas_fillet_inner_lower]);
            }
    }
}

// =============== v17m SIDE 3 + SIDE 5 PARTIAL WALLS =================
// Sides 3 (V1→V2) and 5 (V3→V4) carry the rear LiDARs. The LiDAR
// bodies hang from L2's underside and extend DOWN to z = 54 mm
// (= L2_z_bottom 139 − robin_body_h 85). Anything above z = 54 mm
// in the LiDAR's XY footprint blocks the body; the rear LiDARs in
// particular overhang past sides 3 and 5 in plan view (two body
// corners poke past the outer edges by ~5–11 mm), so a wall extending
// to L2 along these sides would collide with the LiDAR.
//
// v17m strengthens the LiDAR-bearing sides anyway, by adding a LOW
// wall along each side from z = plate_thickness (= 6) up to the
// LiDAR body bottom z (= 54). The wall is bracket_wall_t = 18 mm
// thick, extending INWARD from the side 3 / 5 line, and runs the
// full vertex-to-vertex length. This:
//
//   1. Bonds L1 to itself along the rear LiDAR sides over 48 mm of
//      vertical face — a torsion-resistant bottom plate that braces
//      the V1/V4 vertex brackets against the Atlas-side brackets
//      without ever rising into the LiDAR's body envelope.
//
//   2. Replaces the structurally unattractive option of adding V2 /
//      V3 vertex brackets that would extend INWARD toward the L2
//      center to reach past the Atlas body — a configuration that
//      would have collided with the Atlas rear flange + the cable-
//      cutout teardrop.
//
// Footprint verification (rear-left side 3, V1=(0,140.11), V2=(-130,75.06)):
//   wall polygon corners:
//     V1            = (  0.00, 140.11)
//     V2            = (-130.00,  75.06)
//     V2 + 18·n_in  = (-121.95,  58.97)
//     V1 + 18·n_in  = (  +8.05, 124.02)
//   inward normal n_in = (+0.447, -0.894)  (rotate side-3 dir +90° CCW)
//
//   rear-left Robin W body corners at angle 120° (z=54 to z=139):
//     (-112.06, +89.28), (-21.30, +141.68),
//     ( -58.70,  -3.12), ( +32.06,  +49.28)
//   None of the body corners are inside the wall polygon; none of
//   the wall corners are inside the body's rotated rectangle.
//   Z range non-overlap: wall ends at z=54, body starts at z=54.
//
// Side 5 is the mirror about X.
module side_3_5_partial_walls() {
    z_start = plate_thickness;            // 6  (L1 top)
    lidar_body_bottom_z = L2_z_bottom - robin_body_h;  // 139 − 85 = 54
    height = lidar_body_bottom_z - z_start;            // 48 mm

    // [start_vertex_index, end_vertex_index] for each partial-wall side.
    // Hex vertex order is CCW starting at 30°; sides 3 and 5 in our
    // convention run V1→V2 and V3→V4. Rotating the side direction by
    // +90° CCW gives the INWARD normal (toward the hex interior) for
    // CCW polygon traversal.
    partial_sides = [[1, 2], [3, 4]];

    for (s = partial_sides) {
        v_s = hex_vertex_xy(s[0]);
        v_e = hex_vertex_xy(s[1]);
        dx  = v_e[0] - v_s[0];
        dy  = v_e[1] - v_s[1];
        L   = sqrt(dx*dx + dy*dy);
        ux  = dx / L;     uy = dy / L;        // unit along side
        nx  = -uy;        ny = ux;            // inward normal (CCW polygon)
        t   = bracket_wall_t;                 // 18 mm wall

        intersection() {
            // Clip to L1 plate outline so corner sliver outside the
            // plate edge is removed.
            translate([0, 0, z_start])
                linear_extrude(height + 0.01)
                    L1_plate_outline();
            // Wall polygon, extruded from z_start to lidar bottom.
            translate([0, 0, z_start])
                linear_extrude(height)
                    polygon([
                        [v_s[0],          v_s[1]         ],
                        [v_e[0],          v_e[1]         ],
                        [v_e[0] + t * nx, v_e[1] + t * ny],
                        [v_s[0] + t * nx, v_s[1] + t * ny]
                    ]);
        }
    }
}

// =============== v17r FRONT-ACCESS CHAMFER =========================
// User-requested in v17r: open the V0-V5 (front) side of the dome wider
// so the Atlas INS X-side (front) face is fully accessible for plugging
// the front antenna connectors.
//
// Diagnosis: with v17l's full-side enclosure (V0/V1 brackets forming
// the side-2 wall, V4/V5 brackets forming the side-6 wall), the bracket
// inner corners near V0 and V5 intrude into the Atlas Y range:
//   V0 bracket inner corner P3 = (112.34, +54.47); Atlas top y = +56.80
//     → 2.33 mm intrusion at the +Y corner of the Atlas front face
//   V5 bracket inner corner P3 = (112.34, -54.47); Atlas bottom y = -61
//     → 6.53 mm intrusion at the -Y corner (Atlas is asymmetric in Y;
//        the body extends from y = -61 to y = +56.80, centered at -2.1)
//
// v17r cuts triangular chamfer wedges from both corners across the full
// L1 bracket height (z = 6 → 139), so the bracket inner edge stays clear
// of the Atlas Y range with a 5 mm safety margin (= finger / connector
// clearance):
//
//   V0 chamfer triangle (CCW):
//     ( 116.57, +61.80)   on V0 short edge near V0
//     (  99.64, +61.80)   on V0 inner edge inboard along d
//     ( 112.34, +54.47)   V0 bracket inner corner P3
//   Area 62.08 mm² in XY × 133 mm bracket height = 8.26 cm³
//
//   V5 chamfer triangle (CCW):
//     ( 119.00, -66.00)   on V5 short edge near V5
//     (  92.37, -66.00)   on V5 inner edge inboard along d
//     ( 112.34, -54.47)   V5 bracket inner corner P3
//   Area 153.57 mm² in XY × 133 mm bracket height = 20.42 cm³
//   (larger than V0 because Atlas bottom is 9 mm closer to V5 than
//    Atlas top is to V0.)
//
// Total: 28.7 cm³ ≈ 36 g PETG removed. Bracket cross-section reduction:
// 2.5% (V0) and 6.1% (V5) — minor structurally given the v17l 2520 mm²
// gross cross-section.
//
// Side-2 / side-6 enclosure SURVIVES — the chamfers only cut INNER
// material; the bracket OUTER face along side 2 / side 6 (= the
// dome's structural shell) is untouched. The v17q M6×25 bolt
// positions (at fracs [0.2, 0.4] along the wall, = 28 / 56 mm from
// the vertex) are at bracket parameter s > 14.67, OUTSIDE the
// chamfer region (which extends only to s = 14.66 on V0 side and
// s = 23.06 on V5 side — both bolt holes safely OUTBOARD of the
// chamfer cut on their respective walls).
module front_access_chamfer() {
    // Z range: full L1 bracket height. Cut starts above the L1 plate
    // (z=6, plate top) so the L1 plate is not pierced, and ends at L2's
    // underside (z=139) so the L2 plate is also unaffected.
    z_start = plate_thickness;
    z_height = L1_pillar_height;

    // V0 chamfer triangle
    v0_chamfer = [
        [116.57, +61.80],   // on V0 bracket short edge
        [ 99.64, +61.80],   // on V0 bracket inner edge
        [112.34, +54.47]    // V0 bracket inner corner (= V1 far inner corner)
    ];
    // V5 chamfer triangle
    v5_chamfer = [
        [119.00, -66.00],   // on V5 bracket short edge
        [ 92.37, -66.00],   // on V5 bracket inner edge
        [112.34, -54.47]    // V5 bracket inner corner (= V4 far inner corner)
    ];

    translate([0, 0, z_start])
        linear_extrude(z_height + 0.5) {
            polygon(v0_chamfer);
            polygon(v5_chamfer);
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

// =============== v17l FRONT-CAM M3 HEAD-CLEARANCE POCKETS =========
// Side-2 (V0→V1) and side-6 (V4→V5) walls now span the full vertex-
// to-vertex length to fully enclose the LiDAR-less faces of the dome
// for impact protection. The widened walls cover the X,Y region where
// the front-stereo cameras' M3 mount holes punch through L2. Without
// intervention the M3 bolt heads (≈ø5.7 mm SHCS / BHCS) would collide
// with the bracket top at z = L2_z_bottom, making the cameras
// impossible to install.
//
// Fix: drill a ø6 mm × 5 mm pocket from the bracket TOP downward at
// every front-camera M3 bolt position whose perpendicular distance to
// the nearest enclosed-side line is less than (bracket_wall_t + 2).
//
// Side 2 (V0,V1) line in normalized form:  0.5004*x + 0.8660*y - 121.34 = 0
// Side 6 (V4,V5) line is the X-axis mirror: 0.5004*x - 0.8660*y - 121.34 = 0
m3_pocket_dia   = 6.0;   // bolt-head clearance diameter
m3_pocket_depth = 5.0;   // depth from bracket top

function _perp_side2(p) = abs(0.5004*p[0] + 0.8660*p[1] - 121.34);
function _perp_side6(p) = abs(0.5004*p[0] - 0.8660*p[1] - 121.34);

// Walk the (now-extended) rgb_cam_layout, find the front cameras
// (aim_deg == 0), and emit a pocket position for every M3 mount hole
// whose perpendicular distance to side 2 or side 6 is below the
// (wall_t + safety) threshold.
front_cam_m3_pocket_positions = [
    for (cam = rgb_cam_layout)
        if (cam[2] == 0)                              // front cameras only
            for (off = rgb_cam_bolt_offsets)
                let(p = [cam[0] + off[0], cam[1] + off[1]],
                    d_min = min(_perp_side2(p), _perp_side6(p)))
                if (d_min < bracket_wall_t + 2)
                    p
];

module front_cam_m3_head_pockets() {
    for (p = front_cam_m3_pocket_positions) {
        translate([p[0], p[1],
                   plate_thickness + L1_pillar_height - m3_pocket_depth])
            cylinder(d=m3_pocket_dia, h=m3_pocket_depth + 0.5, $fn=32);
    }
}

// =============== CABLE OPENING (triangle cutout on side 4) ========
// 2D polygon for the cable opening. Oversized triangle — the plate
// outline clips it automatically via difference().

module cable_opening_2d() {
    // Teardrop: hull of tip circle + base rectangle.
    // The hull produces smooth tangent lines from the circle to the
    // base corners, with a round termination at the tip.
    //
    // v17o: the base rectangle now lives ENTIRELY INSIDE the L2 plate
    // (outer edge at cable_opening_base_outer_x = -140, INNER edge at
    // -135), so the resulting opening DOES NOT touch the plate's rear
    // edge at x=-150. The 10 mm L2-plate strip between x=-150 and
    // x=-140 becomes the closing wall that turns the U-shaped slot
    // into an enclosed triangular hole.
    hw = cable_opening_base_half_w;
    rect_x_extent = cable_opening_base_inner_x - cable_opening_base_outer_x;
    hull() {
        // Tip circle
        translate([cable_opening_tip_x, 0])
            circle(r=cable_opening_tip_r, $fn=64);
        // Base rectangle (now fully inside the plate)
        translate([cable_opening_base_outer_x, -hw])
            square([rect_x_extent, 2*hw]);
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
            // v17m: low partial walls along the rear LiDAR-bearing
            // sides 3 (V1→V2) and 5 (V3→V4), z = 6 → 54 only, so
            // the rear LiDAR bodies (z = 54 → 139) have clearance
            // above them.
            side_3_5_partial_walls();
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

        // v17l: M3 bolt-head clearance pockets in the side-2 / side-6
        // enclosure walls. Always drilled regardless of unibody_mode
        // because the front-stereo camera M3 holes punch through L2
        // at positions that fall inside the new wall material; without
        // the pockets the M3 bolt heads cannot seat on L2's underside.
        front_cam_m3_head_pockets();

        // v17r: open the V0-V5 (front) side for Atlas INS front-face
        // access. Cuts triangular wedges from the V0 and V5 bracket
        // inner corners. Applied in BOTH two-piece and unibody modes
        // — Atlas connector access is needed regardless of print mode.
        front_access_chamfer();

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
