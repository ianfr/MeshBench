# SR-71 Blackbird — procedural CAD/mesh reconstruction

## Reference

The model was reconstructed using **only** the following source:

- <https://en.wikipedia.org/wiki/Lockheed_SR-71_Blackbird>, including its
  Wikimedia 3-view drawing and photographs.

Key published dimensions used from that article:

| Property | Value |
| --- | --- |
| Length | 32.74 m (107 ft 5 in) |
| Wingspan | 16.94 m (55 ft 7 in) |
| Height (with landing gear) | 5.64 m (18 ft 6 in) |
| Wing area | 170 m² (1,800 sq ft) |
| Crew | 2, tandem cockpits |
| Powerplant | 2 × Pratt & Whitney J58 afterburning turbojets |

The reconstruction deliberately models the aircraft in flight, so the landing
gear and drag chute are omitted. The model uses the SR-71's defining features:
the slender chined forebody, tandem canopy, blended delta wing, two integrated
J58 nacelles with shock-cone inlets and ejector exhausts, and the twin
inward-canted vertical tails.

## Coordinate system and final dimensions

- **X** = longitudinal, nose at `+X`, tail at `-X`
- **Y** = span, right wing at `+Y`
- **Z** = up

Measured from the saved STL:

| Property | Model value |
| --- | --- |
| Bounding box | `[-16.370, -8.470, -1.300]` to `[16.370, 8.470, 3.818]` |
| Length | 32.740 m |
| Span | 16.940 m |
| Height (flight configuration) | 5.118 m |
| Mesh volume | 221.06 m³ |
| Triangles | **464,958** |
| Vertices | 232,481 |

The 464,958-triangle result is the requested approximately-500k final mesh.

## Pipeline / object tree

The workflow was intentionally split into several independent Gmsh CAD tree
objects rather than one monolithic CAD stage. Each stage can be inspected,
hidden, or processed individually before the final union:

| Stage | Label | Contents | Final faces |
| --- | --- | --- | --- |
| 0 | `GmshCAD: SR-71 fuselage, chines and canopy` | Ellipsoidal center/aft body, slender nose cone, pitot, tail cone, cockpit canopy, triangular chine plate | 115,296 |
| 1 | `GmshCAD: SR-71 wing` | Clipped-delta wing slab shaped by four planform boolean cuts | 218,526 |
| 2 | `GmshCAD: SR-71 left nacelle` | J58 nacelle, nozzle, cylindrical inlet recess, shock cone, conical exhaust recess | 92,190 |
| 3 | `GmshCAD: SR-71 right nacelle` | Mirrored nacelle assembly | 92,136 |
| 4 | `GmshCAD: SR-71 left vertical tail` | Canted trapezoidal fin | 15,248 |
| 5 | `GmshCAD: SR-71 right vertical tail` | Mirrored canted fin | 15,256 |
| 6 | `Combine` | Watertight union of stages 0–5 | 464,958 |
| 7 | `FeatureEdges` | Same watertight mesh plus 30° feature-edge overlay | 464,958, 17,811 edges |

The final STL was saved from the `FeatureEdges` stage, which carries the same
watertight surface as the `Combine` stage.

## Construction steps

All solid geometry was built through the **MCP-only low-level Gmsh OCC API**,
not the GUI. The main calls were:

- `gmsh_session_begin`, `gmsh_session_end`
- `gmsh_add_ellipsoid`, `gmsh_add_cone`, `gmsh_add_cylinder`, `gmsh_add_box`
- `gmsh_boolean` (`fuse`, `cut`)
- `gmsh_rotate`, `gmsh_translate`
- `gmsh_set_mesh_size` with `refresh:false` during modeling, followed by one
  final refinement per component.

### Fuselage, chines and canopy

- Main body: long ellipsoid, `radii = [15.0, 1.05, 0.92]`.
- Nose: cone from `x = 8.5` to `x = 15.9`, tapering from `r = 0.80` to
  `r = 0.05`.
- Pitot: short 22 mm-radius tube from `x = 15.70` to `x = 16.37`.
- Tail: cone from `x = -12.0` to `x = -16.37`.
- Canopy: ellipsoid `center = [9.5, 0, 0.78]`,
  `radii = [2.55, 0.60, 0.52]`, fused into the upper fuselage.
- Chine: 0.22 m-thick plate from `x = 2.0` to `x = 14.0`, with two planar
  boolean cuts producing the triangular planform that widens into the wing
  root and tapers toward the nose.

### Wing

- Root chord from `x = +4.0` to `x = -13.0`; semi-span `8.47 m`.
- Tip chord from `x = -5.5` to `x = -9.0`; thickness `0.28 m`.
- Four rotated-box boolean cuts formed the swept leading edges and the
  tapered trailing edges.
- Approximate leading-edge sweep ≈ 55°, matching the delta-like SR-71
  planform.

### Nacelles

Each nacelle is an independent tree object built at `y = ±4.2`, `z = -0.15`:

- Forward-tapered frustum, 16.4 m long, radius 1.15 m → 1.02 m.
- Fused rear nozzle cone, radius 0.85 m → 1.00 m.
- Cylindrical inlet recess cut into the front (radius 0.92 m, depth 2.3 m).
- Shock cone fused into the recess bottom, tapering from `r = 0.95` to
  `r = 0.05` at `x = 4.2` (the visible inlet spike).
- Conical exhaust recess cut into the rear nozzle.

### Vertical tails

Each tail is an independent tree object:

- 4.8 m root chord, 3.2 m height, 0.16 m thickness.
- Two angled boolean cuts produce the swept leading edge and tapered top.
- Canted inward by 18° and embedded into the top of its nacelle at
  `x = -12.4`.

## Assembly and watertightness

The six component stages were merged with:

```json
operations_combine {"stages": [0,1,2,3,4,5], "mode": "union"}
```

The resulting `Combine` stage reports:

- `closed: true`
- `boundary_edges: 0`
- `volume: 221.057 m³`
- `faces: 464,958`

The final mesh was independently checked from the binary STL using a
coordinate-welded edge count at `1e-6` tolerance:

- boundary edges: **0**
- non-manifold edges: **0**
- signed volume: **+221.06 m³** (outward-oriented)

Because the CAD union was already watertight, `operations_alpha_wrap`,
`operations_shrinkwrap`, `operations_autoclean`, and
`operations_patch_holes`/leak filling were **not required** for the final
result. They were considered as fallback repair steps but would have smoothed
or altered the sharp CAD features unnecessarily. Gmsh surface remeshing was
used, with the model built coarsely and each component refined once to the
final `0.063 m` element size before the union.

## Feature edges

`operations_extract_feature_edges` was run on the final union with a 30°
threshold and boundary-edge inclusion:

- 17,811 feature-edge segments
- 17,747 feature-edge points

The sharp edges are retained geometrically at:

- wing leading and trailing edges,
- wing/fuselage and wing/nacelle intersections,
- nacelle inlet lip and shock-cone silhouette,
- exhaust nozzle rim and recess,
- vertical-tail leading/trailing/top edges,
- chine perimeter and canopy/fuselage intersection.

The extracted set was also exported to `/tmp/opencode/high-speed-plane.eMesh`
for inspection; the STL itself retains the sharp geometry.

## Visual verification

The scene was inspected repeatedly through `scene_get_image` and the standard
MCP views (`view_from_pos_z`, `view_from_pos_y`, `view_from_pos_x`,
`view_from_neg_y`, `view_from_neg_z`) during and after construction. The
top view confirms the pointed chined nose, delta wing, two long nacelles with
inlet spikes, and inward-canted tails. The side view confirms the slender
fuselage, canopy, wing line and tail height. The front view confirms the two
nacelles, inlet rims/spikes, thin wings and canted fins. The final
feature-edge overlay was inspected on the 464,958-triangle assembly.

## MeshBench changes made

Only critical functionality was changed, with tests:

1. **`src/GmshCadSession.cpp`**
   - Welds duplicate Gmsh/OCC seam nodes by coordinate before constructing
     the CGAL mesh. Without this, cone/cylinder seam duplicate vertices made
     otherwise-closed CAD solids appear open.
   - Rejects non-finite node coordinates.
   - Orients closed shells outward with
     `CGAL::Polygon_mesh_processing::orient_to_bound_a_volume`, so boolean
     inputs have consistent winding after OCC cuts.

2. **`src/MainWindowCommands.cpp`, `src/Commands.cpp`**
   - `gmsh_set_mesh_size` now accepts `refresh:false`, allowing geometry to be
     modeled at a coarse size and refined once at `gmsh_session_end` instead
     of remeshing after every CAD operation.
   - Raised the MCP command response deadline from 120 s to 3600 s for large
     CAD booleans and fine meshing.

3. **`src/McpHttpServer.cpp`**
   - Clears the 30 s read deadline before writing a response
     (`expires_never()`), so long-running but healthy commands are not
     reported as aborted connections.

4. **`tests/tst_gmshcad.cpp`**
   - Added `coneSeamIsWeldedWatertight`, which verifies a cone converts to a
     closed mesh with zero boundary edges.
   - Extended the boolean-cut test to assert a closed mesh with positive
     volume.

`tst_gmshcad` passes all 9 tests after these changes.

## Deliverable

- `high-speed-plane.stl` — binary STL, 464,958 triangles, watertight,
  outward-oriented, sharp feature edges retained.
- `high-speed-plane.md` — this summary.
