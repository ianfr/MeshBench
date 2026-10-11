# Extra Details

Experiments: randomly remove N triangles from the active mesh
("Introduce random holes") to puncture test leaks, then run Detect leaks.
Detect leaks appends a LeakProbe tree item carrying the mesh plus the leak
report (hole loops, inside->outside paths); Patch holes appends a patched
mesh item. Applying either hides the input's eye (as in ParaView) so the
result renders as a flicker-free solid. The voxel barrier blocks faces
pierced by the surface (never whole cells), so even small holes stay open
to the flood fill. Patching is path-driven by default: only holes that
leak routes traverse are filled, using OpenMP-parallel BFS (one shortest
path per leaking inside/outside pair) plus seeded random walks for route
diversity, attributed to hole loops voxel-style. Fills use minimal-area
hole triangulation (centroid fan fallback). Tick "Patch every boundary
loop" to restore the old fill-everything behavior. A mesh with no
boundary loops is reported as sealed without leak paths: a closed surface
has no hole to leak through, so any voxel flood path through it threaded
a sub-cell gap (surface running nearly parallel to grid faces) rather
than a real hole.
Like ParaView, editing highlights render only for the active, eye-visible
stage: hole loops glow green on a LeakProbe stage (these are the patch
candidates) and stay red elsewhere. Probe spheres draw while a LeakProbe
stage is activated; leak-path polylines/markers (red BFS shortest paths
plus orange random-walk routes — every route patching is based on) and
the background voxel grid are opt-in via the "Show leak paths" and "Show
voxel grid" checkboxes (both default off). They draw on the active
LeakProbe stage, and also on a PatchHoles stage, which keeps the
routes/grid its patching was based on even when no Detect-leaks run
precedes patching. Unchecking a tree item's eye hides it completely, and
applied stages are plain mesh objects. Probed stages are tagged
LEAK/sealed in the sidebar. Visible stages render opaque when active,
faint otherwise; the Opacity spinner in the Active stage panel pins any
checked stage to an explicit opacity (shrinkwrap stays translucent).
