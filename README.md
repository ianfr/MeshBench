# MeshBench: GUI for Surface Mesh Operations

![GUI](MeshBench-Transform-Combine-Shrinkwrap-PatchHoles.png)

**AI Usage Disclaimer**: AI coding tools were used heavily in the development of MeshBench

MeshBench is a simple, easy-to-use GUI written in C++ for
3D surface mesh (i.e. STL) operations.

Supported **CGAL**-based operations include:
- Boolean operations
- Center geometry at center of volume
- Transform (translate/rotate a pipeline stage)
- CGAL alpha wrapping
- Isotropic remeshing with feature-edge preservation
- Feature-edge extraction with OpenFOAM `.eMesh` import/export
- Feature-edge snapping via constrained Delaunay remeshing
- Autoclean (experimental) for small dense CAD details
- Slice with plane

MeshBench has additional functionality for **leak detection and leak patching**
which has been adapted from OpenFOAM's snappyHexMesh codebase.

Additional operations are provided by **gmsh** using the C++ gmsh API
- Create primitive shapes (box, sphere, ellipsoid, cylinder, cone, torus)
  meshed through the Gmsh OpenCASCADE kernel
- Remesh an STL with a target element size (the mesh is reclassified
  into patches and regenerated, so the size is actually honored; an
  empty result fails loudly instead of appending a 0-face stage)

**Shrinkwrap**: start from a high-resolution icosphere enclosing the active
mesh and snap it onto the body (closest-point attraction + Laplacian
smoothing). 
- Enable "Select panels" and click baseline triangles to select
them, then set a weight (0 = ignore region, 1 = normal pull, up to 3 =
strong pull) and Apply — the wrap re-snaps live. 
- The Shrinkwrap dialog also offers an Even spacing (0-1) knob: higher values relax vertices
tangentially toward even spacing without shrinking the wrap. 
- The wrap renders translucent cyan so the baseline shows through.

**Pipeline stages**: tick the **Combine** checkbox on one or more
pipeline objects, or right-click a stage and choose "Select this stage
for Combine". Stages can be added one at a time; "Clear Combine
selection" resets them. This is useful when platform modifier-clicks
are unreliable.

Select pipeline objects and press **Delete** to remove them, or use
**Delete this stage (and downstream)** from the right-click menu.
Downstream operations that depend on a deleted stage are removed too;
independent imported sources are kept.

**Selection**: click inside the viewport, then press `S` (or the
"Select (S)" button) to enter selection mode on the active stage.
Click toggles individual triangles; click-and-drag draws a rectangle
that adds every triangle inside it. On a feature-edge stage, clicking a
drawn amber edge toggles that feature edge instead of a triangle (a drag
rectangle also adds the edges inside it). Selected faces highlight yellow
and selected edges highlight bright yellow.

Leaving selection mode (press `S` again, use **Delete selected**, or
uncheck the button) appends the deletions as a **new pipeline stage**; the
stage you selected on is never modified. The Selection panel can also run
non-destructive filters on just the selected subset:
- Extract as stage (new pipeline node with only the selected faces)
- Remesh selection (Gmsh remesh of the subset)
- Detect leaks (leak probe of the subset)

**Feature edges**: Operations > Extract feature edges creates a stage
with sharp edges selected by a feature angle (open boundary edges are
optional). The edges render amber over a translucent copy of the
surface. Use File > Export feature edges as eMesh to write an ASCII
OpenFOAM `featureEdgeMesh`, and File > Import feature edges (eMesh) to
load one back into the pipeline.

**Snap to feature edges**: Operations > Snap to feature edges remeshes
the active stage with a selected feature-edge stage as protected
polyline constraints, restoring sharp lines after operations such as
alpha wrapping. It uses CGAL's
[`surface_Delaunay_remeshing`](https://doc.cgal.org/latest/PMP_Remeshing/group__PMP__global__remeshing__grp.html),
implementing Delaunay surface refinement (Boissonnat & Oudot, *Provably
Good Sampling and Meshing of Surfaces*, 2005) via the
[CGAL 3D Mesh Generation package](https://doc.cgal.org/latest/Mesh_3/index.html).
The minimum facet-angle option bounds triangle angles to avoid sliver
or near-zero-area triangles.

**Quick launch**: press **Ctrl+Space** to open a searchable Operations
popup. Type any words from an operation name, use Up/Down to select,
and press Enter to run it.

**Autoclean (experimental)**: detects connected clusters of unusually
small triangles (washers, bolts, threaded inserts, etc.), removes them,
and re-fills the resulting holes with CGAL's
[hole triangulation and refinement](https://doc.cgal.org/latest/PMP_Mesh_repair/group__PMP__hole__filling__grp.html),
followed by Laplacian smoothing of the new patch vertices. Inputs
control the small-triangle ratio, maximum cluster size relative to the
model, minimum face count, rim growth, and maximum hole size. Existing
open boundaries are preserved.

**Viewing**: a small XYZ axis indicator in the lower-left corner shows
the current camera orientation. The View menu provides six snap-to
directions (+X, -X, +Y, -Y, +Z, -Z), plus Reset camera. File > Export
scene image... writes the current viewport to a PNG.

**LLM control (MCP)**: MeshBench runs a local, unauthenticated MCP server
on `http://127.0.0.1:8765/mcp` (Boost.Beast, loopback only). Every File,
Operations and View menu action is exposed as a tool, along with object-tree
inspection and PNG scene capture. See
[`skills/meshbench-mcp/SKILL.md`](skills/meshbench-mcp/SKILL.md) for the
complete LLM-facing reference and examples. Set `MESHBENCH_MCP_PORT` to change
the port, or to `off` to disable the server.

**MCP-only low-level Gmsh CAD tools**: `gmsh_session_begin` creates a single
pipeline stage and starts an OpenCASCADE CAD session. Low-level `gmsh_*` tools
(add primitives, extrude, revolve, fillet, chamfer, boolean cut/fuse/intersect,
transforms, delete, import/export CAD) update that same stage in place until
`gmsh_session_end`, so LLM-driven CAD workflows do not add a tree node per
operation. These tools are intentionally **not available in the GUI menus**;
see the skill document for the API and workflows.

## Dependencies
- Qt 6
- CGAL
- VTK
- Boost (Beast/Asio, header-only)
- CMake
- gmsh

### OpenSUSE Tumbleweed

```bash
sudo zypper install \
    gcc-c++ \
    cmake \
    ninja \
    cgal-devel \
    vtk-devel \
    vtk-qt \
    qt6-base-devel \
    qt6-widgets-devel \
    qt6-openglwidgets-devel \
    qt6-test-devel \
    gmsh-devel \
    boost-devel \
    libSM-devel \
    libICE-devel
```

### Ubuntu 26.10

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    ninja-build \
    qt6-base-dev \
    libvtk9-dev \
    libvtk9-qt-dev \
    libcgal-dev \
    libgmsh-dev \
    libboost-dev
```

## Building and running

1. Install dependencies (see above).
2. Configure:

```bash
rm -rf build
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
```

3. Build (limit to 8 jobs: CGAL/VTK files are heavy and full
parallelism oversubscribes most machines):

```bash
cmake --build build -j 8
```

4. Run:

```bash
./build/meshbench
```

Gmsh is a required dependency; the C++ API is linked directly and enabled
without extra flags.

Running the GUI requires a display server (X11/Wayland).

## Testing

```bash
ctest --test-dir build --output-on-failure
```

