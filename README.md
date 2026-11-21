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

**Selection**: click inside the viewport, then press `S` (or the
"Select (S)" button) to enter selection mode on the active stage.
Click toggles individual triangles; click-and-drag draws a rectangle
that adds every triangle inside it. The selection highlights yellow.
The Selection panel then runs filters on just that subset:
- Extract as stage (new pipeline node with only the selected faces)
- Remesh selection (Gmsh remesh of the subset)
- Detect leaks (leak probe of the subset)

**Viewing**: a small XYZ axis indicator in the lower-left corner shows
the current camera orientation. The View menu provides six snap-to
directions (+X, -X, +Y, -Y, +Z, -Z), plus Reset camera.

## Dependencies
- Qt 6
- CGAL
- VTK
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
    libgmsh-dev
```

## Building and running

1. Install dependencies (see above).
2. Configure:

```bash
rm -rf build
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
```

3. Build (limit to 4 jobs: CGAL/VTK files are heavy and full
parallelism oversubscribes most machines):

```bash
cmake --build build -j 4
```

4. Run:

```bash
./build/meshbench
```

Gmsh support is enabled by default and needs no extra flags
(`-DMESHBENCH_ENABLE_GMSH=OFF` disables it).

Running the GUI requires a display server (X11/Wayland).

## Testing

```bash
ctest --test-dir build --output-on-failure
```

