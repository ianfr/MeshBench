#pragma once

// Leak detection and patching for surface meshes.
//
// Adaptation of OpenFOAM snappyHexMesh's leak-closure strategy
// (see openfoam-OpenFOAM-v2512, meshRefinement::blockLeakFaces and
// holeToFace) to a surface-mesh healing tool. snappyHexMesh works on a
// background hex mesh, so the mapping is:
//
//   snappyHexMesh concept            MeshBench analogue
//   ---------------------            ------------------
//   background hex mesh cells        uniform voxel grid over the mesh bbox
//   selectIntersectedFaces: faces    voxel *faces* pierced by any input
//     hit by the geometry are        triangle are blocked (barrier); cells
//     blocked                        themselves stay traversable, so small
//                                    holes are not paved over by rim cells
//   regionSplit: flood fill of       BFS flood fill of unblocked cells
//     cells through unblocked faces  through 6-neighbourhood (regionSplit)
//   locationsInMesh /                user-supplied inside/outside probe
//     locationsOutsideMesh           points; located with a findCell
//                                    analogue (nearest unblocked cell)
//   outside point reaching an        leak: inside and outside points share
//     inside region => leak           a region; the BFS shortest path is the
//                                    leak path (shortestPathSet analogue)
//   holeToFace::calcClosure +        boundary-edge loops of the surface mesh
//     baffle faces to seal the hole   are the closure set; patchHoles fills
//                                    them with minimal-area triangulation
//                                    (centroid fan fallback)
//
// This deliberately does NOT recreate full snappyHexMesh: there is no
// castellated refinement, snapping or layer addition -- only the
// inside/outside probing for holes plus simple patching.

#include "MeshTypes.h"

#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>
#include <array>
#include <vector>

namespace meshbench {

struct LeakPath {
  std::array<double, 3> from; // inside probe point
  std::array<double, 3> to;   // outside probe point
  // Voxel-cell centers from inside to outside (leak path to draw).
  QVector<std::array<double, 3>> polyline;
};

struct ProbeParams {
  int gridRes = 48;         // voxels per axis, clamped to [4, 128]
  int walksPerInside = 32;  // random walks per inside point
  int maxWalkSteps = 0;     // 0 = auto (256 * gridRes)
  unsigned walkSeed = 12345;
};

struct HoleLoop {
  QVector<std::array<double, 3>> points; // loop vertices in order
  double length = 0.0;
  int edges() const { return points.size(); }
};

struct LeakProbeResult {
  bool hasLeak = false;
  int regions = 0;       // number of voxel regions found
  int blockedFaces = 0;  // voxel faces pierced by the surface
  int totalCells = 0;
  double cellSize = 0.0; // background voxel edge length
  int gridRes = 0;       // voxels per axis of the background grid
  std::array<double, 3> gridOrigin = {0.0, 0.0, 0.0}; // min corner
  // Shortest (BFS) route per leaking inside->outside pair. Drawn in the UI.
  QVector<LeakPath> paths;
  // Extra leak routes from seeded random walks (corroboration: they catch
  // holes the single shortest path per pair never traverses). Each spans
  // its inside point to the nearest outside point (straight connector
  // from the grid border when the outside point is off-grid).
  QVector<LeakPath> walkPaths;
  QStringList warnings;
  // Boundary loops of the probed mesh, plus the subset lying on some
  // inside->outside route (bridge analysis: an open voxel face near the
  // rim joining an inside-reachable cell to an outside-reachable one).
  // Complete, unlike route sampling: every bridgeable hole is implicated.
  QVector<HoleLoop> loops;
  QSet<int> bridgingLoops;
};

namespace detail {
// Triangle vs axis-aligned voxel-face quad (unit-test exposure of the
// barrier predicate): true iff the triangle pierces the quad in the plane
// `axis == plane` spanning [lo1, hi1] x [lo2, hi2] in the other axes.
bool triOverlapsFaceQuad(const double a[3], const double b[3],
                         const double c[3], int axis, double plane, double lo1,
                         double hi1, double lo2, double hi2, double tol);
} // namespace detail

struct PatchResult {
  int loopsFound = 0;
  int loopsFilled = 0;
  int loopsSkippedTooLarge = 0;
  int loopsSkippedNotLeaking = 0; // not on any leak path (selective mode)
  int facesAdded = 0;
  QStringList warnings;
};



// View policy shared with VtkView: probe spheres render only while a
// LeakProbe stage is the activated one; leak paths need the "Show leak
// paths" toggle on top of that.
inline bool showsProbeOverlays(const QString& stageLabel) {
  return stageLabel.startsWith(QStringLiteral("LeakProbe"));
}

// Flood-fill probe of the voxelised background grid. inside/outside are
// (x,y,z) points in mesh coordinates. Pathfinding runs one OpenMP-parallel
// BFS per leaking inside->outside pair plus seeded random walks from every
// inside cell; both feed hole attribution. Deterministic for fixed params.
LeakProbeResult detectLeaks(const Mesh& mesh,
                            const std::vector<std::array<double, 3>>& inside,
                            const std::vector<std::array<double, 3>>& outside,
                            const ProbeParams& params = ProbeParams{});

// Boundary-edge loops of the mesh (each loop is one hole).
QVector<HoleLoop> findHoleLoops(const Mesh& mesh);

// Fill hole loops with at most maxLoopEdges edges using a centroid fan.
// Loops larger than that are left open and counted as skipped. When
// onlyLoops is non-empty, only those loop indices are patched (path-driven
// patching); the rest count as loopsSkippedNotLeaking. Empty = patch all.
PatchResult patchHoles(Mesh& mesh, int maxLoopEdges = 64,
                       const QSet<int>& onlyLoops = {});

inline bool isWatertight(const Mesh& mesh) {
  for (auto e : mesh.edges()) {
    if (mesh.is_border(e))
      return false;
  }
  return mesh.num_faces() > 0;
}

} // namespace meshbench
