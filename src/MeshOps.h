#pragma once

// CGAL-based surface mesh operations exposed by MeshBench, plus Gmsh
// OpenCASCADE primitives, remeshing and the low-level CAD session helpers.
//
// Supported operations (see README.md):
//  - Boolean operations (union / intersection / difference)
//  - Center geometry at center of volume
//  - CGAL alpha wrapping
//  - Slice with plane (clip, keeps the negative side)
// Gmsh additionally provides STL import/export round-trips and isotropic
// remeshing through the Gmsh C++ API.

#include "MeshTypes.h"

#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>

#include <array>
#include <vector>

namespace meshbench {

struct MeshStats {
  bool valid = false;
  std::size_t vertices = 0;
  std::size_t faces = 0;
  std::size_t edges = 0;
  std::size_t boundaryEdges = 0;
  bool closed = false;
  double volume = 0.0;
  double area = 0.0;
  double bboxMin[3] = {0, 0, 0};
  double bboxMax[3] = {0, 0, 0};
};

enum class BooleanOp { Union, Intersection, Difference };

bool loadMesh(const QString& path, Mesh& mesh, QString& error);
bool saveMesh(const QString& path, const Mesh& mesh, QString& error);

MeshStats computeStats(const Mesh& mesh);

// Translate the mesh so the volume centroid sits at the origin.
bool centerAtVolumeCentroid(Mesh& mesh, QString& error);

// Shift every vertex by (dx, dy, dz). Used to place a newly created
// mesh beside existing pipeline content so parallel objects do not
// start out overlapping (and occluding) each other.
void translateMesh(Mesh& mesh, double dx, double dy, double dz);

// ParaView-style Transform filter parameters. Rotation is given as
// right-handed Euler angles in degrees, applied X then Y then Z; the
// translation is applied afterwards. When rotateAboutCenter is true the
// mesh's bounding-box center is used as the rotation pivot, otherwise
// the world origin is used.
struct TransformParams {
  std::array<double, 3> translate{0.0, 0.0, 0.0};
  std::array<double, 3> rotateDeg{0.0, 0.0, 0.0};
  bool rotateAboutCenter = true;
};

// Apply a rigid translate + rotate transform in place. Returns false (with
// an error) for an empty mesh or non-finite parameters.
bool transformMesh(Mesh& mesh, const TransformParams& params, QString& error);

// Corefined boolean of two closed, triangulated meshes.
bool booleanOp(const Mesh& a, const Mesh& b, BooleanOp op, Mesh& out, QString& error);

// Closed watertight approximation of the input (CGAL alpha_wrap_3).
bool alphaWrap(const Mesh& in, double alpha, double offset, Mesh& out, QString& error);

// Feature-preserving isotropic remeshing (CGAL isotropic_remeshing). Edges
// whose incident face normals differ by more than featureAngleDeg are marked
// as constrained, so they may be split but not flipped or smoothed off the
// feature line. Boundary edges are preserved automatically. Runs internally
// on an inexact-constructions kernel, as required by CGAL remeshing.
bool isotropicRemesh(const Mesh& in, double targetEdgeLength, int iterations,
                     double featureAngleDeg, Mesh& out, QString& error);

// Extract feature edges whose incident face normals differ by more than
// featureAngleDeg, optionally including open boundary edges. Points are
// deduplicated and edges index into them (OpenFOAM edgeMesh layout).
bool extractFeatureEdges(const Mesh& mesh, double featureAngleDeg,
                         bool includeBoundary, FeatureEdges& out,
                         QString& error);

// Write an ASCII OpenFOAM featureEdgeMesh (.eMesh) file.
bool writeFeatureEdgeMesh(const QString& path, const FeatureEdges& edges,
                          QString& error);
// Read an ASCII OpenFOAM featureEdgeMesh (.eMesh) file.
bool loadFeatureEdges(const QString& path, FeatureEdges& edges, QString& error);

// Restore a selected feature-edge set on a surface by constrained Delaunay
// remeshing: the supplied polylines are protected/resampled and the output
// is refined with a minimum facet angle and bounded edge/facet size, avoiding
// degenerate sliver triangles. Implemented with CGAL's
// surface_Delaunay_remeshing (Mesh_3 Delaunay refinement; Boissonnat & Oudot,
// "Provably Good Sampling and Meshing of Surfaces", 2005). See
// https://doc.cgal.org/latest/PMP_Remeshing/group__PMP__global__remeshing__grp.html
// and https://doc.cgal.org/latest/Mesh_3/index.html
bool snapToFeatureEdges(const Mesh& in, const FeatureEdges& featureEdges,
                        double targetEdgeLength, double minFacetAngleDeg,
                        Mesh& out, QString& error);

// Experimental automatic cleanup of small, densely triangulated CAD details
// (washers, bolts, threaded inserts, ...). Faces smaller than
// detailSizeRatio * median triangle size are clustered; clusters whose
// bounding box stays below maxRegionFraction of the mesh bbox diagonal and
// hold at least minSmallFaces triangles are removed (optionally growing a few
// rings), and the resulting holes are filled, refined and smoothed using the
// local surroundings. Hole filling uses CGAL PMP
// triangulate_and_refine_hole; see
// https://doc.cgal.org/latest/PMP_Mesh_repair/group__PMP__hole__filling__grp.html
struct AutoCleanParams {
  double detailSizeRatio = 0.35;   // small-face threshold vs median triangle
  double maxRegionFraction = 0.05; // max cluster bbox / mesh bbox diagonal
  int minSmallFaces = 8;
  int growRings = 1;
  int maxHoleEdges = 256;
};

struct AutoCleanResult {
  int clustersRemoved = 0;
  int facesRemoved = 0;
  int holesPatched = 0;
  int holesSkipped = 0;
  int patchFacesAdded = 0;
  QStringList warnings;
};

bool autoCleanMesh(Mesh& mesh, const AutoCleanParams& params,
                   AutoCleanResult& result, QString& error);

// Docs-page style relative parameters: alpha = bboxDiag / alphaDenom,
// offset = bboxDiag / offsetDenom. Returns false (with error) unless the
// diagonal and both denominators are positive and finite.
bool alphaWrapParams(double bboxDiag, double alphaDenom, double offsetDenom,
                     double& alpha, double& offset, QString& error);

// Clip with a plane, keeping the negative side (like a ParaView slice).
bool clipWithPlane(const Mesh& in, const Point3& origin, const Vector3& normal,
                   Mesh& out, QString& error);

// Query the linked Gmsh library version (used by the UI + tests).
QString gmshVersion();// Isotropic surface remesh of an STL file via the Gmsh C++ API.
bool remeshStlWithGmsh(const QString& inPath, const QString& outPath,
                       double targetSize, QString& error);

// Basic 3D shapes created through the Gmsh OpenCASCADE kernel and meshed
// to an STL file. dims holds shape parameters in order:
//   Box: dx, dy, dz | Sphere: r | Ellipsoid: a, b, c
//   Cylinder: r, h | Cone: rBase, h | Torus: R, r
enum class PrimitiveShape { Box, Sphere, Ellipsoid, Cylinder, Cone, Torus };
QString primitiveShapeName(PrimitiveShape shape);
QStringList primitiveDimNames(PrimitiveShape shape);
struct PrimitiveParams {
  PrimitiveShape shape = PrimitiveShape::Box;
  QVector<double> dims = {1.0, 1.0, 1.0};
  double meshSize = 0.1; // target edge length
};
bool createPrimitiveStl(const PrimitiveParams& params, const QString& outPath,
                        QString& error);

// Randomly delete n triangles (deterministic for a given seed) to
// intentionally puncture holes for leak-detection experiments.
// Returns the number of faces actually removed.
int removeRandomTriangles(Mesh& mesh, int n, unsigned seed, QString& error);

// Build a mesh from a subset of faces (by face index); unreferenced
// vertices are dropped. Out-of-range ids are ignored; an empty selection
// yields an empty mesh. Used to run filters on a viewport selection.
Mesh extractSubmesh(const Mesh& mesh, const QSet<int>& faceIds);

// Build a copy of mesh with the selected face indices removed. The input is
// left untouched so callers can append the result as a new pipeline stage.
// Unreferenced vertices are dropped. Out-of-range ids are ignored; removed
// reports how many faces were actually deleted.
bool removeMeshFaces(const Mesh& mesh, const QSet<int>& faceIds, Mesh& out,
                     int& removed, QString& error);

// Build a copy of an edge set with the selected edge indices removed. Points
// that no surviving edge references are dropped and remaining edges are
// remapped. Out-of-range ids are ignored; removed reports how many edges were
// actually deleted.
bool removeFeatureEdges(const FeatureEdges& edges, const QSet<int>& edgeIds,
                        FeatureEdges& out, int& removed, QString& error);

// Concatenate meshes into one (faces appended, vertices remapped per
// input; shared vertices are NOT merged). Null entries are skipped;
// an empty input list yields an empty mesh. Used by Combine.
Mesh mergeMeshes(const std::vector<const Mesh*>& inputs);

} // namespace meshbench
