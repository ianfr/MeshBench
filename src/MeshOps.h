#pragma once

// CGAL-based surface mesh operations exposed by MeshBench, plus optional
// Gmsh helpers (guarded by MESHBENCH_HAVE_GMSH).
//
// Supported operations (see README.md):
//  - Boolean operations (union / intersection / difference)
//  - Center geometry at center of volume
//  - CGAL alpha wrapping
//  - Slice with plane (clip, keeps the negative side)
// Gmsh additionally provides STL import/export round-trips and isotropic
// remeshing through the Gmsh C++ API.

#include "MeshTypes.h"

#include <QString>
#include <QVector>

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

// Corefined boolean of two closed, triangulated meshes.
bool booleanOp(const Mesh& a, const Mesh& b, BooleanOp op, Mesh& out, QString& error);

// Closed watertight approximation of the input (CGAL alpha_wrap_3).
bool alphaWrap(const Mesh& in, double alpha, double offset, Mesh& out, QString& error);

// Docs-page style relative parameters: alpha = bboxDiag / alphaDenom,
// offset = bboxDiag / offsetDenom. Returns false (with error) unless the
// diagonal and both denominators are positive and finite.
bool alphaWrapParams(double bboxDiag, double alphaDenom, double offsetDenom,
                     double& alpha, double& offset, QString& error);

// Clip with a plane, keeping the negative side (like a ParaView slice).
bool clipWithPlane(const Mesh& in, const Point3& origin, const Vector3& normal,
                   Mesh& out, QString& error);

#ifdef MESHBENCH_HAVE_GMSH
// Query the linked Gmsh library version (used by the UI + tests).
QString gmshVersion();
// Isotropic surface remesh of an STL file via the Gmsh C++ API.
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
#endif

// Randomly delete n triangles (deterministic for a given seed) to
// intentionally puncture holes for leak-detection experiments.
// Returns the number of faces actually removed.
int removeRandomTriangles(Mesh& mesh, int n, unsigned seed, QString& error);

} // namespace meshbench
