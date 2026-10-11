#include "MeshOps.h"

#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/alpha_wrap_3.h>

#include <CGAL/IO/STL.h>

#include <QFileInfo>

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef MESHBENCH_HAVE_GMSH
#include <gmsh.h>
#endif

namespace meshbench {
namespace PMP = CGAL::Polygon_mesh_processing;
namespace params = CGAL::Polygon_mesh_processing::parameters;

bool loadMesh(const QString& path, Mesh& mesh, QString& error) {
  Mesh tmp;
  const QString suffix = QFileInfo(path).suffix().toLower();
  bool ok = false;
  try {
    if (suffix == QStringLiteral("stl")) {
      ok = CGAL::IO::read_STL(path.toStdString(), tmp);
    } else {
      ok = CGAL::IO::read_polygon_mesh(path.toStdString(), tmp);
    }
  } catch (const std::exception& e) {
    error = QStringLiteral("Failed to read %1: %2").arg(path, e.what());
    return false;
  }
  if (!ok || tmp.num_vertices() == 0) {
    error = QStringLiteral("Failed to read %1 (unsupported or empty file).").arg(path);
    return false;
  }
  mesh = std::move(tmp);
  return true;
}

bool saveMesh(const QString& path, const Mesh& mesh, QString& error) {
  const QString suffix = QFileInfo(path).suffix().toLower();
  try {
    if (suffix == QStringLiteral("stl")) {
      if (!CGAL::IO::write_STL(path.toStdString(), mesh)) {
        error = QStringLiteral("Failed to write %1.").arg(path);
        return false;
      }
      return true;
    }
    if (!CGAL::IO::write_polygon_mesh(path.toStdString(), mesh)) {
      error = QStringLiteral("Failed to write %1.").arg(path);
      return false;
    }
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("Failed to write %1: %2").arg(path, e.what());
    return false;
  }
}

MeshStats computeStats(const Mesh& mesh) {
  MeshStats s;
  if (mesh.num_vertices() == 0)
    return s;
  s.valid = true;
  s.vertices = mesh.num_vertices();
  s.faces = mesh.num_faces();
  s.edges = mesh.num_edges();
  s.boundaryEdges = 0;
  for (auto e : mesh.edges()) {
    if (mesh.is_border(e))
      ++s.boundaryEdges;
  }
  s.closed = (s.boundaryEdges == 0);
  try {
    s.area = toDouble(PMP::area(mesh));
    if (s.closed)
      s.volume = toDouble(PMP::volume(mesh));
  } catch (...) {
    s.area = 0.0;
    s.volume = 0.0;
  }
  try {
    const auto bb = PMP::bbox(mesh);
    s.bboxMin[0] = toDouble(bb.xmin());
    s.bboxMin[1] = toDouble(bb.ymin());
    s.bboxMin[2] = toDouble(bb.zmin());
    s.bboxMax[0] = toDouble(bb.xmax());
    s.bboxMax[1] = toDouble(bb.ymax());
    s.bboxMax[2] = toDouble(bb.zmax());
  } catch (...) {
  }
  return s;
}

bool centerAtVolumeCentroid(Mesh& mesh, QString& error) {
  if (mesh.num_vertices() == 0) {
    error = QStringLiteral("Mesh is empty.");
    return false;
  }
  try {
    // Volume centroid of a closed mesh; fall back to bbox center otherwise.
    Point3 c = makePoint(0, 0, 0);
    if (CGAL::is_closed(mesh)) {
      c = PMP::centroid(mesh);
    } else {
      const auto bb = PMP::bbox(mesh);
      c = Point3((bb.xmin() + bb.xmax()) / 2, (bb.ymin() + bb.ymax()) / 2,
                 (bb.zmin() + bb.zmax()) / 2);
    }
    const Vector3 t(-c.x(), -c.y(), -c.z());
    const CGAL::Aff_transformation_3<Kernel> tr(CGAL::TRANSLATION, t);
    for (auto v : mesh.vertices())
      mesh.point(v) = tr(mesh.point(v));
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    return false;
  }
}

static bool triangulatedCopy(const Mesh& in, Mesh& out, QString& error) {
  try {
    out = in;
    PMP::triangulate_faces(out);
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    return false;
  }
}

bool booleanOp(const Mesh& a, const Mesh& b, BooleanOp op, Mesh& out, QString& error) {
  Mesh ta, tb;
  if (!triangulatedCopy(a, ta, error) || !triangulatedCopy(b, tb, error))
    return false;
  try {
    Mesh result;
    bool ok = false;
    switch (op) {
    case BooleanOp::Union:
      ok = PMP::corefine_and_compute_union(ta, tb, result);
      break;
    case BooleanOp::Intersection:
      ok = PMP::corefine_and_compute_intersection(ta, tb, result);
      break;
    case BooleanOp::Difference:
      ok = PMP::corefine_and_compute_difference(ta, tb, result);
      break;
    }
    if (!ok) {
      error = QStringLiteral("Boolean operation failed (inputs must be closed, "
                             "intersection-free triangle meshes).");
      return false;
    }
    out = std::move(result);
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    return false;
  }
}

bool alphaWrapParams(double bboxDiag, double alphaDenom, double offsetDenom,
                     double& alpha, double& offset, QString& error) {
  if (!(bboxDiag > 0) || !std::isfinite(bboxDiag)) {
    error = QStringLiteral("Bounding-box diagonal must be positive.");
    return false;
  }
  if (!(alphaDenom > 0) || !std::isfinite(alphaDenom) ||
      !(offsetDenom > 0) || !std::isfinite(offsetDenom)) {
    error = QStringLiteral("Alpha/offset denominators must be positive.");
    return false;
  }
  alpha = bboxDiag / alphaDenom;
  offset = bboxDiag / offsetDenom;
  return true;
}

bool alphaWrap(const Mesh& in, double alpha, double offset, Mesh& out, QString& error) {
  if (in.num_vertices() == 0) {
    error = QStringLiteral("Mesh is empty.");
    return false;
  }
  if (!(alpha > 0) || !(offset > 0)) {
    error = QStringLiteral("Alpha and offset must be positive.");
    return false;
  }
  try {
    Mesh tri = in;
    PMP::triangulate_faces(tri);
    // alpha_wrap_3 consumes a polygon soup and requires an inexact
    // constructions kernel, so convert through EPICK and back.
    using FastKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using FastMesh = CGAL::Surface_mesh<FastKernel::Point_3>;
    std::vector<FastKernel::Point_3> points;
    points.reserve(tri.num_vertices());
    std::unordered_map<std::size_t, std::size_t> idToPos;
    for (auto v : tri.vertices()) {
      const Point3& p = tri.point(v);
      idToPos[v.idx()] = points.size();
      points.emplace_back(toDouble(p.x()), toDouble(p.y()), toDouble(p.z()));
    }
    std::vector<std::vector<std::size_t>> faces;
    faces.reserve(tri.num_faces());
    for (auto f : tri.faces()) {
      std::vector<std::size_t> idx;
      for (auto v : vertices_around_face(tri.halfedge(f), tri))
        idx.push_back(idToPos[v.idx()]);
      if (idx.size() == 3)
        faces.push_back(std::move(idx));
    }
    if (faces.empty()) {
      error = QStringLiteral("Mesh has no triangular faces to wrap.");
      return false;
    }
    FastMesh wrap;
    CGAL::alpha_wrap_3(points, faces, alpha, offset, wrap);
    if (wrap.num_vertices() == 0) {
      error = QStringLiteral("Alpha wrapping produced an empty mesh; try a "
                             "smaller alpha or larger offset.");
      return false;
    }
    Mesh result;
    std::unordered_map<std::size_t, Mesh::Vertex_index> fastToExact;
    for (auto v : wrap.vertices()) {
      const auto& p = wrap.point(v);
      fastToExact[v.idx()] =
          result.add_vertex(makePoint(p.x(), p.y(), p.z()));
    }
    for (auto f : wrap.faces()) {
      std::vector<Mesh::Vertex_index> idx;
      for (auto v : vertices_around_face(wrap.halfedge(f), wrap))
        idx.push_back(fastToExact[v.idx()]);
      if (idx.size() == 3)
        result.add_face(idx[0], idx[1], idx[2]);
    }
    out = std::move(result);
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    return false;
  }
}

bool clipWithPlane(const Mesh& in, const Point3& origin, const Vector3& normal,
                   Mesh& out, QString& error) {
  if (in.num_vertices() == 0) {
    error = QStringLiteral("Mesh is empty.");
    return false;
  }
  if (normal.squared_length() == 0) {
    error = QStringLiteral("Plane normal must be non-zero.");
    return false;
  }
  try {
    Mesh tmp = in;
    PMP::triangulate_faces(tmp);
    const Plane3 plane(origin, normal);
    // clip_volume(true): a closed input stays closed, so volume stays valid.
    PMP::clip(tmp, plane, params::clip_volume(true));
    if (tmp.num_vertices() == 0) {
      error = QStringLiteral("Clipping removed the whole mesh.");
      return false;
    }
    out = std::move(tmp);
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    return false;
  }
}

#ifdef MESHBENCH_HAVE_GMSH
QString gmshVersion() {
  // The Gmsh C++ API exposes no runtime version query; report the API
  // version from the headers we compiled against.
  return QString::fromUtf8(GMSH_API_VERSION);
}

QString primitiveShapeName(PrimitiveShape shape) {
  switch (shape) {
  case PrimitiveShape::Box:
    return QStringLiteral("Box");
  case PrimitiveShape::Sphere:
    return QStringLiteral("Sphere");
  case PrimitiveShape::Ellipsoid:
    return QStringLiteral("Ellipsoid");
  case PrimitiveShape::Cylinder:
    return QStringLiteral("Cylinder");
  case PrimitiveShape::Cone:
    return QStringLiteral("Cone");
  case PrimitiveShape::Torus:
    return QStringLiteral("Torus");
  }
  return QStringLiteral("Shape");
}

QStringList primitiveDimNames(PrimitiveShape shape) {
  switch (shape) {
  case PrimitiveShape::Box:
    return {QStringLiteral("dx"), QStringLiteral("dy"), QStringLiteral("dz")};
  case PrimitiveShape::Sphere:
    return {QStringLiteral("r")};
  case PrimitiveShape::Ellipsoid:
    return {QStringLiteral("a"), QStringLiteral("b"), QStringLiteral("c")};
  case PrimitiveShape::Cylinder:
    return {QStringLiteral("r"), QStringLiteral("h")};
  case PrimitiveShape::Cone:
    return {QStringLiteral("rBase"), QStringLiteral("h")};
  case PrimitiveShape::Torus:
    return {QStringLiteral("R"), QStringLiteral("r")};
  }
  return {};
}

static bool positiveDims(const QVector<double>& dims, int need, QString& error) {
  if (dims.size() != need) {
    error = QStringLiteral("Shape needs %1 dimensions.").arg(need);
    return false;
  }
  for (double d : dims) {
    if (!(d > 0)) {
      error = QStringLiteral("All dimensions must be positive.");
      return false;
    }
  }
  return true;
}

bool createPrimitiveStl(const PrimitiveParams& params, const QString& outPath,
                        QString& error) {
  if (!(params.meshSize > 0)) {
    error = QStringLiteral("Mesh size must be positive.");
    return false;
  }
  // Validate before touching Gmsh so failures never leak an init session.
  const int need = primitiveDimNames(params.shape).size();
  if (!positiveDims(params.dims, need, error))
    return false;
  if (params.shape == PrimitiveShape::Torus && !(params.dims[0] > params.dims[1])) {
    error = QStringLiteral("Torus needs R > r.");
    return false;
  }
  const QVector<double> d = params.dims;
  try {
    gmsh::initialize();
    gmsh::option::setNumber("General.Terminal", 0);
    gmsh::clear();
    gmsh::model::add("primitive");
    switch (params.shape) {
    case PrimitiveShape::Box:
      // Centered at the origin.
      gmsh::model::occ::addBox(-d[0] / 2, -d[1] / 2, -d[2] / 2, d[0], d[1], d[2]);
      break;
    case PrimitiveShape::Sphere:
      gmsh::model::occ::addSphere(0, 0, 0, d[0]);
      break;
    case PrimitiveShape::Ellipsoid: {
      const int tag = gmsh::model::occ::addSphere(0, 0, 0, 1.0);
      gmsh::model::occ::dilate({{3, tag}}, 0, 0, 0, d[0], d[1], d[2]);
      break;
    }
    case PrimitiveShape::Cylinder:
      gmsh::model::occ::addCylinder(0, 0, -d[1] / 2, 0, 0, d[1], d[0]);
      break;
    case PrimitiveShape::Cone:
      gmsh::model::occ::addCone(0, 0, -d[1] / 2, 0, 0, d[1], d[0], 0.0);
      break;
    case PrimitiveShape::Torus:
      gmsh::model::occ::addTorus(0, 0, 0, d[0], d[1]);
      break;
    }
    gmsh::model::occ::synchronize();
    gmsh::option::setNumber("Mesh.CharacteristicLengthMin", params.meshSize);
    gmsh::option::setNumber("Mesh.CharacteristicLengthMax", params.meshSize);
    gmsh::model::mesh::generate(2); // surface mesh is what STL carries
    gmsh::write(outPath.toStdString());
    gmsh::finalize();
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    try {
      gmsh::finalize();
    } catch (...) {
    }
    return false;
  }
}

bool remeshStlWithGmsh(const QString& inPath, const QString& outPath,
                       double targetSize, QString& error) {
  if (!(targetSize > 0)) {
    error = QStringLiteral("Target element size must be positive.");
    return false;
  }
  // A merged STL is one discrete surface still carrying its input mesh,
  // so generate() alone just keeps it (target size ignored). Rebuild a
  // parametrizable model first: classify into patches and create their
  // geometry, then mesh at size. Descending classify angles: smaller
  // angles make finer patches, which are more likely disk-like
  // (parametrizable) on organic shapes where a coarse angle fails.
  const double anglesDeg[] = {40.0, 20.0, 10.0};
  QString lastError;
  try {
    gmsh::initialize();
    gmsh::option::setNumber("General.Terminal", 0);
    for (double aDeg : anglesDeg) {
      gmsh::clear();
      gmsh::merge(inPath.toStdString());
      gmsh::option::setNumber("Mesh.CharacteristicLengthMin", targetSize);
      gmsh::option::setNumber("Mesh.CharacteristicLengthMax", targetSize);
      try {
        gmsh::model::mesh::classifySurfaces(aDeg * 3.141592653589793 / 180.0,
                                            true, true);
        gmsh::model::mesh::createGeometry();
        gmsh::model::mesh::generate(2);
        // Never hand back an empty-but-valid STL ("remesh done: 0 faces"):
        // treat it as a failed angle and keep trying.
        std::vector<std::size_t> triTags, triNodes;
        gmsh::model::mesh::getElementsByType(2, triTags, triNodes);
        if (triTags.empty()) {
          lastError =
              QStringLiteral("classify angle %1 produced no faces").arg(aDeg);
          continue;
        }
        gmsh::write(outPath.toStdString());
        gmsh::finalize();
        return true;
      } catch (const std::exception& e) {
        lastError = QStringLiteral("classify angle %1: %2")
                        .arg(aDeg)
                        .arg(QString::fromUtf8(e.what()));
      }
    }
    gmsh::finalize();
  } catch (const std::exception& e) {
    lastError = QString::fromUtf8(e.what());
    try {
      gmsh::finalize();
    } catch (...) {
    }
  }
  error = QStringLiteral("Gmsh remeshing failed at element size %1 (%2); "
                         "try a smaller size.")
              .arg(targetSize)
              .arg(lastError);
  return false;
}
#endif

int removeRandomTriangles(Mesh& mesh, int n, unsigned seed, QString& error) {
  if (n <= 0) {
    error = QStringLiteral("Number of triangles must be positive.");
    return 0;
  }
  if (mesh.num_faces() == 0) {
    error = QStringLiteral("Mesh is empty.");
    return 0;
  }
  std::vector<Mesh::Face_index> faces;
  faces.reserve(mesh.num_faces());
  for (auto f : mesh.faces())
    faces.push_back(f);
  std::mt19937 rng(seed);
  std::shuffle(faces.begin(), faces.end(), rng);
  const int take = std::min<int>(n, static_cast<int>(faces.size()));
  // Mark the doomed faces, then rebuild from the kept ones. Rebuilding
  // (rather than Euler removal) keeps border edges exact in every case,
  // including removing all faces or adjacent ones.
  std::unordered_set<std::size_t> doomed;
  for (int i = 0; i < take; ++i)
    doomed.insert(faces[static_cast<size_t>(i)].idx());
  Mesh kept;
  std::unordered_map<std::size_t, Mesh::Vertex_index> vmap;
  for (auto f : mesh.faces()) {
    if (doomed.count(f.idx()) > 0)
      continue;
    auto h = mesh.halfedge(f);
    Mesh::Vertex_index ring[3] = {mesh.source(h), mesh.target(h),
                                  mesh.target(mesh.next(h))};
    Mesh::Vertex_index fresh[3];
    for (int k = 0; k < 3; ++k) {
      auto it = vmap.find(ring[k].idx());
      if (it == vmap.end()) {
        fresh[k] = kept.add_vertex(mesh.point(ring[k]));
        vmap[ring[k].idx()] = fresh[k];
      } else {
        fresh[k] = it->second;
      }
    }
    kept.add_face(fresh[0], fresh[1], fresh[2]);
  }
  mesh = std::move(kept);
  return take;
}

} // namespace meshbench
