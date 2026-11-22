#include "MeshOps.h"

#include <CGAL/boost/graph/border.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/surface_Delaunay_remeshing.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Mesh_constant_domain_field_3.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/alpha_wrap_3.h>

#include <CGAL/IO/STL.h>

#include <QFile>
#include <QFileInfo>
#include <QLocale>
#include <QRegularExpression>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gmsh.h>

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

void translateMesh(Mesh& mesh, double dx, double dy, double dz) {
  const Vector3 t(dx, dy, dz);
  const CGAL::Aff_transformation_3<Kernel> tr(CGAL::TRANSLATION, t);
  for (auto v : mesh.vertices())
    mesh.point(v) = tr(mesh.point(v));
}

bool transformMesh(Mesh& mesh, const TransformParams& params, QString& error) {
  if (mesh.num_vertices() == 0) {
    error = QStringLiteral("Mesh is empty.");
    return false;
  }
  for (int k = 0; k < 3; ++k) {
    if (!std::isfinite(params.translate[k]) ||
        !std::isfinite(params.rotateDeg[k])) {
      error = QStringLiteral("Transform values must be finite.");
      return false;
    }
  }
  try {
    const std::size_t vertexCount = mesh.num_vertices();
    const std::size_t faceCount = mesh.num_faces();
    constexpr double pi = 3.14159265358979323846;
    const double rx = params.rotateDeg[0] * pi / 180.0;
    const double ry = params.rotateDeg[1] * pi / 180.0;
    const double rz = params.rotateDeg[2] * pi / 180.0;
    const double cx = std::cos(rx), sx = std::sin(rx);
    const double cy = std::cos(ry), sy = std::sin(ry);
    const double cz = std::cos(rz), sz = std::sin(rz);
    // Right-handed Euler rotation R = Rz * Ry * Rx (X applied first).
    const double r11 = cz * cy;
    const double r12 = cz * sy * sx - sz * cx;
    const double r13 = cz * sy * cx + sz * sx;
    const double r21 = sz * cy;
    const double r22 = sz * sy * sx + cz * cx;
    const double r23 = sz * sy * cx - cz * sx;
    const double r31 = -sy;
    const double r32 = cy * sx;
    const double r33 = cy * cx;

    double ox = 0.0, oy = 0.0, oz = 0.0;
    if (params.rotateAboutCenter) {
      const auto bb = PMP::bbox(mesh);
      ox = (bb.xmin() + bb.xmax()) / 2;
      oy = (bb.ymin() + bb.ymax()) / 2;
      oz = (bb.zmin() + bb.zmax()) / 2;
    }
    // Fold the pivot into the homogeneous translation so a single affine
    // map rotates about the pivot and then translates.
    const double tx = params.translate[0] + ox - (r11 * ox + r12 * oy + r13 * oz);
    const double ty = params.translate[1] + oy - (r21 * ox + r22 * oy + r23 * oz);
    const double tz = params.translate[2] + oz - (r31 * ox + r32 * oy + r33 * oz);
    const CGAL::Aff_transformation_3<Kernel> tr(
        r11, r12, r13, tx,
        r21, r22, r23, ty,
        r31, r32, r33, tz);
    for (auto v : mesh.vertices())
      mesh.point(v) = tr(mesh.point(v));
    // Exit check: a rigid transform must preserve topology and stay finite.
    if (mesh.num_vertices() != vertexCount || mesh.num_faces() != faceCount) {
      error = QStringLiteral("Transform changed the mesh topology.");
      return false;
    }
    for (auto v : mesh.vertices()) {
      const Point3& p = mesh.point(v);
      if (!std::isfinite(toDouble(p.x())) || !std::isfinite(toDouble(p.y())) ||
          !std::isfinite(toDouble(p.z()))) {
        error = QStringLiteral("Transform produced non-finite coordinates.");
        return false;
      }
    }
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

bool isotropicRemesh(const Mesh& in, double targetEdgeLength, int iterations,
                     double featureAngleDeg, Mesh& out, QString& error) {
  if (in.num_vertices() == 0) {
    error = QStringLiteral("Mesh is empty.");
    return false;
  }
  if (!(targetEdgeLength > 0.0) || !std::isfinite(targetEdgeLength)) {
    error = QStringLiteral("Target edge length must be positive.");
    return false;
  }
  if (iterations < 1) {
    error = QStringLiteral("Iterations must be at least 1.");
    return false;
  }
  if (!(featureAngleDeg >= 0.0 && featureAngleDeg <= 180.0) ||
      !std::isfinite(featureAngleDeg)) {
    error = QStringLiteral("Feature angle must be between 0 and 180 degrees.");
    return false;
  }
  try {
    // CGAL remeshing does not support exact-constructions kernels, so
    // triangulate and convert to EPICK, then convert the result back.
    Mesh tri = in;
    PMP::triangulate_faces(tri);
    using FastKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using FastMesh = CGAL::Surface_mesh<FastKernel::Point_3>;
    FastMesh fast;
    std::unordered_map<std::size_t, FastMesh::Vertex_index> exactToFast;
    exactToFast.reserve(tri.num_vertices());
    for (auto v : tri.vertices()) {
      const Point3& p = tri.point(v);
      exactToFast[v.idx()] =
          fast.add_vertex(FastKernel::Point_3(toDouble(p.x()), toDouble(p.y()),
                                              toDouble(p.z())));
    }
    for (auto f : tri.faces()) {
      std::vector<FastMesh::Vertex_index> ring;
      for (auto v : vertices_around_face(tri.halfedge(f), tri))
        ring.push_back(exactToFast.at(v.idx()));
      if (ring.size() >= 3 && fast.add_face(ring) == FastMesh::null_face()) {
        error = QStringLiteral("Could not convert the mesh for remeshing.");
        return false;
      }
    }
    if (fast.num_faces() == 0) {
      error = QStringLiteral("Mesh has no faces to remesh.");
      return false;
    }

    // Mark sharp edges as constrained so remeshing keeps feature lines.
    // Boundary edges are constrained by CGAL automatically.
    auto edgeConstrained =
        fast.add_property_map<FastMesh::Edge_index, bool>("e:is_feature", false)
            .first;
    constexpr double pi = 3.14159265358979323846;
    const double cosThreshold = std::cos(featureAngleDeg * pi / 180.0);
    for (auto e : fast.edges()) {
      bool constrained = fast.is_border(e);
      if (!constrained) {
        const auto h = fast.halfedge(e);
        const auto f1 = fast.face(h);
        const auto f2 = fast.face(fast.opposite(h));
        if (f1 != FastMesh::null_face() && f2 != FastMesh::null_face()) {
          const auto n1 = PMP::compute_face_normal(f1, fast);
          const auto n2 = PMP::compute_face_normal(f2, fast);
          const double len1 = std::sqrt(toDouble(n1.squared_length()));
          const double len2 = std::sqrt(toDouble(n2.squared_length()));
          if (len1 > 0.0 && len2 > 0.0) {
            double cosine = toDouble(n1 * n2) / (len1 * len2);
            cosine = std::clamp(cosine, -1.0, 1.0);
            constrained = cosine < cosThreshold;
          }
        }
      }
      edgeConstrained[e] = constrained;
    }

    PMP::isotropic_remeshing(
        faces(fast), targetEdgeLength, fast,
        params::number_of_iterations(static_cast<unsigned int>(iterations))
            .edge_is_constrained_map(edgeConstrained)
            // Feature edges may be split, but not collapsed, flipped, or
            // smoothed off their line. protect_constraints(true) would
            // refuse long feature edges at fine target sizes.
            .protect_constraints(false)
            .collapse_constraints(false));
    if (fast.num_faces() == 0) {
      error = QStringLiteral("Isotropic remeshing produced an empty mesh; "
                             "try a larger target edge length.");
      return false;
    }

    Mesh result;
    std::unordered_map<std::size_t, Mesh::Vertex_index> fastToExact;
    fastToExact.reserve(fast.num_vertices());
    for (auto v : fast.vertices()) {
      const auto& p = fast.point(v);
      fastToExact[v.idx()] = result.add_vertex(makePoint(p.x(), p.y(), p.z()));
    }
    for (auto f : fast.faces()) {
      std::vector<Mesh::Vertex_index> ring;
      for (auto v : vertices_around_face(fast.halfedge(f), fast))
        ring.push_back(fastToExact.at(v.idx()));
      if (ring.size() >= 3)
        result.add_face(ring);
    }
    if (result.num_faces() == 0) {
      error = QStringLiteral("Could not convert the remeshed result.");
      return false;
    }
    out = std::move(result);
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    return false;
  }
}

bool extractFeatureEdges(const Mesh& mesh, double featureAngleDeg,
                         bool includeBoundary, FeatureEdges& out,
                         QString& error) {
  if (mesh.num_vertices() == 0) {
    error = QStringLiteral("Mesh is empty.");
    return false;
  }
  if (!(featureAngleDeg >= 0.0 && featureAngleDeg <= 180.0) ||
      !std::isfinite(featureAngleDeg)) {
    error = QStringLiteral("Feature angle must be between 0 and 180 degrees.");
    return false;
  }
  try {
    FeatureEdges result;
    std::unordered_map<std::size_t, int> pointIndex;
    const auto pointFor = [&](Mesh::Vertex_index v) {
      const auto it = pointIndex.find(v.idx());
      if (it != pointIndex.end())
        return it->second;
      const Point3& p = mesh.point(v);
      const int index = static_cast<int>(result.points.size());
      result.points.push_back(
          {toDouble(p.x()), toDouble(p.y()), toDouble(p.z())});
      pointIndex.emplace(v.idx(), index);
      return index;
    };
    // Cache face normals: each face is shared by several edges.
    std::unordered_map<std::size_t, Vector3> faceNormals;
    const auto normalFor = [&](Mesh::Face_index f) {
      const auto it = faceNormals.find(f.idx());
      if (it != faceNormals.end())
        return it->second;
      const Vector3 n = PMP::compute_face_normal(f, mesh);
      faceNormals.emplace(f.idx(), n);
      return n;
    };

    constexpr double pi = 3.14159265358979323846;
    const double cosThreshold = std::cos(featureAngleDeg * pi / 180.0);
    for (auto e : mesh.edges()) {
      bool include = false;
      if (mesh.is_border(e)) {
        include = includeBoundary;
      } else {
        const auto h = mesh.halfedge(e);
        const auto f1 = mesh.face(h);
        const auto f2 = mesh.face(mesh.opposite(h));
        if (f1 == Mesh::null_face() || f2 == Mesh::null_face()) {
          include = includeBoundary;
        } else {
          const Vector3 n1 = normalFor(f1);
          const Vector3 n2 = normalFor(f2);
          const double len1 = std::sqrt(toDouble(n1.squared_length()));
          const double len2 = std::sqrt(toDouble(n2.squared_length()));
          if (len1 > 0.0 && len2 > 0.0) {
            double cosine = toDouble(n1 * n2) / (len1 * len2);
            cosine = std::clamp(cosine, -1.0, 1.0);
            include = cosine < cosThreshold;
          }
        }
      }
      if (!include)
        continue;
      const auto h = mesh.halfedge(e);
      result.edges.push_back(
          {pointFor(mesh.source(h)), pointFor(mesh.target(h))});
    }
    out = std::move(result);
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    return false;
  }
}

bool writeFeatureEdgeMesh(const QString& path, const FeatureEdges& edges,
                          QString& error) {
  if (path.isEmpty()) {
    error = QStringLiteral("Output path is empty.");
    return false;
  }
  if (edges.points.empty() || edges.edges.empty()) {
    error = QStringLiteral("No feature edges to write.");
    return false;
  }
  for (const auto& e : edges.edges) {
    if (e[0] < 0 || e[1] < 0 ||
        e[0] >= static_cast<int>(edges.points.size()) ||
        e[1] >= static_cast<int>(edges.points.size())) {
      error = QStringLiteral("Feature edge index is out of range.");
      return false;
    }
  }
  for (const auto& p : edges.points) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
        !std::isfinite(p[2])) {
      error = QStringLiteral("Feature edge point is not finite.");
      return false;
    }
  }
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    error = QStringLiteral("Cannot open %1 for writing.").arg(path);
    return false;
  }
  try {
    QTextStream os(&file);
    os.setLocale(QLocale::c());
    os.setRealNumberNotation(QTextStream::SmartNotation);
    os.setRealNumberPrecision(17);
    const QString objectName = QFileInfo(path).fileName();
    os << "FoamFile\n"
       << "{\n"
       << "    version     2.0;\n"
       << "    format      ascii;\n"
       << "    class       featureEdgeMesh;\n"
       << "    location    \"constant/triSurface\";\n"
       << "    object      " << objectName << ";\n"
       << "}\n"
       << "// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * "
          "* * * * * * * //\n\n"
       << "// points:\n"
       << edges.points.size() << "\n(\n";
    for (const auto& p : edges.points)
      os << "(" << p[0] << " " << p[1] << " " << p[2] << ")\n";
    os << ")\n\n"
       << "// edges:\n"
       << edges.edges.size() << "\n(\n";
    for (const auto& e : edges.edges)
      os << "(" << e[0] << " " << e[1] << ")\n";
    os << ")\n\n"
       << "// "
          "*********************************************************************"
          "**** //\n";
    if (os.status() != QTextStream::Ok) {
      error = QStringLiteral("Failed while writing %1.").arg(path);
      file.close();
      return false;
    }
    file.close();
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    return false;
  }
}

bool loadFeatureEdges(const QString& path, FeatureEdges& edges, QString& error) {
  if (path.isEmpty()) {
    error = QStringLiteral("Input path is empty.");
    return false;
  }
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    error = QStringLiteral("Cannot open %1 for reading.").arg(path);
    return false;
  }
  QString text = QString::fromUtf8(file.readAll());
  file.close();
  if (text.contains(QRegularExpression(
          QStringLiteral("\\bformat\\s+binary\\s*;")))) {
    error = QStringLiteral("%1 is a binary eMesh; only ASCII is supported.")
                .arg(path);
    return false;
  }

  // Drop OpenFOAM comment lines so only header fields and the two lists
  // remain. Tokenisation also handles compact inline lists.
  text.remove(QRegularExpression(QStringLiteral("//[^\\n]*")));
  static const QRegularExpression tokenPattern(
      QStringLiteral("[A-Za-z_][A-Za-z0-9_]*|[(){}\\[\\];]|[^\\s(){}\\[\\];]+"));
  QStringList tokens;
  for (auto it = tokenPattern.globalMatch(text); it.hasNext();)
    tokens.append(it.next().captured());

  qsizetype pos = 0;
  // Skip the FoamFile { ... } header if present. Files without a header
  // start directly with the point list.
  for (qsizetype i = 0; i < tokens.size(); ++i) {
    if (tokens[i] != QStringLiteral("FoamFile"))
      continue;
    qsizetype j = i + 1;
    while (j < tokens.size() && tokens[j] != QStringLiteral("{"))
      ++j;
    if (j >= tokens.size())
      break;
    int depth = 0;
    for (; j < tokens.size(); ++j) {
      if (tokens[j] == QStringLiteral("{"))
        ++depth;
      else if (tokens[j] == QStringLiteral("}") && --depth == 0) {
        ++j;
        break;
      }
    }
    pos = j;
    break;
  }

  const auto readCount = [&](qsizetype& count) {
    if (pos >= tokens.size())
      return false;
    bool ok = false;
    const qlonglong value = tokens[pos++].toLongLong(&ok);
    if (!ok || value < 0 || value > 100000000LL)
      return false;
    count = static_cast<qsizetype>(value);
    return true;
  };
  const auto openList = [&]() {
    if (pos < tokens.size() &&
        (tokens[pos] == QStringLiteral("(") ||
         tokens[pos] == QStringLiteral("{"))) {
      return tokens[pos++];
    }
    return QString();
  };
  const auto closeMatches = [](const QString& open) {
    return open == QStringLiteral("(") ? QStringLiteral(")")
                                        : QStringLiteral("}");
  };

  FeatureEdges result;
  qsizetype pointCount = 0;
  if (!readCount(pointCount)) {
    error = QStringLiteral("%1 is not a valid OpenFOAM eMesh file.").arg(path);
    return false;
  }
  QString open = openList();
  if (!open.isEmpty()) {
    result.points.reserve(static_cast<std::size_t>(pointCount));
    for (qsizetype i = 0; i < pointCount; ++i) {
      if (pos >= tokens.size() || tokens[pos++] != QStringLiteral("(")) {
        error = QStringLiteral("Malformed point %1 in %2.").arg(i).arg(path);
        return false;
      }
      if (pos + 3 > tokens.size()) {
        error = QStringLiteral("Truncated point %1 in %2.").arg(i).arg(path);
        return false;
      }
      bool okX = false, okY = false, okZ = false;
      const double x = tokens[pos++].toDouble(&okX);
      const double y = tokens[pos++].toDouble(&okY);
      const double z = tokens[pos++].toDouble(&okZ);
      if (!okX || !okY || !okZ || !std::isfinite(x) || !std::isfinite(y) ||
          !std::isfinite(z) || pos >= tokens.size() ||
          tokens[pos++] != QStringLiteral(")")) {
        error = QStringLiteral("Malformed point %1 in %2.").arg(i).arg(path);
        return false;
      }
      result.points.push_back({x, y, z});
    }
    if (pos >= tokens.size() || tokens[pos++] != closeMatches(open)) {
      error = QStringLiteral("Unterminated point list in %1.").arg(path);
      return false;
    }
  } else if (pointCount != 0) {
    error = QStringLiteral("Missing point list in %1.").arg(path);
    return false;
  }

  qsizetype edgeCount = 0;
  if (!readCount(edgeCount)) {
    error = QStringLiteral("Missing edge count in %1.").arg(path);
    return false;
  }
  open = openList();
  if (!open.isEmpty()) {
    result.edges.reserve(static_cast<std::size_t>(edgeCount));
    for (qsizetype i = 0; i < edgeCount; ++i) {
      const bool wrapped = pos < tokens.size() &&
                           tokens[pos] == QStringLiteral("(");
      if (wrapped)
        ++pos;
      if (pos + 2 > tokens.size()) {
        error = QStringLiteral("Truncated edge %1 in %2.").arg(i).arg(path);
        return false;
      }
      bool okA = false, okB = false;
      const qlonglong a = tokens[pos++].toLongLong(&okA);
      const qlonglong b = tokens[pos++].toLongLong(&okB);
      if (wrapped && (pos >= tokens.size() ||
                      tokens[pos++] != QStringLiteral(")"))) {
        error = QStringLiteral("Malformed edge %1 in %2.").arg(i).arg(path);
        return false;
      }
      if (!okA || !okB || a < 0 || b < 0 ||
          a >= static_cast<qlonglong>(result.points.size()) ||
          b >= static_cast<qlonglong>(result.points.size())) {
        error = QStringLiteral("Edge %1 has an invalid point index in %2.")
                    .arg(i)
                    .arg(path);
        return false;
      }
      result.edges.push_back(
          {static_cast<int>(a), static_cast<int>(b)});
    }
    if (pos >= tokens.size() || tokens[pos++] != closeMatches(open)) {
      error = QStringLiteral("Unterminated edge list in %1.").arg(path);
      return false;
    }
  } else if (edgeCount != 0) {
    error = QStringLiteral("Missing edge list in %1.").arg(path);
    return false;
  }

  if (result.points.empty() || result.edges.empty()) {
    error = QStringLiteral("%1 contains no feature edges.").arg(path);
    return false;
  }
  edges = std::move(result);
  return true;
}

bool snapToFeatureEdges(const Mesh& in, const FeatureEdges& featureEdges,
                        double targetEdgeLength, double minFacetAngleDeg,
                        Mesh& out, QString& error) {
  if (in.num_vertices() == 0) {
    error = QStringLiteral("Mesh is empty.");
    return false;
  }
  if (featureEdges.points.empty() || featureEdges.edges.empty()) {
    error = QStringLiteral("No feature edges selected.");
    return false;
  }
  if (!(targetEdgeLength > 0.0) || !std::isfinite(targetEdgeLength)) {
    error = QStringLiteral("Target edge length must be positive.");
    return false;
  }
  // CGAL's Mesh_3 refinement angle bounds are intended to stay below 30
  // degrees; 0 disables the angle criterion.
  if (!(minFacetAngleDeg >= 0.0 && minFacetAngleDeg <= 30.0) ||
      !std::isfinite(minFacetAngleDeg)) {
    error = QStringLiteral("Minimum facet angle must be between 0 and 30 "
                           "degrees.");
    return false;
  }
  for (const auto& e : featureEdges.edges) {
    if (e[0] < 0 || e[1] < 0 ||
        e[0] >= static_cast<int>(featureEdges.points.size()) ||
        e[1] >= static_cast<int>(featureEdges.points.size())) {
      error = QStringLiteral("Feature edge index is out of range.");
      return false;
    }
  }
  try {
    // Delaunay remeshing does not support exact-constructions kernels.
    Mesh tri = in;
    PMP::triangulate_faces(tri);
    using FastKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using FastMesh = CGAL::Surface_mesh<FastKernel::Point_3>;
    FastMesh fast;
    std::unordered_map<std::size_t, FastMesh::Vertex_index> exactToFast;
    exactToFast.reserve(tri.num_vertices());
    for (auto v : tri.vertices()) {
      const Point3& p = tri.point(v);
      exactToFast[v.idx()] =
          fast.add_vertex(FastKernel::Point_3(toDouble(p.x()), toDouble(p.y()),
                                              toDouble(p.z())));
    }
    for (auto f : tri.faces()) {
      std::vector<FastMesh::Vertex_index> ring;
      for (auto v : vertices_around_face(tri.halfedge(f), tri))
        ring.push_back(exactToFast.at(v.idx()));
      if (ring.size() >= 3 && fast.add_face(ring) == FastMesh::null_face()) {
        error = QStringLiteral("Could not convert the mesh for feature-edge "
                               "snapping.");
        return false;
      }
    }
    if (fast.num_faces() == 0) {
      error = QStringLiteral("Mesh has no faces to snap.");
      return false;
    }

    // The selected edge set is passed as polyline constraints. CGAL merges
    // the individual segments into a feature graph, resamples it, and keeps
    // it protected during Delaunay refinement.
    std::vector<std::vector<FastKernel::Point_3>> polylines;
    polylines.reserve(featureEdges.edges.size());
    for (const auto& e : featureEdges.edges) {
      const auto& a = featureEdges.points[static_cast<std::size_t>(e[0])];
      const auto& b = featureEdges.points[static_cast<std::size_t>(e[1])];
      const double dx = b[0] - a[0];
      const double dy = b[1] - a[1];
      const double dz = b[2] - a[2];
      if (dx * dx + dy * dy + dz * dz <= 0.0)
        continue; // ignore zero-length segments
      polylines.push_back(
          {FastKernel::Point_3(a[0], a[1], a[2]),
           FastKernel::Point_3(b[0], b[1], b[2])});
    }
    if (polylines.empty()) {
      error = QStringLiteral("The selected feature edge set has no usable "
                             "segments.");
      return false;
    }

    // Constant sizing fields: feature/surface edge length and the allowed
    // surface approximation distance. The angle bound is what prevents
    // sliver triangles with near-zero area.
    using SizingField = CGAL::Mesh_constant_domain_field_3<FastKernel, int>;
    SizingField edgeSize(targetEdgeLength);
    SizingField facetSize(targetEdgeLength);
    SizingField surfaceDistance(targetEdgeLength / 10.0);

    FastMesh remeshed = PMP::surface_Delaunay_remeshing(
        fast, params::protect_constraints(true)
                  .polyline_constraints(polylines)
                  .mesh_edge_size(edgeSize)
                  .mesh_facet_size(facetSize)
                  .mesh_facet_angle(minFacetAngleDeg)
                  .mesh_facet_distance(surfaceDistance));
    if (remeshed.num_faces() == 0) {
      error = QStringLiteral("Feature-edge snapping produced an empty mesh; "
                             "try a larger target edge length or a smaller "
                             "facet angle.");
      return false;
    }

    Mesh result;
    std::unordered_map<std::size_t, Mesh::Vertex_index> fastToExact;
    fastToExact.reserve(remeshed.num_vertices());
    for (auto v : remeshed.vertices()) {
      const auto& p = remeshed.point(v);
      fastToExact[v.idx()] = result.add_vertex(makePoint(p.x(), p.y(), p.z()));
    }
    for (auto f : remeshed.faces()) {
      std::vector<Mesh::Vertex_index> ring;
      for (auto v : vertices_around_face(remeshed.halfedge(f), remeshed))
        ring.push_back(fastToExact.at(v.idx()));
      if (ring.size() >= 3)
        result.add_face(ring);
    }
    if (result.num_faces() == 0) {
      error = QStringLiteral("Could not convert the snapped result.");
      return false;
    }
    out = std::move(result);
    return true;
  } catch (const std::exception& e) {
    error = QString::fromUtf8(e.what());
    return false;
  }
}

bool autoCleanMesh(Mesh& mesh, const AutoCleanParams& params,
                   AutoCleanResult& result, QString& error) {
  result = AutoCleanResult{};
  if (mesh.num_faces() == 0) {
    error = QStringLiteral("Mesh has no faces.");
    return false;
  }
  if (!(params.detailSizeRatio > 0.0 && params.detailSizeRatio <= 1.0) ||
      !std::isfinite(params.detailSizeRatio)) {
    error = QStringLiteral("Small triangle ratio must be in (0, 1].");
    return false;
  }
  if (!(params.maxRegionFraction > 0.0 && params.maxRegionFraction <= 1.0) ||
      !std::isfinite(params.maxRegionFraction)) {
    error = QStringLiteral("Max region size must be in (0, 1].");
    return false;
  }
  if (params.minSmallFaces < 1) {
    error = QStringLiteral("Minimum small faces must be at least 1.");
    return false;
  }
  if (params.growRings < 0 || params.growRings > 20) {
    error = QStringLiteral("Grow rings must be between 0 and 20.");
    return false;
  }
  if (params.maxHoleEdges < 3) {
    error = QStringLiteral("Max hole edges must be at least 3.");
    return false;
  }
  try {
    Mesh tri = mesh;
    PMP::triangulate_faces(tri);
    if (tri.num_faces() == 0) {
      error = QStringLiteral("Mesh has no triangular faces.");
      return false;
    }

    const auto bb = PMP::bbox(tri);
    const double gx = bb.xmax() - bb.xmin();
    const double gy = bb.ymax() - bb.ymin();
    const double gz = bb.zmax() - bb.zmin();
    const double globalDiag = std::sqrt(gx * gx + gy * gy + gz * gz);
    if (!(globalDiag > 0.0)) {
      error = QStringLiteral("Mesh bounding box is degenerate.");
      return false;
    }

    // Face-area statistics. Larger detail density => smaller median size;
    // the ratio is relative so the filter scales with the model.
    std::unordered_map<std::size_t, double> faceArea;
    std::vector<double> areas;
    areas.reserve(tri.num_faces());
    for (auto f : tri.faces()) {
      const double a = toDouble(PMP::face_area(f, tri));
      faceArea.emplace(f.idx(), a);
      areas.push_back(a);
    }
    std::sort(areas.begin(), areas.end());
    const double medianArea = areas[areas.size() / 2];
    if (!(medianArea > 0.0)) {
      error = QStringLiteral("Mesh has no positive-area faces.");
      return false;
    }
    const double medianSize = std::sqrt(medianArea);
    const double smallLimit =
        (params.detailSizeRatio * medianSize) *
        (params.detailSizeRatio * medianSize);

    std::unordered_set<std::size_t> smallFaces;
    for (auto f : tri.faces()) {
      if (faceArea.at(f.idx()) <= smallLimit)
        smallFaces.insert(f.idx());
    }
    if (smallFaces.empty()) {
      result.warnings << QStringLiteral("No small triangles matched the "
                                        "detail size ratio.");
      return true;
    }

    const auto faceNeighbors = [&](Mesh::Face_index f) {
      std::vector<Mesh::Face_index> neighbors;
      auto h = tri.halfedge(f);
      const auto start = h;
      do {
        const auto nf = tri.face(tri.opposite(h));
        if (nf != Mesh::null_face())
          neighbors.push_back(nf);
        h = tri.next(h);
      } while (h != start);
      return neighbors;
    };

    const double maxClusterDiag = params.maxRegionFraction * globalDiag;
    std::unordered_set<std::size_t> visited;
    std::unordered_set<std::size_t> selected;
    for (auto seed : tri.faces()) {
      if (!smallFaces.count(seed.idx()) || visited.count(seed.idx()))
        continue;

      std::vector<Mesh::Face_index> cluster;
      std::queue<Mesh::Face_index> pending;
      pending.push(seed);
      visited.insert(seed.idx());
      while (!pending.empty()) {
        const auto f = pending.front();
        pending.pop();
        cluster.push_back(f);
        for (auto n : faceNeighbors(f)) {
          if (smallFaces.count(n.idx()) && !visited.count(n.idx())) {
            visited.insert(n.idx());
            pending.push(n);
          }
        }
      }
      if (static_cast<int>(cluster.size()) < params.minSmallFaces)
        continue;

      // Grow a small rim so the patch does not leave a fringe of the old
      // detailed feature behind, then verify the whole cluster stays small.
      std::unordered_set<std::size_t> expanded;
      for (auto f : cluster)
        expanded.insert(f.idx());
      std::vector<Mesh::Face_index> frontier = cluster;
      for (int ring = 0; ring < params.growRings; ++ring) {
        std::vector<Mesh::Face_index> next;
        for (auto f : frontier) {
          for (auto n : faceNeighbors(f)) {
            if (expanded.insert(n.idx()).second)
              next.push_back(n);
          }
        }
        frontier.swap(next);
        if (frontier.empty())
          break;
      }

      double lo[3] = {std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max()};
      double hi[3] = {std::numeric_limits<double>::lowest(),
                      std::numeric_limits<double>::lowest(),
                      std::numeric_limits<double>::lowest()};
      for (auto id : expanded) {
        const Mesh::Face_index f(static_cast<Mesh::Face_index::size_type>(id));
        for (auto v : vertices_around_face(tri.halfedge(f), tri)) {
          const Point3& p = tri.point(v);
          const double c[3] = {toDouble(p.x()), toDouble(p.y()),
                               toDouble(p.z())};
          for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], c[k]);
            hi[k] = std::max(hi[k], c[k]);
          }
        }
      }
      const double dx = hi[0] - lo[0];
      const double dy = hi[1] - lo[1];
      const double dz = hi[2] - lo[2];
      const double clusterDiag = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (clusterDiag > maxClusterDiag)
        continue;

      for (auto id : expanded)
        selected.insert(id);
      ++result.clustersRemoved;
    }

    if (selected.empty()) {
      result.warnings << QStringLiteral("No small complex cluster met the "
                                        "size/count criteria.");
      return true;
    }
    if (selected.size() >= tri.num_faces()) {
      error = QStringLiteral("Every face matched the cleanup criteria; "
                             "nothing would remain. Lower the small-triangle "
                             "ratio or max region size.");
      return false;
    }

    // Remember the original open boundaries so patching only closes holes
    // created by removing detail clusters.
    using EdgeKey = std::array<double, 6>;
    const auto edgeKeyOf = [](const Mesh& m, Mesh::Halfedge_index h) {
      const Point3& a = m.point(m.source(h));
      const Point3& b = m.point(m.target(h));
      std::array<double, 3> ka = {toDouble(a.x()), toDouble(a.y()),
                                  toDouble(a.z())};
      std::array<double, 3> kb = {toDouble(b.x()), toDouble(b.y()),
                                  toDouble(b.z())};
      if (kb < ka)
        std::swap(ka, kb);
      return EdgeKey{ka[0], ka[1], ka[2], kb[0], kb[1], kb[2]};
    };
    std::set<EdgeKey> originalBoundary;
    for (auto e : tri.edges()) {
      if (tri.is_border(e))
        originalBoundary.insert(edgeKeyOf(tri, tri.halfedge(e)));
    }

    Mesh cleaned;
    std::unordered_map<std::size_t, Mesh::Vertex_index> vertexMap;
    for (auto f : tri.faces()) {
      if (selected.count(f.idx()))
        continue;
      std::vector<Mesh::Vertex_index> ring;
      for (auto v : vertices_around_face(tri.halfedge(f), tri)) {
        const auto it = vertexMap.find(v.idx());
        if (it == vertexMap.end()) {
          const auto fresh = cleaned.add_vertex(tri.point(v));
          vertexMap.emplace(v.idx(), fresh);
          ring.push_back(fresh);
        } else {
          ring.push_back(it->second);
        }
      }
      if (ring.size() >= 3)
        cleaned.add_face(ring);
    }
    if (cleaned.num_faces() == 0) {
      error = QStringLiteral("Cleanup removed every face.");
      return false;
    }
    result.facesRemoved =
        static_cast<int>(tri.num_faces() - cleaned.num_faces());

    // Fill only boundary loops that contain at least one edge created by
    // the removal. Existing open borders are left untouched.
    std::vector<Mesh::Halfedge_index> seeds;
    CGAL::extract_boundary_cycles(cleaned, std::back_inserter(seeds));
    for (auto seed : seeds) {
      if (!cleaned.is_border(seed))
        continue;
      bool newHole = false;
      int edgeCount = 0;
      auto h = seed;
      do {
        if (!originalBoundary.count(edgeKeyOf(cleaned, h)))
          newHole = true;
        ++edgeCount;
        h = cleaned.next(h);
      } while (h != seed);
      if (!newHole) {
        ++result.holesSkipped;
        continue;
      }
      if (edgeCount > params.maxHoleEdges) {
        result.warnings << QStringLiteral("Left a %1-edge hole open; it exceeds "
                                          "the max hole-edge limit.")
                               .arg(edgeCount);
        ++result.holesSkipped;
        continue;
      }

      const std::size_t before = cleaned.num_faces();
      std::unordered_set<std::size_t> rimVertices;
      auto hr = seed;
      do {
        rimVertices.insert(cleaned.source(hr).idx());
        hr = cleaned.next(hr);
      } while (hr != seed);

      bool filled = false;
      std::vector<Mesh::Vertex_index> patchVertices;
      try {
        PMP::triangulate_and_refine_hole(
            cleaned, seed,
            params::use_delaunay_triangulation(true)
                .use_2d_constrained_delaunay_triangulation(true)
                .density_control_factor(std::sqrt(2.0))
                .vertex_output_iterator(std::back_inserter(patchVertices)));
        filled = !cleaned.is_border(seed);
      } catch (const std::exception& e) {
        result.warnings << QStringLiteral("Smooth hole patch failed: %1")
                               .arg(QString::fromUtf8(e.what()));
      }
      if (!filled) {
        try {
          PMP::triangulate_hole(cleaned, seed);
          filled = !cleaned.is_border(seed);
        } catch (const std::exception& e) {
          result.warnings << QStringLiteral("Hole patch failed: %1")
                                 .arg(QString::fromUtf8(e.what()));
        }
      }
      if (filled) {
        // Laplacian smoothing of the newly created patch vertices only;
        // the original rim stays fixed, so the patch blends into the
        // surrounding surface without moving the rest of the mesh.
        std::vector<Mesh::Vertex_index> movable;
        for (auto v : patchVertices) {
          if (!rimVertices.count(v.idx()))
            movable.push_back(v);
        }
        for (int iteration = 0; iteration < 3 && !movable.empty(); ++iteration) {
          std::vector<std::pair<Mesh::Vertex_index, Point3>> updates;
          updates.reserve(movable.size());
          for (auto v : movable) {
            if (cleaned.is_isolated(v))
              continue;
            double sx = 0.0, sy = 0.0, sz = 0.0;
            int count = 0;
            for (auto hv : halfedges_around_target(cleaned.halfedge(v), cleaned)) {
              const Point3& p = cleaned.point(cleaned.source(hv));
              sx += toDouble(p.x());
              sy += toDouble(p.y());
              sz += toDouble(p.z());
              ++count;
            }
            if (count == 0)
              continue;
            const Point3& old = cleaned.point(v);
            const double ox = toDouble(old.x());
            const double oy = toDouble(old.y());
            const double oz = toDouble(old.z());
            updates.emplace_back(
                v, makePoint(0.5 * ox + 0.5 * sx / count,
                             0.5 * oy + 0.5 * sy / count,
                             0.5 * oz + 0.5 * sz / count));
          }
          for (const auto& [v, p] : updates)
            cleaned.point(v) = p;
        }
        ++result.holesPatched;
        result.patchFacesAdded +=
            static_cast<int>(cleaned.num_faces() - before);
      } else {
        ++result.holesSkipped;
      }
    }

    if (cleaned.num_faces() == 0) {
      error = QStringLiteral("Autoclean produced an empty mesh.");
      return false;
    }
    mesh = std::move(cleaned);
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

Mesh extractSubmesh(const Mesh& mesh, const QSet<int>& faceIds) {
  Mesh sub;
  std::unordered_map<std::size_t, Mesh::Vertex_index> vmap;
  for (auto f : mesh.faces()) {
    if (!faceIds.contains(static_cast<int>(f.idx())))
      continue;
    // Copy the full vertex ring so polygonal faces survive intact.
    std::vector<Mesh::Vertex_index> ring;
    auto h = mesh.halfedge(f);
    const auto start = h;
    do {
      const auto src = mesh.source(h);
      auto it = vmap.find(src.idx());
      if (it == vmap.end()) {
        const auto fresh = sub.add_vertex(mesh.point(src));
        vmap[src.idx()] = fresh;
        ring.push_back(fresh);
      } else {
        ring.push_back(it->second);
      }
      h = mesh.next(h);
    } while (h != start);
    sub.add_face(ring);
  }
  return sub;
}

bool removeMeshFaces(const Mesh& mesh, const QSet<int>& faceIds, Mesh& out,
                     int& removed, QString& error) {
  error.clear();
  removed = 0;
  out.clear();
  if (faceIds.isEmpty()) {
    error = QStringLiteral("No faces selected.");
    return false;
  }
  // Rebuild from the kept faces instead of mutating the input: the caller
  // needs the original stage intact for the new pipeline node.
  std::unordered_map<std::size_t, Mesh::Vertex_index> vmap;
  int kept = 0;
  for (auto f : mesh.faces()) {
    if (faceIds.contains(static_cast<int>(f.idx()))) {
      ++removed;
      continue;
    }
    // Copy the full vertex ring so polygonal faces survive intact.
    std::vector<Mesh::Vertex_index> ring;
    auto h = mesh.halfedge(f);
    const auto start = h;
    do {
      const auto src = mesh.source(h);
      auto it = vmap.find(src.idx());
      if (it == vmap.end()) {
        const auto fresh = out.add_vertex(mesh.point(src));
        vmap[src.idx()] = fresh;
        ring.push_back(fresh);
      } else {
        ring.push_back(it->second);
      }
      h = mesh.next(h);
    } while (h != start);
    if (!out.add_face(ring).is_valid()) {
      error = QStringLiteral("Failed to copy a surviving face while deleting "
                             "faces.");
      out.clear();
      removed = 0;
      return false;
    }
    ++kept;
  }
  // End safety check: every input face is either kept or reported removed.
  if (kept + removed != static_cast<int>(mesh.num_faces()) ||
      out.num_faces() != static_cast<std::size_t>(kept)) {
    error = QStringLiteral("Face deletion produced an inconsistent mesh.");
    out.clear();
    removed = 0;
    return false;
  }
  if (removed == 0) {
    error = QStringLiteral("None of the selected faces exist in this mesh.");
    out.clear();
    return false;
  }
  return true;
}

bool removeFeatureEdges(const FeatureEdges& edges, const QSet<int>& edgeIds,
                        FeatureEdges& out, int& removed, QString& error) {
  error.clear();
  removed = 0;
  out = FeatureEdges{};
  if (edgeIds.isEmpty()) {
    error = QStringLiteral("No feature edges selected.");
    return false;
  }
  // Compact points as surviving edges are copied; selected edges are absent
  // from the result and so are points no remaining edge references.
  std::vector<int> pointMap(edges.points.size(), -1);
  out.edges.reserve(edges.edges.size());
  for (std::size_t i = 0; i < edges.edges.size(); ++i) {
    if (edgeIds.contains(static_cast<int>(i))) {
      ++removed;
      continue;
    }
    const auto& e = edges.edges[i];
    if (e[0] < 0 || e[1] < 0 ||
        static_cast<std::size_t>(e[0]) >= edges.points.size() ||
        static_cast<std::size_t>(e[1]) >= edges.points.size()) {
      error = QStringLiteral("Feature edge %1 references a missing point.")
                  .arg(i);
      out = FeatureEdges{};
      removed = 0;
      return false;
    }
    int fresh[2];
    for (int k = 0; k < 2; ++k) {
      const int src = e[k];
      if (pointMap[static_cast<std::size_t>(src)] < 0) {
        pointMap[static_cast<std::size_t>(src)] =
            static_cast<int>(out.points.size());
        out.points.push_back(edges.points[static_cast<std::size_t>(src)]);
      }
      fresh[k] = pointMap[static_cast<std::size_t>(src)];
    }
    out.edges.push_back({fresh[0], fresh[1]});
  }
  // End safety check: every input edge is either kept or reported removed.
  if (out.edges.size() + static_cast<std::size_t>(removed) !=
      edges.edges.size()) {
    error = QStringLiteral("Feature-edge deletion produced an inconsistent "
                           "edge set.");
    out = FeatureEdges{};
    removed = 0;
    return false;
  }
  if (removed == 0) {
    error = QStringLiteral(
        "None of the selected feature edges exist in this edge set.");
    out = FeatureEdges{};
    return false;
  }
  return true;
}

Mesh mergeMeshes(const std::vector<const Mesh*>& inputs) {
  Mesh merged;
  for (const Mesh* in : inputs) {
    if (!in)
      continue;
    // Per-input vertex map: coincident vertices across inputs stay split,
    // so Combine never stitches meshes that merely touch.
    std::unordered_map<std::size_t, Mesh::Vertex_index> vmap;
    for (auto f : in->faces()) {
      std::vector<Mesh::Vertex_index> ring;
      auto h = in->halfedge(f);
      const auto start = h;
      do {
        const auto src = in->source(h);
        auto it = vmap.find(src.idx());
        if (it == vmap.end()) {
          const auto fresh = merged.add_vertex(in->point(src));
          vmap[src.idx()] = fresh;
          ring.push_back(fresh);
        } else {
          ring.push_back(it->second);
        }
        h = in->next(h);
      } while (h != start);
      merged.add_face(ring);
    }
  }
  return merged;
}

} // namespace meshbench
