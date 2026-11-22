#include "GmshCadSession.h"
#include "MeshOps.h"

#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

#include <QStringList>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_map>
#include <utility>

#include <gmsh.h>

namespace meshbench {

namespace {

constexpr double kPi = 3.14159265358979323846;

double degreesToRadians(double degrees) { return degrees * kPi / 180.0; }

bool finite(double v) { return std::isfinite(v); }

bool finite3(const std::array<double, 3>& v) {
  return finite(v[0]) && finite(v[1]) && finite(v[2]);
}

bool positive3(const std::array<double, 3>& v) {
  return finite3(v) && v[0] > 0.0 && v[1] > 0.0 && v[2] > 0.0;
}

// Gmsh creates duplicate nodes along OCC seam curves (cone/sphere/cylinder
// parameter wraps). They have identical coordinates but different node tags,
// which makes the converted surface mesh look open. Weld them by coordinate.
constexpr double kWeldTolerance = 1e-8;

struct WeldKey {
  long long x = 0;
  long long y = 0;
  long long z = 0;

  bool operator==(const WeldKey& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct WeldKeyHash {
  std::size_t operator()(const WeldKey& key) const noexcept {
    std::size_t h = std::hash<long long>{}(key.x);
    h ^= std::hash<long long>{}(key.y) + 0x9e3779b97f4a7c15ULL +
         (h << 6) + (h >> 2);
    h ^= std::hash<long long>{}(key.z) + 0x9e3779b97f4a7c15ULL +
         (h << 6) + (h >> 2);
    return h;
  }
};

WeldKey weldKey(double x, double y, double z) {
  return WeldKey{std::llround(x / kWeldTolerance),
                 std::llround(y / kWeldTolerance),
                 std::llround(z / kWeldTolerance)};
}

} // namespace

namespace {

std::vector<CadEntity> fromPairs(const gmsh::vectorpair& pairs) {
  std::vector<CadEntity> out;
  out.reserve(pairs.size());
  for (const auto& [dim, tag] : pairs)
    out.push_back(CadEntity{dim, tag});
  return out;
}

gmsh::vectorpair toPairs(int dim, const std::vector<int>& tags) {
  gmsh::vectorpair pairs;
  pairs.reserve(tags.size());
  for (int tag : tags)
    pairs.emplace_back(dim, tag);
  return pairs;
}

bool validTags(const std::vector<int>& tags, const QString& what,
               QString& error) {
  if (tags.empty()) {
    error = QStringLiteral("%1 must contain at least one tag.").arg(what);
    return false;
  }
  for (int tag : tags) {
    if (tag < 0) {
      error = QStringLiteral("%1 contains a negative tag.").arg(what);
      return false;
    }
  }
  return true;
}

bool normalise(const std::array<double, 3>& v, std::array<double, 3>& out,
               QString& error) {
  if (!finite3(v)) {
    error = QStringLiteral("Direction components must be finite.");
    return false;
  }
  const double len =
      std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (!(len > 1e-15)) {
    error = QStringLiteral("Direction must be non-zero.");
    return false;
  }
  out = {v[0] / len, v[1] / len, v[2] / len};
  return true;
}

} // namespace

struct GmshCadSession::Impl {
  bool active = false;
  double meshSize = 0.2;
};

GmshCadSession::GmshCadSession() : m_impl(std::make_unique<Impl>()) {}
GmshCadSession::~GmshCadSession() { abort(); }

bool GmshCadSession::active() const {
  return m_impl && m_impl->active;
}

double GmshCadSession::meshSize() const {
  return m_impl ? m_impl->meshSize : 0.0;
}

bool GmshCadSession::begin(const QString& importPath, double meshSize,
                           QString& error) {
  error.clear();
  if (!m_impl)
    m_impl = std::make_unique<Impl>();
  if (m_impl->active) {
    error = QStringLiteral("A Gmsh CAD session is already active.");
    return false;
  }
  if (!(meshSize > 0.0) || !finite(meshSize)) {
    error = QStringLiteral("mesh_size must be positive and finite.");
    return false;
  }
  try {
    gmsh::initialize();
    gmsh::option::setNumber("General.Terminal", 0);
    gmsh::clear();
    gmsh::model::add("meshbench_cad");
    if (!importPath.isEmpty()) {
      gmsh::vectorpair out;
      gmsh::model::occ::importShapes(importPath.toStdString(), out, true);
      gmsh::model::occ::synchronize();
    }
    m_impl->meshSize = meshSize;
    m_impl->active = true;
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("Could not start the Gmsh CAD session: %1")
                .arg(QString::fromUtf8(e.what()));
    try {
      gmsh::finalize();
    } catch (...) {
    }
    m_impl->active = false;
    return false;
  }
}

bool GmshCadSession::end(QString& error) {
  error.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("No Gmsh CAD session is active.");
    return false;
  }
  try {
    gmsh::finalize();
    m_impl->active = false;
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("Could not finalise the Gmsh CAD session: %1")
                .arg(QString::fromUtf8(e.what()));
    m_impl->active = false;
    return false;
  }
}

void GmshCadSession::abort() {
  if (!m_impl || !m_impl->active)
    return;
  try {
    gmsh::finalize();
  } catch (...) {
  }
  m_impl->active = false;
}

bool GmshCadSession::setMeshSize(double meshSize, QString& error) {
  error.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("No Gmsh CAD session is active.");
    return false;
  }
  if (!(meshSize > 0.0) || !finite(meshSize)) {
    error = QStringLiteral("mesh_size must be positive and finite.");
    return false;
  }
  m_impl->meshSize = meshSize;
  return true;
}

bool GmshCadSession::addBox(const std::array<double, 3>& origin,
                            const std::array<double, 3>& size, int tag,
                            int& outTag, QString& error) {
  error.clear();
  outTag = -1;
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!finite3(origin) || !positive3(size)) {
    error = QStringLiteral("Box origin must be finite and size positive.");
    return false;
  }
  try {
    outTag = gmsh::model::occ::addBox(origin[0], origin[1], origin[2],
                                      size[0], size[1], size[2], tag);
    gmsh::model::occ::synchronize();
    return outTag > 0;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh addBox failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::addSphere(const std::array<double, 3>& center,
                               double radius, int tag, int& outTag,
                               QString& error) {
  error.clear();
  outTag = -1;
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!finite3(center) || !(radius > 0.0) || !finite(radius)) {
    error = QStringLiteral("Sphere center must be finite and radius positive.");
    return false;
  }
  try {
    outTag = gmsh::model::occ::addSphere(center[0], center[1], center[2],
                                         radius, tag);
    gmsh::model::occ::synchronize();
    return outTag > 0;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh addSphere failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::addEllipsoid(const std::array<double, 3>& center,
                                  const std::array<double, 3>& radii, int tag,
                                  int& outTag, QString& error) {
  error.clear();
  outTag = -1;
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!finite3(center) || !positive3(radii)) {
    error = QStringLiteral("Ellipsoid center must be finite and radii positive.");
    return false;
  }
  try {
    // Gmsh has no ellipsoid primitive: make a unit sphere at the target
    // center and scale it around that center.
    outTag = gmsh::model::occ::addSphere(center[0], center[1], center[2], 1.0,
                                         tag);
    gmsh::model::occ::dilate({{3, outTag}}, center[0], center[1], center[2],
                             radii[0], radii[1], radii[2]);
    gmsh::model::occ::synchronize();
    return outTag > 0;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh ellipsoid failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::addCylinder(
    const std::array<double, 3>& base,
    const std::array<double, 3>& axisDirection, double height, double radius,
    double angleDegrees, int tag, int& outTag, QString& error) {
  error.clear();
  outTag = -1;
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!finite3(base) || !(height > 0.0) || !finite(height) ||
      !(radius > 0.0) || !finite(radius)) {
    error = QStringLiteral("Cylinder base must be finite; height and radius "
                           "must be positive.");
    return false;
  }
  std::array<double, 3> dir;
  if (!normalise(axisDirection, dir, error))
    return false;
  const double angle =
      angleDegrees > 0.0 ? degreesToRadians(angleDegrees) : 2.0 * kPi;
  try {
    outTag = gmsh::model::occ::addCylinder(
        base[0], base[1], base[2], dir[0] * height, dir[1] * height,
        dir[2] * height, radius, tag, angle);
    gmsh::model::occ::synchronize();
    return outTag > 0;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh addCylinder failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::addCone(const std::array<double, 3>& base,
                             const std::array<double, 3>& axisDirection,
                             double height, double radius1, double radius2,
                             double angleDegrees, int tag, int& outTag,
                             QString& error) {
  error.clear();
  outTag = -1;
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!finite3(base) || !(height > 0.0) || !finite(height) ||
      !(radius1 >= 0.0) || !(radius2 >= 0.0) || !finite(radius1) ||
      !finite(radius2)) {
    error = QStringLiteral("Cone base must be finite; height positive; radii "
                           "non-negative.");
    return false;
  }
  std::array<double, 3> dir;
  if (!normalise(axisDirection, dir, error))
    return false;
  const double angle =
      angleDegrees > 0.0 ? degreesToRadians(angleDegrees) : 2.0 * kPi;
  try {
    outTag = gmsh::model::occ::addCone(
        base[0], base[1], base[2], dir[0] * height, dir[1] * height,
        dir[2] * height, radius1, radius2, tag, angle);
    gmsh::model::occ::synchronize();
    return outTag > 0;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh addCone failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::addTorus(const std::array<double, 3>& center,
                              double majorRadius, double minorRadius,
                              double angleDegrees, int tag, int& outTag,
                              QString& error) {
  error.clear();
  outTag = -1;
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!finite3(center) || !(majorRadius > 0.0) ||
      !(minorRadius > 0.0) || !finite(majorRadius) ||
      !finite(minorRadius)) {
    error = QStringLiteral("Torus center must be finite and both radii "
                           "positive.");
    return false;
  }
  const double angle =
      angleDegrees > 0.0 ? degreesToRadians(angleDegrees) : 2.0 * kPi;
  try {
    outTag = gmsh::model::occ::addTorus(center[0], center[1], center[2],
                                        majorRadius, minorRadius, tag, angle);
    gmsh::model::occ::synchronize();
    return outTag > 0;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh addTorus failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::addRectangle(const std::array<double, 3>& origin,
                                  std::array<double, 2> size, int tag,
                                  int& outTag, QString& error) {
  error.clear();
  outTag = -1;
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!finite3(origin) || !(size[0] > 0.0) || !(size[1] > 0.0) ||
      !finite(size[0]) || !finite(size[1])) {
    error = QStringLiteral("Rectangle origin must be finite and size positive.");
    return false;
  }
  try {
    outTag = gmsh::model::occ::addRectangle(origin[0], origin[1], origin[2],
                                            size[0], size[1], tag);
    gmsh::model::occ::synchronize();
    return outTag > 0;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh addRectangle failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::addDisk(const std::array<double, 3>& center,
                             double radiusX, double radiusY,
                             const std::array<double, 3>& zAxis,
                             const std::array<double, 3>& xAxis, int tag,
                             int& outTag, QString& error) {
  error.clear();
  outTag = -1;
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!finite3(center) || !(radiusX > 0.0) || !(radiusY > 0.0) ||
      !finite(radiusX) || !finite(radiusY) || radiusX < radiusY) {
    error = QStringLiteral("Disk radii must be positive and radius_x >= "
                           "radius_y.");
    return false;
  }
  std::vector<double> z;
  std::vector<double> x;
  if (finite3(zAxis) &&
      (std::abs(zAxis[0]) + std::abs(zAxis[1]) + std::abs(zAxis[2]) > 1e-15))
    z = {zAxis[0], zAxis[1], zAxis[2]};
  if (finite3(xAxis) &&
      (std::abs(xAxis[0]) + std::abs(xAxis[1]) + std::abs(xAxis[2]) > 1e-15))
    x = {xAxis[0], xAxis[1], xAxis[2]};
  try {
    outTag = gmsh::model::occ::addDisk(center[0], center[1], center[2],
                                       radiusX, radiusY, tag, z, x);
    gmsh::model::occ::synchronize();
    return outTag > 0;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh addDisk failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::extrude(int dim, const std::vector<int>& tags,
                             const std::array<double, 3>& delta,
                             std::vector<CadEntity>& out, QString& error) {
  error.clear();
  out.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (dim < 0 || dim > 2) {
    error = QStringLiteral("extrude dim must be 0, 1 or 2.");
    return false;
  }
  if (!validTags(tags, QStringLiteral("tags"), error))
    return false;
  if (!finite3(delta) ||
      std::abs(delta[0]) + std::abs(delta[1]) + std::abs(delta[2]) < 1e-15) {
    error = QStringLiteral("extrude translate vector must be non-zero and "
                           "finite.");
    return false;
  }
  try {
    gmsh::vectorpair generated;
    gmsh::model::occ::extrude(toPairs(dim, tags), delta[0], delta[1], delta[2],
                              generated);
    gmsh::model::occ::synchronize();
    out = fromPairs(generated);
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh extrude failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::revolve(
    int dim, const std::vector<int>& tags,
    const std::array<double, 3>& axisPoint,
    const std::array<double, 3>& axisDirection, double angleDegrees,
    std::vector<CadEntity>& out, QString& error) {
  error.clear();
  out.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (dim < 0 || dim > 2) {
    error = QStringLiteral("revolve dim must be 0, 1 or 2.");
    return false;
  }
  if (!validTags(tags, QStringLiteral("tags"), error))
    return false;
  if (!finite3(axisPoint) || !(angleDegrees > 0.0) ||
      !finite(angleDegrees)) {
    error = QStringLiteral("revolve needs a finite axis point and a positive "
                           "angle_degrees.");
    return false;
  }
  std::array<double, 3> dir;
  if (!normalise(axisDirection, dir, error))
    return false;
  try {
    gmsh::vectorpair generated;
    gmsh::model::occ::revolve(toPairs(dim, tags), axisPoint[0], axisPoint[1],
                              axisPoint[2], dir[0], dir[1], dir[2],
                              degreesToRadians(angleDegrees), generated);
    gmsh::model::occ::synchronize();
    out = fromPairs(generated);
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh revolve failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::fillet(const std::vector<int>& volumes,
                            const std::vector<int>& curves, double radius,
                            bool removeVolume, std::vector<CadEntity>& out,
                            QString& error) {
  error.clear();
  out.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!validTags(volumes, QStringLiteral("volumes"), error) ||
      !validTags(curves, QStringLiteral("curves"), error))
    return false;
  if (!(radius > 0.0) || !finite(radius)) {
    error = QStringLiteral("Fillet radius must be positive and finite.");
    return false;
  }
  try {
    gmsh::vectorpair generated;
    gmsh::model::occ::fillet(volumes, curves, {radius}, generated, removeVolume);
    gmsh::model::occ::synchronize();
    out = fromPairs(generated);
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh fillet failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::chamfer(const std::vector<int>& volumes,
                             const std::vector<int>& curves,
                             const std::vector<int>& surfaces, double distance,
                             bool removeVolume, std::vector<CadEntity>& out,
                             QString& error) {
  error.clear();
  out.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!validTags(volumes, QStringLiteral("volumes"), error) ||
      !validTags(curves, QStringLiteral("curves"), error) ||
      !validTags(surfaces, QStringLiteral("surfaces"), error))
    return false;
  if (!(distance > 0.0) || !finite(distance)) {
    error = QStringLiteral("Chamfer distance must be positive and finite.");
    return false;
  }
  try {
    gmsh::vectorpair generated;
    gmsh::model::occ::chamfer(volumes, curves, surfaces, {distance}, generated,
                              removeVolume);
    gmsh::model::occ::synchronize();
    out = fromPairs(generated);
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh chamfer failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::booleanOp(const QString& operation,
                               const std::vector<int>& objects,
                               const std::vector<int>& tools, bool removeObject,
                               bool removeTool, std::vector<CadEntity>& out,
                               QString& error) {
  error.clear();
  out.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!validTags(objects, QStringLiteral("objects"), error) ||
      !validTags(tools, QStringLiteral("tools"), error))
    return false;
  const QString op = operation.toLower();
  if (op != QStringLiteral("cut") && op != QStringLiteral("fuse") &&
      op != QStringLiteral("intersect")) {
    error = QStringLiteral("operation must be cut, fuse or intersect.");
    return false;
  }
  try {
    const gmsh::vectorpair objectPairs = toPairs(3, objects);
    const gmsh::vectorpair toolPairs = toPairs(3, tools);
    gmsh::vectorpair generated;
    std::vector<gmsh::vectorpair> map;
    if (op == QStringLiteral("cut")) {
      gmsh::model::occ::cut(objectPairs, toolPairs, generated, map, -1,
                            removeObject, removeTool);
    } else if (op == QStringLiteral("fuse")) {
      gmsh::model::occ::fuse(objectPairs, toolPairs, generated, map, -1,
                             removeObject, removeTool);
    } else {
      gmsh::model::occ::intersect(objectPairs, toolPairs, generated, map, -1,
                                  removeObject, removeTool);
    }
    gmsh::model::occ::synchronize();
    out = fromPairs(generated);
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh boolean %1 failed: %2")
                .arg(op, QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::translate(int dim, const std::vector<int>& tags,
                               const std::array<double, 3>& delta,
                               QString& error) {
  error.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!validTags(tags, QStringLiteral("tags"), error) || !finite3(delta)) {
    if (error.isEmpty())
      error = QStringLiteral("Translation must be finite.");
    return false;
  }
  try {
    gmsh::model::occ::translate(toPairs(dim, tags), delta[0], delta[1],
                                delta[2]);
    gmsh::model::occ::synchronize();
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh translate failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::rotate(int dim, const std::vector<int>& tags,
                            const std::array<double, 3>& axisPoint,
                            const std::array<double, 3>& axisDirection,
                            double angleDegrees, QString& error) {
  error.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!validTags(tags, QStringLiteral("tags"), error) ||
      !finite3(axisPoint) || !finite(angleDegrees)) {
    if (error.isEmpty())
      error = QStringLiteral("Rotation point and angle must be finite.");
    return false;
  }
  std::array<double, 3> dir;
  if (!normalise(axisDirection, dir, error))
    return false;
  try {
    gmsh::model::occ::rotate(toPairs(dim, tags), axisPoint[0], axisPoint[1],
                             axisPoint[2], dir[0], dir[1], dir[2],
                             degreesToRadians(angleDegrees));
    gmsh::model::occ::synchronize();
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh rotate failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::scale(int dim, const std::vector<int>& tags,
                           const std::array<double, 3>& center,
                           const std::array<double, 3>& factors,
                           QString& error) {
  error.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!validTags(tags, QStringLiteral("tags"), error) || !finite3(center) ||
      !positive3(factors)) {
    if (error.isEmpty())
      error = QStringLiteral("Scale center must be finite and factors "
                             "positive.");
    return false;
  }
  try {
    gmsh::model::occ::dilate(toPairs(dim, tags), center[0], center[1],
                             center[2], factors[0], factors[1], factors[2]);
    gmsh::model::occ::synchronize();
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh scale failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::mirror(int dim, const std::vector<int>& tags,
                            const std::array<double, 4>& plane,
                            QString& error) {
  error.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!validTags(tags, QStringLiteral("tags"), error)) {
    return false;
  }
  for (double v : plane) {
    if (!finite(v)) {
      error = QStringLiteral("Mirror plane coefficients must be finite.");
      return false;
    }
  }
  const double normalLength =
      std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] +
                plane[2] * plane[2]);
  if (!(normalLength > 1e-15)) {
    error = QStringLiteral("Mirror plane normal must be non-zero.");
    return false;
  }
  try {
    gmsh::model::occ::mirror(toPairs(dim, tags), plane[0], plane[1], plane[2],
                             plane[3]);
    gmsh::model::occ::synchronize();
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh mirror failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::copy(int dim, const std::vector<int>& tags,
                          std::vector<CadEntity>& out, QString& error) {
  error.clear();
  out.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!validTags(tags, QStringLiteral("tags"), error))
    return false;
  try {
    gmsh::vectorpair generated;
    gmsh::model::occ::copy(toPairs(dim, tags), generated);
    gmsh::model::occ::synchronize();
    out = fromPairs(generated);
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh copy failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::remove(int dim, const std::vector<int>& tags,
                            bool recursive, QString& error) {
  error.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (!validTags(tags, QStringLiteral("tags"), error))
    return false;
  try {
    gmsh::model::occ::remove(toPairs(dim, tags), recursive);
    gmsh::model::occ::synchronize();
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh remove failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::importShapes(const QString& path, bool highestDimOnly,
                                  std::vector<CadEntity>& out, QString& error) {
  error.clear();
  out.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (path.isEmpty()) {
    error = QStringLiteral("path must name a STEP/IGES/BREP file.");
    return false;
  }
  try {
    gmsh::vectorpair generated;
    gmsh::model::occ::importShapes(path.toStdString(), generated,
                                   highestDimOnly);
    gmsh::model::occ::synchronize();
    out = fromPairs(generated);
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh importShapes failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::exportModel(const QString& path, QString& error) {
  error.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  if (path.isEmpty()) {
    error = QStringLiteral("path must name an output file (.step, .brep, ...).");
    return false;
  }
  try {
    gmsh::write(path.toStdString());
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh export failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::listEntities(int dim, QVector<CadEntityInfo>& out,
                                  QString& error) {
  error.clear();
  out.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  try {
    gmsh::vectorpair entities;
    gmsh::model::occ::getEntities(entities, dim);
    out.reserve(static_cast<int>(entities.size()));
    for (const auto& [d, tag] : entities) {
      CadEntityInfo info;
      info.entity = CadEntity{d, tag};
      std::string type;
      gmsh::model::getEntityType(d, tag, type);
      info.type = QString::fromStdString(type);
      gmsh::model::occ::getBoundingBox(
          d, tag, info.bbox[0], info.bbox[1], info.bbox[2], info.bbox[3],
          info.bbox[4], info.bbox[5]);
      out.append(info);
    }
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh getEntities failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::entityCount(int minDim, int& count, QString& error) {
  error.clear();
  count = 0;
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("Start a session with gmsh_session_begin first.");
    return false;
  }
  try {
    gmsh::vectorpair entities;
    gmsh::model::occ::getEntities(entities);
    for (const auto& [dim, tag] : entities) {
      Q_UNUSED(tag);
      if (dim >= minDim)
        ++count;
    }
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("gmsh getEntities failed: %1")
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::hasSurfaceGeometry(bool& hasGeometry, QString& error) {
  int count = 0;
  if (!entityCount(2, count, error)) {
    hasGeometry = false;
    return false;
  }
  hasGeometry = count > 0;
  return true;
}

bool GmshCadSession::meshToMesh(Mesh& mesh, QString& error) {
  error.clear();
  mesh.clear();
  if (!m_impl || !m_impl->active) {
    error = QStringLiteral("No Gmsh CAD session is active.");
    return false;
  }
  bool hasGeometry = false;
  if (!hasSurfaceGeometry(hasGeometry, error))
    return false;
  if (!hasGeometry) {
    error = QStringLiteral("The CAD model has no surface or volume geometry "
                           "to mesh yet.");
    return false;
  }
  try {
    gmsh::model::mesh::clear();
    gmsh::option::setNumber("Mesh.CharacteristicLengthMin", m_impl->meshSize);
    gmsh::option::setNumber("Mesh.CharacteristicLengthMax", m_impl->meshSize);
    // Surface mesh is enough for display; the OCC model stays intact for
    // subsequent low-level operations.
    gmsh::model::mesh::generate(2);

    std::vector<std::size_t> nodeTags;
    std::vector<double> coordinates;
    std::vector<double> parametric;
    gmsh::model::mesh::getNodes(nodeTags, coordinates, parametric, -1, -1,
                                false, false);
    std::vector<std::size_t> elementTags;
    std::vector<std::size_t> elementNodes;
    gmsh::model::mesh::getElementsByType(2, elementTags, elementNodes);
    if (elementNodes.empty()) {
      error = QStringLiteral("Gmsh generated no surface triangles at "
                             "mesh_size %1.")
                  .arg(m_impl->meshSize);
      return false;
    }
    if (coordinates.size() != nodeTags.size() * 3) {
      error = QStringLiteral("Gmsh returned inconsistent node coordinates.");
      return false;
    }

    std::unordered_map<std::size_t, std::size_t> nodeIndex;
    nodeIndex.reserve(nodeTags.size());
    for (std::size_t i = 0; i < nodeTags.size(); ++i)
      nodeIndex.emplace(nodeTags[i], i);

    // Build an oriented polygon soup. Weld gmsh nodes that share coordinates
    // (OCC seam curves produce duplicate tags). Gmsh can also emit locally
    // inconsistent triangle winding for extruded surfaces; orient_polygon_soup
    // fixes it (and splits non-manifold vertices when necessary).
    std::vector<Point3> points;
    std::vector<std::vector<std::size_t>> polygons;
    std::unordered_map<WeldKey, std::size_t, WeldKeyHash> usedPoint;
    usedPoint.reserve(nodeTags.size());
    for (std::size_t i = 0; i + 2 < elementNodes.size(); i += 3) {
      std::vector<std::size_t> triangle;
      triangle.reserve(3);
      bool degenerate = false;
      for (std::size_t k = 0; k < 3; ++k) {
        const std::size_t node = elementNodes[i + k];
        const auto it = nodeIndex.find(node);
        if (it == nodeIndex.end()) {
          error = QStringLiteral("Gmsh element references an unknown node.");
          return false;
        }
        const std::size_t base = it->second * 3;
        const double x = coordinates[base];
        const double y = coordinates[base + 1];
        const double z = coordinates[base + 2];
        if (!finite(x) || !finite(y) || !finite(z)) {
          error = QStringLiteral("Gmsh returned a non-finite node coordinate.");
          return false;
        }
        const WeldKey key = weldKey(x, y, z);
        auto used = usedPoint.find(key);
        if (used == usedPoint.end()) {
          points.push_back(makePoint(x, y, z));
          used = usedPoint.emplace(key, points.size() - 1).first;
        }
        triangle.push_back(used->second);
      }
      if (triangle[0] == triangle[1] || triangle[1] == triangle[2] ||
          triangle[2] == triangle[0])
        degenerate = true;
      if (!degenerate)
        polygons.push_back(std::move(triangle));
    }
    if (polygons.empty()) {
      error = QStringLiteral("Gmsh produced only degenerate triangles.");
      return false;
    }

    CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons);
    if (!CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(
            polygons)) {
      error = QStringLiteral("The generated CAD surface is not a valid "
                             "triangle mesh.");
      return false;
    }
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(
        points, polygons, mesh);
    if (mesh.num_vertices() == 0 || mesh.num_faces() == 0) {
      error = QStringLiteral("Gmsh mesh conversion produced an empty mesh.");
      return false;
    }
    // Booleans later need outward-facing closed shells. OCC cut results can
    // come back with inverted normals, so orient every closed component.
    if (CGAL::is_closed(mesh)) {
      CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(mesh);
    }
    return true;
  } catch (const std::exception& e) {
    error = QStringLiteral("Gmsh meshing failed at mesh_size %1: %2")
                .arg(m_impl->meshSize)
                .arg(QString::fromUtf8(e.what()));
    return false;
  }
}

bool GmshCadSession::meshToStl(const QString& path, QString& error) {
  error.clear();
  if (path.isEmpty()) {
    error = QStringLiteral("No output path given.");
    return false;
  }
  Mesh mesh;
  if (!meshToMesh(mesh, error))
    return false;
  // CGAL writes STL with the now-consistent orientation, so MeshBench's
  // read_STL-based loader can read the file back.
  return saveMesh(path, mesh, error);
}


} // namespace meshbench
