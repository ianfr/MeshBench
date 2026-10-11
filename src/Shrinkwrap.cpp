#include "Shrinkwrap.h"

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/boost/graph/iterator.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

namespace meshbench {
namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

using Primitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
using Traits = CGAL::AABB_traits<Kernel, Primitive>;
using Tree = CGAL::AABB_tree<Traits>;

struct DVec {
  double x, y, z;
};

inline DVec toDVec(const Point3& p) {
  return {toDouble(p.x()), toDouble(p.y()), toDouble(p.z())};
}

} // namespace

Mesh makeEnclosingSphere(const Mesh& target, int subdivisions) {
  subdivisions = std::clamp(subdivisions, 0, 6);
  // Unit icosahedron.
  const double t = (1.0 + std::sqrt(5.0)) / 2.0;
  std::vector<DVec> verts = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0},
                             {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t},
                             {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
  for (auto& v : verts) {
    const double l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    v = {v.x / l, v.y / l, v.z / l};
  }
  std::vector<std::array<std::size_t, 3>> faces = {
      {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
      {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
      {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
      {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
  // Shared edges must share their midpoint vertex, otherwise the sphere
  // gets duplicate vertices, cracks, and broken smoothing neighbourhoods.
  std::map<std::pair<std::size_t, std::size_t>, std::size_t> edgeMid;
  auto midpoint = [&](std::size_t a, std::size_t b) {
    const auto key = std::minmax(a, b);
    auto it = edgeMid.find(key);
    if (it != edgeMid.end())
      return it->second;
    DVec m{(verts[a].x + verts[b].x) / 2, (verts[a].y + verts[b].y) / 2,
           (verts[a].z + verts[b].z) / 2};
    const double l = std::sqrt(m.x * m.x + m.y * m.y + m.z * m.z);
    verts.push_back({m.x / l, m.y / l, m.z / l});
    const std::size_t id = verts.size() - 1;
    edgeMid[key] = id;
    return id;
  };
  for (int s = 0; s < subdivisions; ++s) {
    edgeMid.clear();
    std::vector<std::array<std::size_t, 3>> refined;
    refined.reserve(faces.size() * 4);
    for (const auto& f : faces) {
      const std::size_t ab = midpoint(f[0], f[1]);
      const std::size_t bc = midpoint(f[1], f[2]);
      const std::size_t ca = midpoint(f[2], f[0]);
      refined.push_back({f[0], ab, ca});
      refined.push_back({f[1], bc, ab});
      refined.push_back({f[2], ca, bc});
      refined.push_back({ab, bc, ca});
    }
    faces = std::move(refined);
  }

  // Fit around the target bbox.
  DVec c{0, 0, 0};
  double radius = 1.0;
  try {
    const auto bb = PMP::bbox(target);
    c = {(toDouble(bb.xmin()) + toDouble(bb.xmax())) / 2,
         (toDouble(bb.ymin()) + toDouble(bb.ymax())) / 2,
         (toDouble(bb.zmin()) + toDouble(bb.zmax())) / 2};
    const double dx = toDouble(bb.xmax()) - toDouble(bb.xmin());
    const double dy = toDouble(bb.ymax()) - toDouble(bb.ymin());
    const double dz = toDouble(bb.zmax()) - toDouble(bb.zmin());
    radius = 0.5 * std::sqrt(dx * dx + dy * dy + dz * dz) * 1.3 + 1e-6;
  } catch (...) {
  }

  Mesh sphere;
  std::vector<Mesh::Vertex_index> idx(verts.size());
  for (std::size_t i = 0; i < verts.size(); ++i) {
    idx[i] = sphere.add_vertex(makePoint(c.x + verts[i].x * radius,
                                         c.y + verts[i].y * radius,
                                         c.z + verts[i].z * radius));
  }
  for (const auto& f : faces)
    sphere.add_face(idx[f[0]], idx[f[1]], idx[f[2]]);
  return sphere;
}

bool snapShrinkwrap(const Mesh& target,
                    const std::unordered_map<std::size_t, double>& weights,
                    Mesh& wrap, const ShrinkParams& params, QString& error) {
  if (target.num_faces() == 0 || wrap.num_vertices() == 0) {
    error = QStringLiteral("Target and wrap meshes must be non-empty.");
    return false;
  }
  if (!(params.step > 0) || params.step > 1.0) {
    error = QStringLiteral("Step must be in (0, 1].");
    return false;
  }
  const double smooth = std::clamp(params.smooth, 0.0, 0.95);
  const double uniform = std::clamp(params.uniformity, 0.0, 1.0);
  const int iters = std::clamp(params.iterations, 1, 1000);

  Tree tree(target.faces().begin(), target.faces().end(), target);
  tree.accelerate_distance_queries();

  // Dense working arrays (wrap is a fresh sphere: indices are dense).
  std::size_t maxIdx = 0;
  for (auto v : wrap.vertices())
    maxIdx = std::max(maxIdx, static_cast<std::size_t>(v.idx()));
  std::vector<DVec> pos(maxIdx + 1), attract(maxIdx + 1), next(maxIdx + 1),
      even(maxIdx + 1);
  // Per-vertex target-surface normals (unit), refreshed each iteration from
  // the closest-face hit; used to keep even-spacing moves tangential.
  std::vector<DVec> normals(maxIdx + 1, {0, 0, 1});
  std::vector<std::vector<std::size_t>> neighbours(maxIdx + 1);
  for (auto v : wrap.vertices()) {
    pos[v.idx()] = toDVec(wrap.point(v));
    for (auto h : halfedges_around_target(v, wrap))
      neighbours[v.idx()].push_back(wrap.source(h).idx());
  }

  auto weightOf = [&](std::size_t faceIdx) {
    auto it = weights.find(faceIdx);
    return it != weights.end() ? std::max(0.0, it->second) : 1.0;
  };

  // Unit normal of a target triangle, for tangential projection.
  auto faceNormal = [&](Mesh::Face_index f) {
    auto h = target.halfedge(f);
    const DVec a = toDVec(target.point(target.source(h)));
    const DVec b = toDVec(target.point(target.target(h)));
    const DVec c = toDVec(target.point(target.target(target.next(h))));
    DVec n{(b.y - a.y) * (c.z - a.z) - (b.z - a.z) * (c.y - a.y),
           (b.z - a.z) * (c.x - a.x) - (b.x - a.x) * (c.z - a.z),
           (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)};
    const double l = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (l > 0)
      n = {n.x / l, n.y / l, n.z / l};
    else
      n = {0, 0, 1};
    return n;
  };

  for (int it = 0; it < iters; ++it) {
    // Attract toward the closest surface point, scaled by face weight.
    for (auto v : wrap.vertices()) {
      const std::size_t i = v.idx();
      const Point3 p = makePoint(pos[i].x, pos[i].y, pos[i].z);
      const auto hit = tree.closest_point_and_primitive(p);
      const DVec q = toDVec(hit.first);
      normals[i] = faceNormal(hit.second);
      const double w = weightOf(hit.second.idx());
      const double k = std::min(1.0, params.step * w);
      attract[i] = {pos[i].x + k * (q.x - pos[i].x),
                    pos[i].y + k * (q.y - pos[i].y),
                    pos[i].z + k * (q.z - pos[i].z)};
    }
    // Laplacian smoothing pass keeps the wrap a clean membrane.
    for (auto v : wrap.vertices()) {
      const std::size_t i = v.idx();
      const auto& nb = neighbours[i];
      if (nb.empty() || smooth == 0.0) {
        next[i] = attract[i];
        continue;
      }
      DVec avg{0, 0, 0};
      for (auto j : nb) {
        avg.x += attract[j].x;
        avg.y += attract[j].y;
        avg.z += attract[j].z;
      }
      const double n = static_cast<double>(nb.size());
      next[i] = {attract[i].x + smooth * (avg.x / n - attract[i].x),
                 attract[i].y + smooth * (avg.y / n - attract[i].y),
                 attract[i].z + smooth * (avg.z / n - attract[i].z)};
    }
    if (uniform == 0.0) {
      pos.swap(next);
      continue;
    }
    // Even-spacing pass: relax toward the neighbour centroid, projected
    // onto the tangent plane so vertices spread out without sinking.
    for (auto v : wrap.vertices()) {
      const std::size_t i = v.idx();
      const auto& nb = neighbours[i];
      if (nb.empty()) {
        even[i] = next[i];
        continue;
      }
      DVec avg{0, 0, 0};
      for (auto j : nb) {
        avg.x += next[j].x;
        avg.y += next[j].y;
        avg.z += next[j].z;
      }
      const double n = static_cast<double>(nb.size());
      DVec d{avg.x / n - next[i].x, avg.y / n - next[i].y,
             avg.z / n - next[i].z};
      const DVec& un = normals[i];
      const double along = d.x * un.x + d.y * un.y + d.z * un.z;
      even[i] = {next[i].x + uniform * (d.x - along * un.x),
                 next[i].y + uniform * (d.y - along * un.y),
                 next[i].z + uniform * (d.z - along * un.z)};
    }
    pos.swap(even);
  }

  for (auto v : wrap.vertices()) {
    const DVec& p = pos[v.idx()];
    wrap.point(v) = makePoint(p.x, p.y, p.z);
  }
  return true;
}

} // namespace meshbench
