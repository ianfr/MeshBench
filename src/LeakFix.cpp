#include "LeakFix.h"

#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/boost/graph/iterator.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <random>

namespace meshbench {
namespace PMP = CGAL::Polygon_mesh_processing;

namespace {

struct Vec3 {
  double x, y, z;
};

struct BBox {
  Vec3 mn{1e300, 1e300, 1e300}, mx{-1e300, -1e300, -1e300};
  void add(const Vec3& p) {
    mn.x = std::min(mn.x, p.x);
    mn.y = std::min(mn.y, p.y);
    mn.z = std::min(mn.z, p.z);
    mx.x = std::max(mx.x, p.x);
    mx.y = std::max(mx.y, p.y);
    mx.z = std::max(mx.z, p.z);
  }
};

inline Vec3 toVec(const Point3& p) {
  return {toDouble(p.x()), toDouble(p.y()), toDouble(p.z())};
}

// Triangle vs axis-aligned voxel-face quad. The quad lies in the plane
// `axis == plane` spanning [lo1, hi1] x [lo2, hi2] in the other axes.
// True iff the triangle pierces the quad: a triangle edge crosses the
// quad interior, a quad edge pierces the triangle, or they overlap
// coplanar. Blocking faces (not cells) is what keeps small holes open:
// rim triangles around a hole rarely cover every face quad of the gap.
namespace {
inline double getComp(const Vec3& v, int axis) {
  return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}
} // namespace

bool segTriHit(const Vec3& p, const Vec3& q, const Vec3& a, const Vec3& b,
               const Vec3& c, double tol) {
  // Moller-Trumbore for a segment (not a ray). The degeneracy epsilon is
  // scaled to the geometry: an absolute tolerance silently drops genuine
  // grazing piercings of small triangles (false open faces = false leaks).
  const double ex = q.x - p.x, ey = q.y - p.y, ez = q.z - p.z;
  const double e1x = b.x - a.x, e1y = b.y - a.y, e1z = b.z - a.z;
  const double e2x = c.x - a.x, e2y = c.y - a.y, e2z = c.z - a.z;
  const double px = ey * e2z - ez * e2y, py = ez * e2x - ex * e2z,
               pz = ex * e2y - ey * e2x;
  const double det = e1x * px + e1y * py + e1z * pz;
  const double mag = std::sqrt(ex * ex + ey * ey + ez * ez) *
                     std::sqrt(e1x * e1x + e1y * e1y + e1z * e1z) *
                     std::sqrt(e2x * e2x + e2y * e2y + e2z * e2z);
  if (std::fabs(det) < 1e-12 * mag + tol * tol * tol)
    return false;
  const double inv = 1.0 / det;
  const double tx = p.x - a.x, ty = p.y - a.y, tz = p.z - a.z;
  const double u = (tx * px + ty * py + tz * pz) * inv;
  if (u < 0.0 || u > 1.0)
    return false;
  const double qx = ty * e1z - tz * e1y, qy = tz * e1x - tx * e1z,
               qz = tx * e1y - ty * e1x;
  const double v = (ex * qx + ey * qy + ez * qz) * inv;
  if (v < 0.0 || u + v > 1.0)
    return false;
  const double t = (e2x * qx + e2y * qy + e2z * qz) * inv;
  return t >= 0.0 && t <= 1.0;
}

bool triOverlapsFaceQuad(const Vec3& v0, const Vec3& v1, const Vec3& v2,
                         int axis, double plane, double lo1, double hi1,
                         double lo2, double hi2, double tol) {
  const int b = (axis + 1) % 3, c = (axis + 2) % 3;
  const Vec3 vs[3] = {v0, v1, v2};
  // 2D segment vs rect overlap (for edges running along the face plane).
  auto segRectHit = [&](double p1, double p2, double q1, double q2) {
    if (p1 >= lo1 - tol && p1 <= hi1 + tol && p2 >= lo2 - tol &&
        p2 <= hi2 + tol)
      return true;
    if (q1 >= lo1 - tol && q1 <= hi1 + tol && q2 >= lo2 - tol &&
        q2 <= hi2 + tol)
      return true;
    // Crossings with the four rect edges.
    const double r[4][4] = {{lo1, lo2, hi1, lo2},
                            {hi1, lo2, hi1, hi2},
                            {hi1, hi2, lo1, hi2},
                            {lo1, hi2, lo1, lo2}};
    for (int f = 0; f < 4; ++f) {
      const double d = (q1 - p1) * (r[f][3] - r[f][1]) -
                       (q2 - p2) * (r[f][2] - r[f][0]);
      if (std::fabs(d) < 1e-18)
        continue; // parallel: endpoint test above suffices
      const double t = ((r[f][0] - p1) * (r[f][3] - r[f][1]) -
                        (r[f][1] - p2) * (r[f][2] - r[f][0])) /
                       d;
      const double u = ((r[f][0] - p1) * (q2 - p2) - (r[f][1] - p2) * (q1 - p1)) / d;
      if (t >= -1e-9 && t <= 1.0 + 1e-9 && u >= -1e-9 && u <= 1.0 + 1e-9)
        return true;
    }
    return false;
  };
  // 1. Triangle edges piercing the quad.
  for (int e = 0; e < 3; ++e) {
    const Vec3& p = vs[e];
    const Vec3& q = vs[(e + 1) % 3];
    const double dp = getComp(p, axis) - plane;
    const double dq = getComp(q, axis) - plane;
    if (std::fabs(dp) < tol && std::fabs(dq) < tol) {
      // Edge runs along the plane: test the whole segment, not one end.
      if (segRectHit(getComp(p, b), getComp(p, c), getComp(q, b),
                     getComp(q, c)))
        return true;
      continue;
    }
    if (dp * dq > 0)
      continue; // strictly same side
    const double denom = dp - dq;
    const double t = dp / denom;
    const double h1 =
        getComp(p, b) + t * (getComp(q, b) - getComp(p, b));
    const double h2 =
        getComp(p, c) + t * (getComp(q, c) - getComp(p, c));
    if (h1 >= lo1 - tol && h1 <= hi1 + tol && h2 >= lo2 - tol &&
        h2 <= hi2 + tol)
      return true;
  }
  // Quad corners for the edge tests below.
  Vec3 corner[4];
  for (int i = 0; i < 4; ++i) {
    corner[i] = {0, 0, 0};
    double* comps[3] = {&corner[i].x, &corner[i].y, &corner[i].z};
    *comps[axis] = plane;
    *comps[b] = (i % 2 == 0) ? lo1 : hi1;
    *comps[c] = (i < 2) ? lo2 : hi2;
  }
  // 2. Quad edges piercing the triangle. NB: with corners ordered
  // (lo,lo),(hi,lo),(lo,hi),(hi,hi), consecutive pairs are NOT all edges
  // (two would be diagonals), so spell the boundary out explicitly.
  const int qe[4][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}};
  for (int e = 0; e < 4; ++e) {
    if (segTriHit(corner[qe[e][0]], corner[qe[e][1]], v0, v1, v2, tol))
      return true;
  }
  // 3. Coplanar overlap: project to 2D, triangle vs AABB.
  {
    const double nx = (v1.y - v0.y) * (v2.z - v0.z) - (v1.z - v0.z) * (v2.y - v0.y);
    const double ny = (v1.z - v0.z) * (v2.x - v0.x) - (v1.x - v0.x) * (v2.z - v0.z);
    const double nz = (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x);
    const double nl = std::sqrt(nx * nx + ny * ny + nz * nz);
    double nc = axis == 0 ? nx : (axis == 1 ? ny : nz);
    if (nl > 0 && std::fabs(nc) / nl > 0.99) {
      // Distance of triangle plane from the face plane.
      const double pn = axis == 0 ? nx : (axis == 1 ? ny : nz);
      const double dist =
          std::fabs(nx * v0.x + ny * v0.y + nz * v0.z - plane * pn) / nl;
      if (dist < tol) {
        auto inBox = [&](const Vec3& p) {
          const double u = getComp(p, b), w = getComp(p, c);
          return u >= lo1 - tol && u <= hi1 + tol && w >= lo2 - tol &&
                 w <= hi2 + tol;
        };
        if (inBox(v0) || inBox(v1) || inBox(v2))
          return true;
        // Box corner inside triangle (2D sign test).
        for (int i = 0; i < 4; ++i) {
          const double px[2] = {getComp(corner[i], b), getComp(corner[i], c)};
          const double ax[2] = {getComp(v0, b), getComp(v0, c)};
          const double bx[2] = {getComp(v1, b), getComp(v1, c)};
          const double cx[2] = {getComp(v2, b), getComp(v2, c)};
          const double d1 = (px[0] - bx[0]) * (ax[1] - bx[1]) -
                            (ax[0] - bx[0]) * (px[1] - bx[1]);
          const double d2 = (px[0] - cx[0]) * (bx[1] - cx[1]) -
                            (bx[0] - cx[0]) * (px[1] - cx[1]);
          const double d3 = (px[0] - ax[0]) * (cx[1] - ax[1]) -
                            (cx[0] - ax[0]) * (px[1] - ax[1]);
          const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
          const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
          if (!(neg && pos))
            return true;
        }
        // 2D edge crossings.
        auto segX = [](double p1x, double p1y, double p2x, double p2y,
                       double p3x, double p3y, double p4x, double p4y) {
          const double d = (p2x - p1x) * (p4y - p3y) - (p2y - p1y) * (p4x - p3x);
          if (std::fabs(d) < 1e-18)
            return false;
          const double t = ((p3x - p1x) * (p4y - p3y) - (p3y - p1y) * (p4x - p3x)) / d;
          const double u = ((p3x - p1x) * (p2y - p1y) - (p3y - p1y) * (p2x - p1x)) / d;
          return t >= 0 && t <= 1 && u >= 0 && u <= 1;
        };
        const double t2[3][2] = {{getComp(v0, b), getComp(v0, c)},
                                 {getComp(v1, b), getComp(v1, c)},
                                 {getComp(v2, b), getComp(v2, c)}};
        const double r2[4][2] = {{lo1, lo2}, {hi1, lo2}, {hi1, hi2}, {lo1, hi2}};
        for (int e = 0; e < 3; ++e) {
          for (int f = 0; f < 4; ++f) {
            if (segX(t2[e][0], t2[e][1], t2[(e + 1) % 3][0], t2[(e + 1) % 3][1],
                     r2[f][0], r2[f][1], r2[(f + 1) % 4][0], r2[(f + 1) % 4][1]))
              return true;
          }
        }
      }
    }
  }
  return false;
}

} // namespace

namespace detail {
// Defined here, outside the anonymous namespace above, so this matches the
// meshbench::detail declaration in LeakFix.h (a definition inside the
// anonymous namespace would get internal linkage and not satisfy it).
bool triOverlapsFaceQuad(const double a[3], const double b[3],
                         const double c[3], int axis, double plane, double lo1,
                         double hi1, double lo2, double hi2, double tol) {
  return triOverlapsFaceQuad(Vec3{a[0], a[1], a[2]}, Vec3{b[0], b[1], b[2]},
                             Vec3{c[0], c[1], c[2]}, axis, plane, lo1, hi1,
                             lo2, hi2, tol);
}
} // namespace detail

struct VoxelGrid {
  int n = 0; // resolution per axis
  Vec3 origin; // grid min corner
  Vec3 size;   // cell size
  // Blocked voxel FACES (snappyHexMesh selectIntersectedFaces analogue):
  // bx holds x-normal faces (n+1) x n x n, etc. Cells are never blocked,
  // so narrow holes are not paved over by the cells around their rim.
  std::vector<char> bx, by, bz;
  std::vector<int> region;   // n^3, -1 = unvisited

  int idx(int i, int j, int k) const { return (i * n + j) * n + k; }
  int fidx(int i, int j, int k) const { return (i * n + j) * n + k; }
  bool inside(int i, int j, int k) const {
    return i >= 0 && i < n && j >= 0 && j < n && k >= 0 && k < n;
  }
  Vec3 center(int i, int j, int k) const {
    return {origin.x + (i + 0.5) * size.x, origin.y + (j + 0.5) * size.y,
            origin.z + (k + 0.5) * size.z};
  }
  // Is the step from cell (i,j,k) along direction a (0:+x,1:-x,2:+y,3:-y,
  // 4:+z,5:-z) open? Matches the di/dj/dk neighbour tables below.
  bool faceOpen(int i, int j, int k, int a) const {
    switch (a) {
    case 0:
      return !bx[fidx(i + 1, j, k)];
    case 1:
      return !bx[fidx(i, j, k)];
    case 2:
      return !by[fidx(i, j + 1, k)];
    case 3:
      return !by[fidx(i, j, k)];
    case 4:
      return !bz[fidx(i, j, k + 1)];
    default:
      return !bz[fidx(i, j, k)];
    }
  }
  // findCell analogue: cell containing p, clamped into the grid.
  void locate(const std::array<double, 3>& p, int& i, int& j, int& k) const {
    i = static_cast<int>((p[0] - origin.x) / size.x);
    j = static_cast<int>((p[1] - origin.y) / size.y);
    k = static_cast<int>((p[2] - origin.z) / size.z);
    i = std::clamp(i, 0, n - 1);
    j = std::clamp(j, 0, n - 1);
    k = std::clamp(k, 0, n - 1);
  }
  bool pointInGrid(const std::array<double, 3>& p) const {
    return p[0] >= origin.x && p[0] <= origin.x + n * size.x &&
           p[1] >= origin.y && p[1] <= origin.y + n * size.y &&
           p[2] >= origin.z && p[2] <= origin.z + n * size.z;
  }
};

LeakProbeResult detectLeaks(const Mesh& mesh,
                            const std::vector<std::array<double, 3>>& inside,
                            const std::vector<std::array<double, 3>>& outside,
                            const ProbeParams& params) {
  LeakProbeResult result;
  if (mesh.num_faces() == 0) {
    result.warnings << QStringLiteral("Mesh is empty; nothing to probe.");
    return result;
  }
  if (inside.empty() && outside.empty()) {
    result.warnings << QStringLiteral("No inside/outside points supplied.");
    return result;
  }

  const int gridRes = std::clamp(params.gridRes, 4, 128);

  // --- Bounding box + padding (background mesh extent) ---
  BBox bb;
  for (auto v : mesh.vertices())
    bb.add(toVec(mesh.point(v)));
  Vec3 diag{bb.mx.x - bb.mn.x, bb.mx.y - bb.mn.y, bb.mx.z - bb.mn.z};
  double dl = std::max({diag.x, diag.y, diag.z});
  if (!(dl > 0)) {
    result.warnings << QStringLiteral("Degenerate mesh bounding box.");
    return result;
  }
  const double pad = 0.1 * dl + 1e-9;
  VoxelGrid g;
  g.n = gridRes;
  g.origin = {bb.mn.x - pad, bb.mn.y - pad, bb.mn.z - pad};
  const double span = dl + 2 * pad;
  g.size = {span / gridRes, span / gridRes, span / gridRes};
  const int n3 = gridRes * gridRes * gridRes;
  g.bx.assign(static_cast<size_t>(gridRes + 1) * gridRes * gridRes, 0);
  g.by.assign(static_cast<size_t>(gridRes + 1) * gridRes * gridRes, 0);
  g.bz.assign(static_cast<size_t>(gridRes + 1) * gridRes * gridRes, 0);
  g.region.assign(n3, -1);
  result.totalCells = n3;
  result.cellSize = span / gridRes;
  result.gridRes = gridRes;
  result.gridOrigin = {g.origin.x, g.origin.y, g.origin.z};
  const double tol = span * 1e-9;

  // --- selectIntersectedFaces analogue: block pierced voxel faces ---
  for (auto f : mesh.faces()) {
    auto h = mesh.halfedge(f);
    const Vec3 v[3] = {toVec(mesh.point(mesh.source(h))),
                       toVec(mesh.point(mesh.target(h))),
                       toVec(mesh.point(mesh.target(mesh.next(h))))};
    BBox tb;
    tb.add(v[0]);
    tb.add(v[1]);
    tb.add(v[2]);
    // X-normal faces: plane x = ox + i*dx, spanning whole j/k cells.
    {
      const int i0 = std::clamp(static_cast<int>(std::floor((tb.mn.x - g.origin.x) / g.size.x)), 0, gridRes);
      const int i1 = std::clamp(static_cast<int>(std::ceil((tb.mx.x - g.origin.x) / g.size.x)), 0, gridRes);
      const int j0 = std::clamp(static_cast<int>(std::floor((tb.mn.y - g.origin.y) / g.size.y)), 0, gridRes - 1);
      const int j1 = std::clamp(static_cast<int>(std::floor((tb.mx.y - g.origin.y) / g.size.y)), 0, gridRes - 1);
      const int k0 = std::clamp(static_cast<int>(std::floor((tb.mn.z - g.origin.z) / g.size.z)), 0, gridRes - 1);
      const int k1 = std::clamp(static_cast<int>(std::floor((tb.mx.z - g.origin.z) / g.size.z)), 0, gridRes - 1);
      for (int i = i0; i <= i1; ++i)
        for (int j = j0; j <= j1; ++j)
          for (int k = k0; k <= k1; ++k) {
            if (g.bx[g.fidx(i, j, k)])
              continue;
            if (triOverlapsFaceQuad(v[0], v[1], v[2], 0, g.origin.x + i * g.size.x,
                                    g.origin.y + j * g.size.y,
                                    g.origin.y + (j + 1) * g.size.y,
                                    g.origin.z + k * g.size.z,
                                    g.origin.z + (k + 1) * g.size.z, tol))
              g.bx[g.fidx(i, j, k)] = 1;
          }
    }
    // Y-normal faces.
    {
      const int j0 = std::clamp(static_cast<int>(std::floor((tb.mn.y - g.origin.y) / g.size.y)), 0, gridRes);
      const int j1 = std::clamp(static_cast<int>(std::ceil((tb.mx.y - g.origin.y) / g.size.y)), 0, gridRes);
      const int i0 = std::clamp(static_cast<int>(std::floor((tb.mn.x - g.origin.x) / g.size.x)), 0, gridRes - 1);
      const int i1 = std::clamp(static_cast<int>(std::floor((tb.mx.x - g.origin.x) / g.size.x)), 0, gridRes - 1);
      const int k0 = std::clamp(static_cast<int>(std::floor((tb.mn.z - g.origin.z) / g.size.z)), 0, gridRes - 1);
      const int k1 = std::clamp(static_cast<int>(std::floor((tb.mx.z - g.origin.z) / g.size.z)), 0, gridRes - 1);
      for (int i = i0; i <= i1; ++i)
        for (int j = j0; j <= j1; ++j)
          for (int k = k0; k <= k1; ++k) {
            if (g.by[g.fidx(i, j, k)])
              continue;
            if (triOverlapsFaceQuad(v[0], v[1], v[2], 1, g.origin.y + j * g.size.y,
                                    g.origin.z + k * g.size.z,
                                    g.origin.z + (k + 1) * g.size.z,
                                    g.origin.x + i * g.size.x,
                                    g.origin.x + (i + 1) * g.size.x, tol))
              g.by[g.fidx(i, j, k)] = 1;
          }
    }
    // Z-normal faces.
    {
      const int k0 = std::clamp(static_cast<int>(std::floor((tb.mn.z - g.origin.z) / g.size.z)), 0, gridRes);
      const int k1 = std::clamp(static_cast<int>(std::ceil((tb.mx.z - g.origin.z) / g.size.z)), 0, gridRes);
      const int i0 = std::clamp(static_cast<int>(std::floor((tb.mn.x - g.origin.x) / g.size.x)), 0, gridRes - 1);
      const int i1 = std::clamp(static_cast<int>(std::floor((tb.mx.x - g.origin.x) / g.size.x)), 0, gridRes - 1);
      const int j0 = std::clamp(static_cast<int>(std::floor((tb.mn.y - g.origin.y) / g.size.y)), 0, gridRes - 1);
      const int j1 = std::clamp(static_cast<int>(std::floor((tb.mx.y - g.origin.y) / g.size.y)), 0, gridRes - 1);
      for (int i = i0; i <= i1; ++i)
        for (int j = j0; j <= j1; ++j)
          for (int k = k0; k <= k1; ++k) {
            if (g.bz[g.fidx(i, j, k)])
              continue;
            if (triOverlapsFaceQuad(v[0], v[1], v[2], 2, g.origin.z + k * g.size.z,
                                    g.origin.x + i * g.size.x,
                                    g.origin.x + (i + 1) * g.size.x,
                                    g.origin.y + j * g.size.y,
                                    g.origin.y + (j + 1) * g.size.y, tol))
              g.bz[g.fidx(i, j, k)] = 1;
          }
    }
  }
  result.blockedFaces =
      static_cast<int>(std::count(g.bx.begin(), g.bx.end(), 1) +
                       std::count(g.by.begin(), g.by.end(), 1) +
                       std::count(g.bz.begin(), g.bz.end(), 1));

  // --- regionSplit analogue: flood fill cells through open faces ---
  const int di[6] = {1, -1, 0, 0, 0, 0};
  const int dj[6] = {0, 0, 1, -1, 0, 0};
  const int dk[6] = {0, 0, 0, 0, 1, -1};
  int nRegions = 0;
  std::vector<int> stack;
  for (int id = 0; id < n3; ++id) {
    if (g.region[id] != -1)
      continue;
    stack.clear();
    stack.push_back(id);
    g.region[id] = nRegions;
    while (!stack.empty()) {
      const int cur = stack.back();
      stack.pop_back();
      const int ci = cur / (g.n * g.n), cj = (cur / g.n) % g.n, ck = cur % g.n;
      for (int a = 0; a < 6; ++a) {
        const int ii = ci + di[a], jj = cj + dj[a], kk = ck + dk[a];
        if (!g.inside(ii, jj, kk))
          continue;
        if (!g.faceOpen(ci, cj, ck, a))
          continue; // surface passes between these cells
        const int nid = g.idx(ii, jj, kk);
        if (g.region[nid] != -1)
          continue;
        g.region[nid] = nRegions;
        stack.push_back(nid);
      }
    }
    ++nRegions;
  }
  result.regions = nRegions;

  // Regions touching the grid border are connected to the outside world
  // ("infinity"): a far-away outside point can reach them.
  std::vector<char> regionTouchesBorder(static_cast<size_t>(nRegions), 0);
  for (int i = 0; i < g.n; ++i)
    for (int j = 0; j < g.n; ++j)
      for (int k = 0; k < g.n; ++k) {
        if (i != 0 && i != g.n - 1 && j != 0 && j != g.n - 1 && k != 0 &&
            k != g.n - 1)
          continue;
        const int id = g.idx(i, j, k);
        if (g.region[id] >= 0)
          regionTouchesBorder[static_cast<size_t>(g.region[id])] = 1;
      }

  // --- findCell analogue for probe points ---
  // Cells are never blocked now, so any in-grid point locates directly.
  enum class Locate { Found, OutsideGrid };
  auto locateOpen = [&](const std::array<double, 3>& p, int& cell) {
    cell = -1;
    if (!g.pointInGrid(p))
      return Locate::OutsideGrid;
    int i, j, k;
    g.locate(p, i, j, k);
    cell = g.idx(i, j, k);
    return Locate::Found;
  };
  auto warnPoint = [&](const QString& what, const std::array<double, 3>& p,
                       const char* reason) {
    result.warnings << QStringLiteral("%1 point (%2, %3, %4) %5.")
                           .arg(what)
                           .arg(p[0])
                           .arg(p[1])
                           .arg(p[2])
                           .arg(QString::fromUtf8(reason));
  };

  std::vector<int> insideCells, insideRegions;
  for (const auto& p : inside) {
    int cell = -1;
    if (locateOpen(p, cell) == Locate::Found) {
      insideCells.push_back(cell);
      insideRegions.push_back(g.region[cell]);
    } else {
      warnPoint(QStringLiteral("Inside"), p, "lies outside the background grid");
    }
  }
  if (!inside.empty() && insideCells.empty()) {
    result.warnings << QStringLiteral("No inside point could be located in "
                                      "the background grid.");
    return result;
  }

  auto cellCenter = [&](int id) {
    const Vec3 c = g.center(id / (g.n * g.n), (id / g.n) % g.n, id % g.n);
    return std::array<double, 3>{c.x, c.y, c.z};
  };
  auto isBorderCell = [&](int id) {
    const int ci = id / (g.n * g.n), cj = (id / g.n) % g.n, ck = id % g.n;
    return ci == 0 || ci == g.n - 1 || cj == 0 || cj == g.n - 1 || ck == 0 ||
           ck == g.n - 1;
  };

  // --- Leak test: outside point in an inside region => leak ---
  // One search pair per leaking (inside, outside) combination (previously
  // only the first inside match per outside point was traced).
  struct SearchPair {
    int startCell = -1;
    int targetCell = -1; // <0 for far-outside (any region border cell)
    int wantRegion = -1;
    std::array<double, 3> fromPt{};
    std::array<double, 3> toPt{};
    bool farOutside = false;
  };
  std::vector<SearchPair> pairs;
  for (const auto& p : outside) {
    int cell = -1;
    const Locate loc = locateOpen(p, cell);
    if (loc == Locate::Found) {
      const int reg = g.region[cell];
      for (size_t k = 0; k < insideRegions.size(); ++k) {
        if (insideRegions[k] != reg)
          continue;
        pairs.push_back({insideCells[k], cell, reg, inside[k], p, false});
      }
    } else {
      // Outside the background grid: leaks iff an inside region touches
      // the grid border (i.e. reaches the outside world).
      for (size_t k = 0; k < insideRegions.size(); ++k) {
        if (!regionTouchesBorder[static_cast<size_t>(insideRegions[k])])
          continue;
        pairs.push_back(
            {insideCells[k], -1, insideRegions[k], inside[k], p, true});
      }
    }
  }

  // BFS shortest path per pair (shortestPathSet analogue), OpenMP-parallel
  // over pairs with thread-local parents: indexed writes keep the output
  // deterministic regardless of thread count.
  std::vector<LeakPath> bfsPaths(pairs.size());
  std::vector<char> bfsFound(pairs.size(), 0);
#pragma omp parallel for schedule(dynamic)
  for (std::size_t pi = 0; pi < pairs.size(); ++pi) {
    const SearchPair& pr = pairs[pi];
    std::vector<int> parent(n3, -1);
    std::queue<int> q;
    q.push(pr.startCell);
    parent[pr.startCell] = pr.startCell;
    int reached = -1;
    if (!pr.farOutside && pr.startCell == pr.targetCell)
      reached = pr.startCell;
    while (!q.empty() && reached < 0) {
      const int cur = q.front();
      q.pop();
      const int ci = cur / (g.n * g.n), cj = (cur / g.n) % g.n, ck = cur % g.n;
      for (int a = 0; a < 6; ++a) {
        const int ii = ci + di[a], jj = cj + dj[a], kk = ck + dk[a];
        if (!g.inside(ii, jj, kk))
          continue;
        if (!g.faceOpen(ci, cj, ck, a))
          continue; // surface passes between these cells
        const int nid = g.idx(ii, jj, kk);
        if (parent[nid] >= 0)
          continue;
        parent[nid] = cur;
        const bool target =
            pr.farOutside
                ? (isBorderCell(nid) && g.region[nid] == pr.wantRegion)
                : (nid == pr.targetCell);
        if (target) {
          reached = nid;
          break;
        }
        q.push(nid);
      }
    }
    if (reached < 0)
      continue; // same region but unreachable: cannot happen, stay safe
    LeakPath path;
    path.from = pr.fromPt;
    path.to = pr.toPt;
    QVector<std::array<double, 3>> rev;
    int cur = reached;
    while (cur != pr.startCell) {
      rev.prepend(cellCenter(cur));
      cur = parent[cur];
    }
    rev.prepend(cellCenter(pr.startCell));
    if (pr.farOutside)
      rev.append(pr.toPt); // finish at the actual outside point
    path.polyline = rev;
    bfsPaths[pi] = std::move(path);
    bfsFound[pi] = 1;
  }
  for (std::size_t pi = 0; pi < pairs.size(); ++pi) {
    if (bfsFound[pi])
      result.paths.append(std::move(bfsPaths[pi]));
  }

  // Seeded random walks from every inside cell (holeToFace-style route
  // diversity: the single shortest path per pair only ever crosses one
  // hole, while walks fan out through every gap). Each walk owns its RNG
  // stream, so results are deterministic under any thread count. Every
  // kept route spans its inside point to an outside point: the polyline
  // starts exactly at the inside probe, and the escape cell connects to
  // the nearest outside probe with a straight segment (outside points
  // usually sit beyond the grid, where no voxel path can reach them).
  const int walksPerInside = std::max(0, params.walksPerInside);
  const int maxSteps =
      params.maxWalkSteps > 0 ? params.maxWalkSteps : 256 * g.n;
  const std::size_t nWalks =
      insideCells.size() * static_cast<std::size_t>(walksPerInside);
  std::vector<LeakPath> walkOut(nWalks);
  std::vector<char> walkFound(nWalks, 0);
#pragma omp parallel for schedule(dynamic)
  for (std::size_t w = 0; w < nWalks; ++w) {
    const std::size_t ik =
        walksPerInside > 0 ? w / static_cast<std::size_t>(walksPerInside) : 0;
    std::mt19937 rng(params.walkSeed + static_cast<unsigned>(w * 7919u + 13u));
    int cur = insideCells[ik];
    std::vector<int> trail;
    trail.reserve(512);
    trail.push_back(cur);
    int nidBuf[6];
    for (int s = 0; s < maxSteps; ++s) {
      if (isBorderCell(cur))
        break; // reached the outside world
      const int ci = cur / (g.n * g.n), cj = (cur / g.n) % g.n, ck = cur % g.n;
      int nb = 0;
      for (int a = 0; a < 6; ++a) {
        const int ii = ci + di[a], jj = cj + dj[a], kk = ck + dk[a];
        if (!g.inside(ii, jj, kk))
          continue;
        if (!g.faceOpen(ci, cj, ck, a))
          continue; // surface passes between these cells
        nidBuf[nb++] = g.idx(ii, jj, kk);
      }
      if (nb == 0)
        break; // dead end inside a sealed pocket
      cur = nidBuf[rng() % static_cast<unsigned>(nb)];
      trail.push_back(cur);
    }
    if (!isBorderCell(cur))
      continue; // never escaped: no leak evidence from this walk
    LeakPath path;
    path.from = inside[ik];
    // Straight connector from the escape cell to the nearest outside
    // point; without outside points the route just ends at the border.
    const std::array<double, 3> exit = cellCenter(cur);
    std::array<double, 3> outPt = exit;
    if (!outside.empty()) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto& o : outside) {
        const double d = (o[0] - exit[0]) * (o[0] - exit[0]) +
                         (o[1] - exit[1]) * (o[1] - exit[1]) +
                         (o[2] - exit[2]) * (o[2] - exit[2]);
        if (d < best) {
          best = d;
          outPt = o;
        }
      }
    }
    path.to = outPt;
    // Stride long trails down so polylines stay drawable.
    const std::size_t stride = trail.size() / 512 + 1;
    QVector<std::array<double, 3>> line;
    line.append(inside[ik]);
    for (std::size_t t = 0; t < trail.size(); t += stride)
      line.append(cellCenter(trail[t]));
    if ((trail.size() - 1) % stride != 0)
      line.append(cellCenter(trail.back()));
    line.append(outPt);
    path.polyline = line;
    walkOut[w] = std::move(path);
    walkFound[w] = 1;
  }
  for (std::size_t w = 0; w < nWalks; ++w) {
    if (walkFound[w])
      result.walkPaths.append(std::move(walkOut[w]));
  }

  // --- Bridge analysis: which hole loops lie on some inside->outside route.
  // Multi-source BFS distances from the inside cells and from the outside
  // world (grid border + outside probe cells). A loop is implicated when an
  // open voxel face near its rim joins an inside-reachable cell to an
  // outside-reachable one. Complete by construction: any bridgeable hole
  // has such a face, with no route sampling involved.
  auto bfsDistances = [&](const std::vector<int>& sources) {
    std::vector<int> dist(n3, -1);
    std::queue<int> q;
    for (int s : sources) {
      if (s >= 0 && s < n3 && dist[s] < 0) {
        dist[s] = 0;
        q.push(s);
      }
    }
    while (!q.empty()) {
      const int cur = q.front();
      q.pop();
      const int ci = cur / (g.n * g.n), cj = (cur / g.n) % g.n, ck = cur % g.n;
      for (int a = 0; a < 6; ++a) {
        const int ii = ci + di[a], jj = cj + dj[a], kk = ck + dk[a];
        if (!g.inside(ii, jj, kk) || !g.faceOpen(ci, cj, ck, a))
          continue;
        const int nid = g.idx(ii, jj, kk);
        if (dist[nid] >= 0)
          continue;
        dist[nid] = dist[cur] + 1;
        q.push(nid);
      }
    }
    return dist;
  };
  const std::vector<int> insideDist = bfsDistances(insideCells);
  std::vector<int> outsideSources;
  for (int id = 0; id < n3; ++id) {
    const int ci = id / (g.n * g.n), cj = (id / g.n) % g.n, ck = id % g.n;
    if (ci == 0 || ci == g.n - 1 || cj == 0 || cj == g.n - 1 || ck == 0 ||
        ck == g.n - 1)
      outsideSources.push_back(id);
  }
  for (const auto& p : outside) {
    int cell = -1;
    if (locateOpen(p, cell) == Locate::Found)
      outsideSources.push_back(cell);
  }
  const std::vector<int> outsideDist = bfsDistances(outsideSources);

  result.loops = findHoleLoops(mesh);
  if (result.loops.isEmpty()) {
    // Closed mesh: with no border there is no hole to leak through, so any
    // voxel path from an inside cell to the grid border threaded a sub-cell
    // gap (surface running nearly parallel to grid faces between blocked
    // sheets) rather than a real hole. Report separated, not leaking.
    result.paths.clear();
    result.walkPaths.clear();
    result.bridgingLoops.clear();
    result.hasLeak = false;
    return result;
  }
  // For each rim edge midpoint, inspect the faces of its cell and the six
  // neighbours: an open one bridging the two distance fields implicates
  // the loop.
  for (int li = 0; li < result.loops.size(); ++li) {
    const auto& pts = result.loops[li].points;
    if (pts.size() < 3)
      continue;
    bool implicated = false;
    for (int s = 0; s < pts.size() && !implicated; ++s) {
      const auto& a = pts[s];
      const auto& b = pts[(s + 1) % pts.size()];
      const std::array<double, 3> m = {(a[0] + b[0]) / 2, (a[1] + b[1]) / 2,
                                       (a[2] + b[2]) / 2};
      if (!g.pointInGrid(m))
        continue;
      int mi, mj, mk;
      g.locate(m, mi, mj, mk);
      for (int oi = -1; oi <= 1 && !implicated; ++oi)
        for (int oj = -1; oj <= 1 && !implicated; ++oj)
          for (int ok = -1; ok <= 1 && !implicated; ++ok) {
            const int ci = mi + oi, cj = mj + oj, ck = mk + ok;
            if (!g.inside(ci, cj, ck))
              continue;
            for (int f = 0; f < 6 && !implicated; ++f) {
              const int ii = ci + di[f], jj = cj + dj[f], kk = ck + dk[f];
              if (!g.inside(ii, jj, kk))
                continue;
              if (!g.faceOpen(ci, cj, ck, f))
                continue;
              const int nid = g.idx(ii, jj, kk);
              const int u = g.idx(ci, cj, ck);
              if ((insideDist[u] >= 0 && outsideDist[nid] >= 0) ||
                  (insideDist[nid] >= 0 && outsideDist[u] >= 0))
                implicated = true;
            }
          }
    }
    if (implicated)
      result.bridgingLoops.insert(li);
  }

  result.hasLeak = !result.paths.isEmpty() || !result.walkPaths.isEmpty();
  return result;
}

// One seed border halfedge per hole; walk the full loop from each seed.
static std::vector<std::vector<Mesh::Halfedge_index>>
boundaryCycles(const Mesh& mesh) {
  std::vector<std::vector<Mesh::Halfedge_index>> cycles;
  std::vector<Mesh::Halfedge_index> seeds;
  CGAL::extract_boundary_cycles(mesh, std::back_inserter(seeds));
  for (auto seed : seeds) {
    std::vector<Mesh::Halfedge_index> cycle;
    for (auto h : halfedges_around_face(seed, mesh))
      cycle.push_back(h);
    if (!cycle.empty())
      cycles.push_back(std::move(cycle));
  }
  return cycles;
}

QVector<HoleLoop> findHoleLoops(const Mesh& mesh) {
  QVector<HoleLoop> loops;
  try {
    for (const auto& cycle : boundaryCycles(mesh)) {
      HoleLoop loop;
      Vec3 prev{0, 0, 0};
      bool first = true;
      for (auto h : cycle) {
        const Vec3 p = toVec(mesh.point(mesh.source(h)));
        loop.points.append({p.x, p.y, p.z});
        if (!first) {
          const double dx = p.x - prev.x, dy = p.y - prev.y, dz = p.z - prev.z;
          loop.length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        prev = p;
        first = false;
      }
      if (loop.points.size() >= 3) {
        const auto& f = loop.points.first();
        const double dx = f[0] - prev.x, dy = f[1] - prev.y, dz = f[2] - prev.z;
        loop.length += std::sqrt(dx * dx + dy * dy + dz * dz);
        loops.append(std::move(loop));
      }
    }
  } catch (...) {
  }
  return loops;
}



PatchResult patchHoles(Mesh& mesh, int maxLoopEdges, const QSet<int>& onlyLoops) {
  PatchResult result;
  std::vector<std::vector<Mesh::Halfedge_index>> cycles;
  try {
    cycles = boundaryCycles(mesh);
  } catch (...) {
    result.warnings << QStringLiteral("Failed to extract boundary loops.");
    return result;
  }
  result.loopsFound = static_cast<int>(cycles.size());
  const bool selective = !onlyLoops.isEmpty();
  // keptIdx tracks the findHoleLoops() ordering (degenerate <3-edge cycles
  // are skipped by both, so attribution indices stay aligned).
  int keptIdx = -1;
  for (const auto& cycle : cycles) {
    if (cycle.size() < 3)
      continue;
    ++keptIdx;
    if (selective && !onlyLoops.contains(keptIdx)) {
      ++result.loopsSkippedNotLeaking;
      continue;
    }
    if (static_cast<int>(cycle.size()) > maxLoopEdges) {
      ++result.loopsSkippedTooLarge;
      continue;
    }
    // Minimal-area triangulation hugging the hole (CGAL hole filling).
    // Unlike a centroid fan this never tents across non-planar loops, so
    // the patched surface stays a clean membrane any voxel barrier seals.
    const std::size_t facesBefore = mesh.num_faces();
    bool filled = false;
    try {
      PMP::triangulate_hole(mesh, cycle.front());
      filled = true;
    } catch (...) {
      filled = false;
    }
    if (!filled) {
      // Fallback: centroid fan. Adding faces in cycle order reuses the
      // existing border halfedges, which is what closes the mesh.
      std::vector<Mesh::Vertex_index> ring;
      ring.reserve(cycle.size());
      Kernel::FT sx(0), sy(0), sz(0);
      int n = 0;
      for (auto h : cycle) {
        const auto v = mesh.source(h);
        ring.push_back(v);
        const Point3& p = mesh.point(v);
        sx += p.x();
        sy += p.y();
        sz += p.z();
        ++n;
      }
      const Point3 centroid(sx / n, sy / n, sz / n);
      const auto vc = mesh.add_vertex(centroid);
      filled = true;
      for (size_t i = 0; i < ring.size(); ++i) {
        const auto f = mesh.add_face(ring[i], ring[(i + 1) % ring.size()], vc);
        if (f == Mesh::null_face()) {
          filled = false;
          break;
        }
      }
      if (!filled) {
        result.warnings << QStringLiteral("Fan fill failed for a %1-edge loop "
                                          "(non-manifold); left open.")
                               .arg(cycle.size());
        mesh.remove_vertex(vc);
        continue;
      }
    }
    ++result.loopsFilled;
    result.facesAdded += static_cast<int>(mesh.num_faces() - facesBefore);
  }
  mesh.collect_garbage();
  // Make the patched mesh consistently oriented where it is closed.
  try {
    if (isWatertight(mesh))
      PMP::orient(mesh);
  } catch (...) {
  }
  return result;
}

} // namespace meshbench
