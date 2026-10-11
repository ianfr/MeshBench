#pragma once

// Interactive shrinkwrap.
//
// A high-resolution icosphere enclosing the target mesh is snapped onto it:
// each wrap vertex is repeatedly pulled toward its closest point on the
// target surface (CGAL AABB tree), followed by a Laplacian smoothing pass.
// Every target triangle carries a user-controlled 'weight' (default 1.0)
// that scales the attraction of its region: 0 ignores the region, >1 pulls
// harder. Weights are painted by clicking target triangles in the viewport
// (see VtkView pick mode) and edited in the sidebar.

#include "MeshTypes.h"

#include <QString>

#include <cstddef>
#include <unordered_map>

namespace meshbench {

struct ShrinkParams {
  int subdivisions = 4;  // icosphere refinement (4 -> 2562 verts)
  int iterations = 25;
  double step = 0.3;     // 0 < step <= 1, fraction of distance per iteration
  double smooth = 0.2;   // Laplacian blend 0..1
  // Even-spacing aggressiveness 0..1: extra tangential relaxation toward
  // the neighbour centroid (projected onto the local surface tangent
  // plane), so vertices spread evenly instead of bunching. Unlike
  // `smooth` it does not shrink the wrap onto itself.
  double uniformity = 0.0;
};

// Icosphere enclosing the target mesh bounding box.
Mesh makeEnclosingSphere(const Mesh& target, int subdivisions);

// One full snap run: attract + smooth for params.iterations. weights maps
// target face idx -> attraction multiplier; missing entries default to 1.
bool snapShrinkwrap(const Mesh& target,
                    const std::unordered_map<std::size_t, double>& weights,
                    Mesh& wrap, const ShrinkParams& params, QString& error);

} // namespace meshbench
