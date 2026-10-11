#pragma once

// Shared CGAL mesh types for MeshBench.
//
// Kernel choice: Exact_predicates_exact_constructions_kernel. Boolean
// operations (corefine), clipping and alpha-wrapping all need exact
// predicates; exact constructions make intersection results robust.
// Rendering converts points to double via CGAL::to_double.

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace meshbench {

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point3 = Kernel::Point_3;
using Vector3 = Kernel::Vector_3;
using Plane3 = Kernel::Plane_3;
using Mesh = CGAL::Surface_mesh<Point3>;

inline double toDouble(const Kernel::FT& v) { return CGAL::to_double(v); }

inline Point3 makePoint(double x, double y, double z) {
  return Point3(Kernel::FT(x), Kernel::FT(y), Kernel::FT(z));
}

} // namespace meshbench
