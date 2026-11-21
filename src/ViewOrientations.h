#pragma once

// Standard camera orientations used by the View menu (ParaView-style
// +/-X, +/-Y, +/-Z). Kept free of VTK/Qt so the mapping is unit-testable.

#include <array>

namespace meshbench {

enum class StandardView { PosX, NegX, PosY, NegY, PosZ, NegZ };

struct ViewBasis {
  // Unit vector from the focal point to the camera position.
  std::array<double, 3> cameraOffset;
  // Camera up vector in world coordinates.
  std::array<double, 3> up;
};

constexpr ViewBasis standardViewBasis(StandardView view) {
  switch (view) {
  case StandardView::PosX:
    return ViewBasis{{1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}};
  case StandardView::NegX:
    return ViewBasis{{-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}};
  case StandardView::PosY:
    return ViewBasis{{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  case StandardView::NegY:
    return ViewBasis{{0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}};
  case StandardView::PosZ:
    return ViewBasis{{0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}};
  case StandardView::NegZ:
    return ViewBasis{{0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}};
  }
  return ViewBasis{{0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}};
}

} // namespace meshbench
