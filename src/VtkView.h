#pragma once

// VtkView: QVTKOpenGLNativeWidget wrapper that renders one actor per
// pipeline stage plus leak overlays (hole-loop edges, inside/outside
// probe points, leak paths, background voxel grid). Leak paths and the
// voxel grid draw only when their display flags are set (both default
// off); probe spheres still show on the active LeakProbe stage.
//
// Pick mode: setPickMode(true, stage) swaps in a click-to-select
// interactor style restricted to that stage's actor; each click emits
// facePicked() with the target-mesh face index under the cursor so the
// sidebar can paint shrinkwrap weights. Shrinkwrap stages always render
// translucent so the baseline geometry shows through.

#include "MeshDocument.h"

#include <QMap>
#include <QSet>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkSmartPointer.h>

#include <array>
#include <cstddef>
#include <functional>
#include <vector>

class vtkActor;
class vtkInteractorStyle;
class vtkRenderer;

namespace meshbench {

class VtkPickStyle;

class VtkView : public QVTKOpenGLNativeWidget {
  Q_OBJECT
public:
  explicit VtkView(QWidget* parent = nullptr);
  ~VtkView() override;

  // Rebuild all stage actors from the document. visibleStages selects which
  // node indices are drawn.
  void showDocument(const MeshDocument& doc, const QSet<int>& visibleStages,
                    int activeStage);
  void clear();
  void resetCamera();

  // Enable/disable click-to-select on the given stage's actor.
  void setPickMode(bool on, int stage = -1);

  // Overlay toggles for the active LeakProbe stage (both default off).
  void setShowLeakPaths(bool on) { m_showLeakPaths = on; }
  void setShowVoxelGrid(bool on) { m_showVoxelGrid = on; }

signals:
  // Target-mesh face index picked in pick mode (face idx, none if invalid).
  void facePicked(int faceIdx);

private:
  vtkSmartPointer<vtkRenderer> m_renderer;
  QMap<int, vtkSmartPointer<vtkActor>> m_stageActors;
  // Per-stage VTK cell id -> target-mesh face idx (display triangulates).
  QMap<int, std::vector<std::size_t>> m_stageCellFaces;
  std::vector<vtkSmartPointer<vtkActor>> m_overlayActors;

  vtkSmartPointer<vtkActor> meshActor(const Mesh& mesh, double r, double g,
                                      double b, double opacity,
                                      std::vector<std::size_t>& cellFaces);
  void addHoleLoopActors(const Mesh& mesh, double r, double g, double b);
  void addProbeActors(const QVector<ProbePoint>& probes);
  void addLeakPathActors(const LeakReport& leaks);
  void addVoxelGridActors(const LeakReport& leaks);
  bool m_showLeakPaths = false;
  bool m_showVoxelGrid = false;
  double m_probeMarkerRad = 0.0; // scene-relative marker size from addProbeActors

  vtkSmartPointer<vtkActor> m_pickActor;
  std::vector<std::size_t> m_pickFaces;
  bool m_pickMode = false;
  int m_pickStage = -1;
  vtkSmartPointer<VtkPickStyle> m_pickStyle;
  vtkSmartPointer<vtkInteractorStyle> m_savedStyle;
};

} // namespace meshbench
