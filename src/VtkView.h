#pragma once

// VtkView: QVTKOpenGLNativeWidget wrapper that renders one actor per
// pipeline stage plus leak overlays (hole-loop edges, inside/outside
// probe points, leak paths, background voxel grid). Leak paths and the
// voxel grid draw only when their display flags are set (both default
// off); probe spheres still show on the active LeakProbe stage. A small
// XYZ orientation marker sits in the lower-left corner.
//
// Pick mode: setPickMode(true, stage) swaps in a click-to-select
// interactor style restricted to that stage's actor; each click emits
// facePicked() with the target-mesh face index under the cursor so the
// sidebar can paint shrinkwrap weights. Shrinkwrap stages always render
// translucent so the baseline geometry shows through.

#include "MeshDocument.h"
#include "ViewOrientations.h"

#include <QMap>
#include <QPoint>
#include <QSet>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkSmartPointer.h>

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

class QMouseEvent;
class QRubberBand;
class QShowEvent;

class vtkActor;
class vtkAxesActor;
class vtkInteractorStyle;
class vtkOrientationMarkerWidget;
class vtkPolyData;
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

  // Export the current scene as PNG. width/height <= 0 use the widget's
  // current size. sceneImagePng returns the bytes for MCP tool results;
  // saveSceneImage writes them to disk (File > Export scene image).
  bool sceneImagePng(int width, int height, QByteArray& png, QString& error);
  bool saveSceneImage(const QString& path, int width, int height,
                      QString& error);

  // Snap the active camera to a standard +/-X, +/-Y or +/-Z view. Keeps
  // the current focal point and camera distance, like ParaView's axis
  // toolbar, and uses the scene's up-axis conventions.
  void setStandardView(StandardView view);

  // Enable/disable click-to-select on the given stage's actor.
  void setPickMode(bool on, int stage = -1);

  // Selection mode (toggled with the 'S' shortcut): single clicks toggle
  // individual triangles, click-and-drag draws a rubber-band rectangle
  // selecting every triangle inside (added to the set). Clicking directly on
  // a drawn feature edge toggles that edge instead of a triangle, and a drag
  // rectangle also adds feature edges inside it. Selection faces are
  // target-mesh face indices (same id space as facePicked); feature edges are
  // FeatureEdges::edges indices. Faces render as a yellow highlight, selected
  // edges as a brighter yellow line overlay.
  void setSelectMode(bool on, int stage = -1);
  bool selectMode() const { return m_selectMode; }
  void setSelection(int stage, const QSet<int>& faces);
  void setEdgeSelection(int stage, const QSet<int>& edges);

  // Overlay toggles for the active LeakProbe stage (both default off).
  void setShowLeakPaths(bool on) { m_showLeakPaths = on; }
  void setShowVoxelGrid(bool on) { m_showVoxelGrid = on; }

signals:
  // Target-mesh face index picked in pick mode (face idx, none if invalid).
  void facePicked(int faceIdx);
  // Target-mesh face indices inside a rubber-band rectangle (additive).
  void facesSelected(const QSet<int>& faces);
  // FeatureEdges::edges index clicked in selection mode.
  void featureEdgePicked(int edgeIdx);
  // Feature-edge indices inside a rubber-band rectangle (additive).
  void featureEdgesSelected(const QSet<int>& edges);

private:
  vtkSmartPointer<vtkRenderer> m_renderer;
  // ParaView-style XYZ orientation marker in the lower-left corner.
  vtkSmartPointer<vtkAxesActor> m_axesActor;
  vtkSmartPointer<vtkOrientationMarkerWidget> m_axesWidget;
  bool m_axesWidgetReady = false;
  void ensureAxesWidget();

  QMap<int, vtkSmartPointer<vtkActor>> m_stageActors;
  // Per-stage VTK cell id -> target-mesh face idx (display triangulates).
  QMap<int, std::vector<std::size_t>> m_stageCellFaces;
  // Feature-edge stages draw one line cell per FeatureEdges::edges entry, so
  // the VTK cell id is also the feature-edge index.
  QMap<int, vtkSmartPointer<vtkActor>> m_featureEdgeActors;
  std::vector<vtkSmartPointer<vtkActor>> m_overlayActors;

  vtkSmartPointer<vtkActor> meshActor(const Mesh& mesh, double r, double g,
                                      double b, double opacity,
                                      std::vector<std::size_t>& cellFaces);
  vtkSmartPointer<vtkActor> featureEdgeActor(const FeatureEdges& edges);
protected:
  void showEvent(QShowEvent* e) override;
  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;
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

  // Selection-mode state (see setSelectMode).
  bool m_selectMode = false;
  int m_selectStage = -1;
  // Per-stage render geometry (kept so the highlight can re-extract its
  // cells after a rebuild without touching the mesh).
  // (Cell order matches m_stageCellFaces, so faces map 1:1.)
  QMap<int, vtkSmartPointer<vtkPolyData>> m_stagePoly;
  QMap<int, vtkSmartPointer<vtkPolyData>> m_stageFeaturePoly;
  vtkSmartPointer<vtkActor> m_selectActor;
  vtkSmartPointer<vtkActor> m_edgeSelectActor;
  QSet<int> m_selFaces;
  QSet<int> m_selEdges;
  int m_selStage = -1;
  bool m_selectPressed = false;
  QPoint m_pressPos;
  std::unique_ptr<QRubberBand> m_band;

  void updateSelectionHighlight();
  void pickSingleCell(const QPoint& pos);
  bool pickSingleFeatureEdge(const QPoint& pos);
  void pickRect(const QPoint& a, const QPoint& b);
};

} // namespace meshbench
