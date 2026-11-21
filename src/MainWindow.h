#pragma once

// MainWindow: menu + left pipeline-history sidebar (ParaView filter-tree
// style) + central VTK viewport + right inspector (stage stats, leak
// probes, log).

#include "MeshDocument.h"
#include "Shrinkwrap.h"
#include "VtkView.h"

#include <QMainWindow>
#include <QMap>
#include <QSet>

class QDoubleSpinBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QTableWidget;
class QTreeWidget;
class QTreeWidgetItem;

namespace meshbench {

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent = nullptr);

private slots:
  void openMesh();
  void importMeshes();
  void runCombineSelected();
  void saveActiveMesh();
  void clearAll();
  void runBoolean();
  void runCenter();
  void runTransform();
  void runAlphaWrap();
  void runClip();
  void runDetectLeaks();
  void runPatchHoles();
  void runGmshRemesh();
  void runCreatePrimitive();
  void runIntroduceHoles();
  void runShrinkwrap();
  void rerunShrinkwrap();
  void togglePickMode(bool on);
  void onFacePicked(int faceIdx);
  void applyWeightToSelection();
  void clearFaceSelection();
  void resetFaceWeights();
  void setDefaultProbes();
  void addProbeRow(bool inside);
  void removeSelectedProbes();
  void toggleSelectMode(bool on);
  void onFacesSelected(const QSet<int>& faces);
  void runRemeshSelection();
  void runDetectLeaksSelection();
  void runExtractSelection();
  void clearSelection();

  void onTreeItemChanged(QTreeWidgetItem* item, int column);
  void onTreeItemActivated(QTreeWidgetItem* item, int column);
  void revertToSelectedStage();
  void refreshAll();

private:
  void log(const QString& msg);
  void rebuildTree();
  void updateStagePanel();
  void refreshView();
  QVector<ProbePoint> probesFromTable() const;
  void probesToTable(const QVector<ProbePoint>& probes);
  bool requireMesh() const;

  // Explicit right-click stage selection for Combine (see StageSelection.h).
  QSet<int> selectedStageIndices() const;
  void setStageSelection(const QSet<int>& stages);
  void clearStageSelection();
  void pruneStageSelection();

  MeshDocument m_doc;
  VtkView* m_view = nullptr;
  QTreeWidget* m_tree = nullptr;
  QSet<int> m_visible; // pipeline indices currently drawn
  QSet<int> m_stageSelection; // persistent Combine selection across right-clicks
  bool m_updatingTree = false;

  // Viewport selection (S shortcut): face indices of m_selectionStage,
  // shared id space with extractSubmesh; filters run on the submesh.
  bool m_selectMode = false;
  int m_selectionStage = -1;
  QSet<int> m_selection;
  QPushButton* m_selectBtn = nullptr;
  QLabel* m_selectLabel = nullptr;
  void updateSelectPanel();
  void validateSelection();
  void probeMeshAndAppend(const Mesh& mesh, const QString& stageLabel);
  bool remeshMesh(const Mesh& in, double size, Mesh& out, QString& error);

  QLabel* m_stageInfo = nullptr;
  QDoubleSpinBox* m_opacitySpin = nullptr;
  QTableWidget* m_probeTable = nullptr;
  QPlainTextEdit* m_log = nullptr;
  QLabel* m_leakSummary = nullptr;

  // Shrinkwrap state: source stage snapped from, wrap stage holding the
  // result, per-target-face attraction weights, and the clicked selection.
  ShrinkParams m_shrinkParams;
  int m_wrapSource = -1;
  int m_wrapStage = -1;
  QMap<int, double> m_faceWeights; // target face idx -> multiplier
  QSet<int> m_faceSelection;
  QPushButton* m_pickBtn = nullptr;
  QLabel* m_selLabel = nullptr;
  QSlider* m_weightSlider = nullptr;
  QDoubleSpinBox* m_weightSpin = nullptr;
  void updateShrinkPanel();
  void validateShrinkState();
};

} // namespace meshbench
