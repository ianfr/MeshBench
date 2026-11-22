#pragma once

// MainWindow: menu + left pipeline-history sidebar (ParaView filter-tree
// style) + central VTK viewport + right inspector (stage stats, leak
// probes, log).
//
// Besides the interactive slots, every File/Operations/View action has a
// scriptable `apply...` path used by executeCommand() and the MCP server.
// Keeping both callers on the same code path means the GUI and the LLM API
// always perform identical operations.

#include "Commands.h"
#include "MeshDocument.h"
#include "MeshOps.h"
#include "Shrinkwrap.h"
#include "VtkView.h"

#include <QMainWindow>
#include <QMap>
#include <QSet>

#include <atomic>
#include <memory>

class QDoubleSpinBox;
class QAction;
class QLabel;
class QMenu;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QTableWidget;
class QTreeWidget;
class QTreeWidgetItem;

namespace meshbench {

class McpHttpServer;
class McpProtocol;
class GmshCadSession;
struct McpHttpRequest;
struct McpHttpResponse;

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

  // Scriptable menu command used by the MCP server. Always safe to call on
  // the GUI thread; returns a structured result (or an error) for the tool
  // caller instead of opening dialogs.
  CommandReply executeCommand(const QString& name,
                              const QJsonObject& arguments);
  QJsonObject objectTreeState() const;

private slots:
  void openMesh();
  void importMeshes();
  void importFeatureEdges();
  void showOperationsSearch();
  void runCombineSelected();
  void saveActiveMesh();
  void saveActiveFeatureEdges();
  void exportSceneImage();
  void clearAll();
  void runBoolean();
  void runCenter();
  void runTransform();
  void runAlphaWrap();
  void runIsotropicRemesh();
  void runExtractFeatureEdges();
  void runSnapToFeatureEdges();
  void runAutoClean();
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
  void onFeatureEdgePicked(int edgeIdx);
  void onFeatureEdgesSelected(const QSet<int>& edges);
  void runRemeshSelection();
  void runDetectLeaksSelection();
  void runExtractSelection();
  void clearSelection();
  void deleteSelectedStages();
  void deleteStages(const QSet<int>& stages);

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
  bool ensureActiveMesh(QString& error) const;

  // --- Scriptable menu actions (GUI slots and MCP commands share these) ---
  bool loadMeshFile(const QString& path, QString& error);
  bool importMeshFiles(const QStringList& paths, QString& error);
  bool importFeatureEdgesFile(const QString& path, QString& error);
  bool saveActiveMeshTo(const QString& path, QString& error);
  bool saveActiveFeatureEdgesTo(const QString& path, QString& error);
  bool saveSceneImageTo(const QString& path, int width, int height,
                        QString& error);

  bool applyBoolean(const QString& path, BooleanOp op, QString& error);
  bool applyCenter(QString& error);
  bool applyTransform(const TransformParams& params, QString& error);
  bool applyAlphaWrap(double alphaDenominator, double offsetDenominator,
                      QString& error);
  bool applyIsotropicRemesh(double targetEdgeLength, int iterations,
                            double featureAngleDeg, QString& error);
  bool applyExtractFeatureEdges(double featureAngleDeg, bool includeBoundary,
                                QString& error);
  bool applySnapToFeatureEdges(int sourceStage, double targetEdgeLength,
                               double minFacetAngleDeg, QString& error);
  bool applyAutoClean(const AutoCleanParams& params, AutoCleanResult& result,
                      QString& error);
  bool applyClip(const Point3& origin, const Vector3& normal, QString& error);
  bool applyCombine(const QList<int>& stages, bool unionMode, QString& error);
  bool probeMeshAndAppend(const Mesh& mesh, const QString& stageLabel,
                          int gridRes, QString& error);
  bool applyPatchHoles(int maxLoopEdges, bool patchAll, QString& error,
                       int* loopsFilled = nullptr);
  bool applyGmshRemesh(double targetElementSize, QString& error);
  bool applyCreatePrimitive(const PrimitiveParams& params, QString& error);
  bool applyIntroduceHoles(int triangles, unsigned seed, QString& error);
  bool applyShrinkwrap(const ShrinkParams& params, QString& error);
  bool fillDefaultProbes(QString& error);

  // --- MCP ---------------------------------------------------------------
  void startMcpServer();
  McpHttpResponse handleHttpRequest(const McpHttpRequest& request);

  // --- MCP-only low-level Gmsh CAD session --------------------------------
  // Keeps one pipeline stage and refreshes it in place after every Gmsh
  // operation, so an LLM CAD workflow does not flood the object tree.
  bool gmshCadRequireActive(QString& error) const;
  bool updateGmshCadStage(QString& error);
  void endGmshCadSession();
  void validateGmshCadState();

  // Explicit right-click stage selection for Combine (see StageSelection.h).
  QSet<int> selectedStageIndices() const;
  void setStageSelection(const QSet<int>& stages);
  void clearStageSelection();
  void pruneStageSelection();

  MeshDocument m_doc;
  VtkView* m_view = nullptr;
  QTreeWidget* m_tree = nullptr;
  QMenu* m_operationsMenu = nullptr;
  QAction* m_operationsSearchAction = nullptr;
  QSet<int> m_visible; // pipeline indices currently drawn
  QSet<int> m_stageSelection; // persistent Combine selection across right-clicks
  bool m_updatingTree = false;

  // Viewport selection (S shortcut): face indices of m_selectionStage,
  // shared id space with extractSubmesh; filters run on the submesh.
  bool m_selectMode = false;
  int m_selectionStage = -1;
  QSet<int> m_selection;
  QSet<int> m_selectedEdges;
  QPushButton* m_selectBtn = nullptr;
  QLabel* m_selectLabel = nullptr;
  // Programmatic selection-mode shutdown (tree rebuild, stage delete) must
  // discard the pending selection instead of committing it as a new stage.
  bool m_suppressSelectionCommit = false;
  void updateSelectPanel();
  void validateSelection();
  void cancelSelectionSession();
  bool commitSelectionDeletions();
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

  std::unique_ptr<McpProtocol> m_mcpProtocol;
  std::unique_ptr<McpHttpServer> m_mcpServer;
  // Gmsh CAD session: one pipeline stage, refreshed in place.
  std::unique_ptr<GmshCadSession> m_gmshCad;
  int m_gmshCadStage = -1;
  // Set during destruction so an in-flight HTTP request stops waiting for
  // the GUI thread instead of deadlocking the server join.
  std::atomic<bool> m_mcpShuttingDown{false};
};

} // namespace meshbench
