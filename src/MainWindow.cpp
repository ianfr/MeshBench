#include "MainWindow.h"
#include "LeakFix.h"
#include "MeshOps.h"
#include "OperationSearch.h"
#include "StageSelection.h"

#include "McpHttpServer.h"
#include "McpProtocol.h"

#include "GmshCadSession.h"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>

namespace meshbench {

namespace {
// Search field for the Ctrl+Space operation launcher: Up/Down move the
// result selection without leaving the text box, so typing and choosing an
// operation stay in one place.
class OperationSearchLineEdit : public QLineEdit {
public:
  using QLineEdit::QLineEdit;
  QListWidget* results = nullptr;

protected:
  void keyPressEvent(QKeyEvent* e) override {
    if (results && (e->key() == Qt::Key_Down || e->key() == Qt::Key_Up)) {
      const int count = results->count();
      if (count == 0)
        return;
      const int step = e->key() == Qt::Key_Down ? 1 : -1;
      if (results->currentRow() < 0) {
        for (int i = 0; i < count; ++i) {
          const int row = step > 0 ? i : count - 1 - i;
          if (!results->item(row)->isHidden()) {
            results->setCurrentRow(row);
            break;
          }
        }
        return;
      }
      int row = results->currentRow();
      for (int n = 0; n < count; ++n) {
        row = (row + step + count) % count;
        if (!results->item(row)->isHidden()) {
          results->setCurrentRow(row);
          break;
        }
      }
      return;
    }
    QLineEdit::keyPressEvent(e);
  }
};
} // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle(QStringLiteral("MeshBench"));
  resize(1280, 800);

  auto* fileMenu = menuBar()->addMenu(QStringLiteral("&File"));
  fileMenu->addAction(QStringLiteral("Open mesh..."), QKeySequence::Open, this,
                      &MainWindow::openMesh);
  fileMenu->addAction(QStringLiteral("Import meshes..."), this,
                      &MainWindow::importMeshes);
  fileMenu->addAction(QStringLiteral("Import feature edges (eMesh)..."), this,
                      &MainWindow::importFeatureEdges);
  fileMenu->addAction(QStringLiteral("Save active stage..."), QKeySequence::Save,
                      this, &MainWindow::saveActiveMesh);
  fileMenu->addAction(QStringLiteral("Export scene image..."), this,
                      &MainWindow::exportSceneImage);
  fileMenu->addAction(QStringLiteral("Export feature edges as eMesh..."), this,
                      &MainWindow::saveActiveFeatureEdges);
  fileMenu->addSeparator();
  fileMenu->addAction(QStringLiteral("Clear"), this, &MainWindow::clearAll);
  fileMenu->addAction(QStringLiteral("Quit"), QKeySequence::Quit, this,
                      &QWidget::close);

  auto* opsMenu = menuBar()->addMenu(QStringLiteral("&Operations"));
  m_operationsMenu = opsMenu;
  m_operationsSearchAction =
      opsMenu->addAction(QStringLiteral("Search operations..."), this,
                         &MainWindow::showOperationsSearch);
  m_operationsSearchAction->setShortcut(
      QKeySequence(QStringLiteral("Ctrl+Space")));
  m_operationsSearchAction->setShortcutContext(Qt::WindowShortcut);
  opsMenu->addSeparator();
  opsMenu->addAction(QStringLiteral("Boolean with another mesh..."), this,
                     &MainWindow::runBoolean);
  opsMenu->addAction(QStringLiteral("Center at volume center"), this,
                     &MainWindow::runCenter);
  opsMenu->addAction(QStringLiteral("Transform (translate/rotate)..."), this,
                     &MainWindow::runTransform);
  opsMenu->addAction(QStringLiteral("Alpha wrap..."), this, &MainWindow::runAlphaWrap);
  opsMenu->addAction(QStringLiteral("Isotropic remesh..."), this,
                     &MainWindow::runIsotropicRemesh);
  opsMenu->addAction(QStringLiteral("Extract feature edges..."), this,
                     &MainWindow::runExtractFeatureEdges);
  opsMenu->addAction(QStringLiteral("Snap to feature edges..."), this,
                     &MainWindow::runSnapToFeatureEdges);
  opsMenu->addAction(QStringLiteral("Autoclean (experimental)..."), this,
                     &MainWindow::runAutoClean);
  opsMenu->addAction(QStringLiteral("Slice with plane..."), this, &MainWindow::runClip);
  opsMenu->addSeparator();
  opsMenu->addAction(QStringLiteral("Combine selected stages..."), this,
                     &MainWindow::runCombineSelected);
  opsMenu->addSeparator();
  opsMenu->addAction(QStringLiteral("Detect leaks"), this, &MainWindow::runDetectLeaks);
  opsMenu->addAction(QStringLiteral("Patch holes..."), this, &MainWindow::runPatchHoles);
  opsMenu->addAction(QStringLiteral("Introduce random holes..."), this,
                     &MainWindow::runIntroduceHoles);
  opsMenu->addAction(QStringLiteral("Shrinkwrap..."), this,
                     &MainWindow::runShrinkwrap);
  opsMenu->addSeparator();
  opsMenu->addAction(QStringLiteral("Create primitive shape..."), this,
                     &MainWindow::runCreatePrimitive);
  opsMenu->addAction(QStringLiteral("Remesh with Gmsh..."), this,
                     &MainWindow::runGmshRemesh);
  auto* viewMenu = menuBar()->addMenu(QStringLiteral("&View"));
  viewMenu->addAction(QStringLiteral("View from +X"), this,
                      [this] { m_view->setStandardView(StandardView::PosX); });
  viewMenu->addAction(QStringLiteral("View from -X"), this,
                      [this] { m_view->setStandardView(StandardView::NegX); });
  viewMenu->addAction(QStringLiteral("View from +Y"), this,
                      [this] { m_view->setStandardView(StandardView::PosY); });
  viewMenu->addAction(QStringLiteral("View from -Y"), this,
                      [this] { m_view->setStandardView(StandardView::NegY); });
  viewMenu->addAction(QStringLiteral("View from +Z"), this,
                      [this] { m_view->setStandardView(StandardView::PosZ); });
  viewMenu->addAction(QStringLiteral("View from -Z"), this,
                      [this] { m_view->setStandardView(StandardView::NegZ); });
  viewMenu->addSeparator();
  viewMenu->addAction(QStringLiteral("Reset camera"), this,
                      [this] { m_view->resetCamera(); });

  auto* splitter = new QSplitter(this);
  setCentralWidget(splitter);

  // --- Left: pipeline history sidebar ---
  m_tree = new QTreeWidget(splitter);
  m_tree->setHeaderLabels({QStringLiteral("Pipeline"), QStringLiteral("Info"),
                           QStringLiteral("Combine")});
  m_tree->setColumnWidth(0, 220);
  m_tree->setColumnWidth(2, 70);
  // ParaView-style: several stages can be selected at once (for Combine).
  m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection);
  m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(m_tree, &QTreeWidget::itemChanged, this, &MainWindow::onTreeItemChanged);
  connect(m_tree, &QTreeWidget::itemDoubleClicked, this,
          &MainWindow::onTreeItemActivated);
  auto* deleteShortcut = new QShortcut(QKeySequence::Delete, m_tree);
  deleteShortcut->setContext(Qt::WidgetShortcut);
  connect(deleteShortcut, &QShortcut::activated, this,
          &MainWindow::deleteSelectedStages);
  connect(m_tree, &QWidget::customContextMenuRequested, this, [this](const QPoint& p) {
    if (auto* item = m_tree->itemAt(p)) {
      // Read the index up front: activating a stage rebuilds the tree and
      // invalidates the QTreeWidgetItem pointer.
      const int idx = item->data(0, Qt::UserRole).toInt();
      // A right-click may transiently change Qt's tree selection. Restore
      // the explicit Combine set so the highlighted stages and the stages
      // Combine will use always agree.
      if (!m_stageSelection.isEmpty())
        setStageSelection(m_stageSelection);
      QMenu menu;
      // Modifier-clicks are not reliable on every platform/window manager,
      // so allow building a Combine selection one stage at a time here.
      auto* selectAction =
          menu.addAction(QStringLiteral("Select this stage for Combine"));
      selectAction->setCheckable(true);
      // Check the explicit persistent set: a right-click may transiently
      // select the item under the cursor, which must not look like an
      // already-added Combine stage.
      selectAction->setChecked(m_stageSelection.contains(idx));
      connect(selectAction, &QAction::toggled, this, [this, idx](bool on) {
        setStageSelection(toggleStageSelection(selectedStageIndices(), idx, on));
      });
      menu.addAction(QStringLiteral("Clear Combine selection"), this,
                     &MainWindow::clearStageSelection);
      menu.addSeparator();
      menu.addAction(QStringLiteral("Show only this stage"), this,
                     [this, idx] {
                       m_doc.setActive(idx);
                       for (int i = 0; i < m_doc.nodeCount(); ++i)
                         m_visible.remove(i);
                       m_visible.insert(m_doc.activeIndex());
                       refreshAll();
                     });
      menu.addAction(QStringLiteral("Transform (translate/rotate)..."), this,
                     [this, idx] {
                       m_doc.setActive(idx);
                       runTransform();
                     });
      menu.addAction(QStringLiteral("Revert to this stage (drop later ops)"),
                     this, &MainWindow::revertToSelectedStage);
      menu.addAction(QStringLiteral("Delete this stage (and downstream)"),
                     this, [this, idx] { deleteStages(QSet<int>{idx}); });
      menu.addAction(QStringLiteral("Combine selected stages..."), this,
                     &MainWindow::runCombineSelected);
      menu.exec(m_tree->viewport()->mapToGlobal(p));
    }
  });

  // --- Center: 3D viewport ---
  m_view = new VtkView(splitter);

  // --- Right: inspector ---
  auto* right = new QWidget(splitter);
  auto* rightLayout = new QVBoxLayout(right);

  auto* stageBox = new QWidget(right);
  auto* stageLayout = new QVBoxLayout(stageBox);
  stageLayout->addWidget(new QLabel(QStringLiteral("<b>Active stage</b>"), stageBox));
  m_stageInfo = new QLabel(stageBox);
  m_stageInfo->setWordWrap(true);
  m_stageInfo->setTextInteractionFlags(Qt::TextSelectableByMouse);
  stageLayout->addWidget(m_stageInfo);
  auto* opRow = new QHBoxLayout();
  opRow->addWidget(new QLabel(QStringLiteral("Opacity"), stageBox));
  m_opacitySpin = new QDoubleSpinBox(stageBox);
  m_opacitySpin->setRange(0.0, 1.0);
  m_opacitySpin->setSingleStep(0.05);
  m_opacitySpin->setValue(1.0);
  m_opacitySpin->setToolTip(QStringLiteral("Pin this stage's opacity "
                                           "(untouched stages stay automatic: "
                                           "opaque when active, faint "
                                           "otherwise)."));
  opRow->addWidget(m_opacitySpin, 1);
  stageLayout->addLayout(opRow);
  connect(m_opacitySpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double v) {
            if (!m_doc.hasMesh())
              return;
            m_doc.node(m_doc.activeIndex()).opacity = v;
            refreshView();
          });
  rightLayout->addWidget(stageBox);

  auto* probeBox = new QWidget(right);
  auto* probeLayout = new QVBoxLayout(probeBox);
  probeLayout->addWidget(
      new QLabel(QStringLiteral("<b>Leak probes</b> (green = inside, blue = outside)"),
                 probeBox));
  m_probeTable = new QTableWidget(0, 4, probeBox);
  m_probeTable->setHorizontalHeaderLabels(
      {QStringLiteral("x"), QStringLiteral("y"), QStringLiteral("z"), QStringLiteral("side")});
  m_probeTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  probeLayout->addWidget(m_probeTable);
  auto* probeBtns = new QHBoxLayout();
  auto* addIn = new QPushButton(QStringLiteral("+ inside"), probeBox);
  auto* addOut = new QPushButton(QStringLiteral("+ outside"), probeBox);
  auto* delProbe = new QPushButton(QStringLiteral("remove"), probeBox);
  auto* defaults = new QPushButton(QStringLiteral("defaults"), probeBox);
  probeBtns->addWidget(addIn);
  probeBtns->addWidget(addOut);
  probeBtns->addWidget(delProbe);
  probeBtns->addWidget(defaults);
  probeLayout->addLayout(probeBtns);
  connect(addIn, &QPushButton::clicked, this, [this] { addProbeRow(true); });
  connect(addOut, &QPushButton::clicked, this, [this] { addProbeRow(false); });
  connect(delProbe, &QPushButton::clicked, this, &MainWindow::removeSelectedProbes);
  connect(defaults, &QPushButton::clicked, this, &MainWindow::setDefaultProbes);
  auto* detectBtn = new QPushButton(QStringLiteral("Detect leaks"), probeBox);
  probeLayout->addWidget(detectBtn);
  connect(detectBtn, &QPushButton::clicked, this, &MainWindow::runDetectLeaks);
  m_leakSummary = new QLabel(probeBox);
  m_leakSummary->setWordWrap(true);
  probeLayout->addWidget(m_leakSummary);
  auto* showPaths = new QCheckBox(QStringLiteral("Show leak paths"), probeBox);
  showPaths->setChecked(false);
  showPaths->setToolTip(QStringLiteral("Draw every leak route the probe found "
                                       "(red BFS paths, orange walk routes) "
                                       "on the active LeakProbe stage."));
  probeLayout->addWidget(showPaths);
  auto* showGrid = new QCheckBox(QStringLiteral("Show voxel grid"), probeBox);
  showGrid->setChecked(false);
  showGrid->setToolTip(QStringLiteral("Draw the background voxel grid the "
                                      "active LeakProbe stage ran on."));
  probeLayout->addWidget(showGrid);
  connect(showPaths, &QCheckBox::toggled, this,
          [this](bool on) {
            m_view->setShowLeakPaths(on);
            refreshView();
          });
  connect(showGrid, &QCheckBox::toggled, this,
          [this](bool on) {
            m_view->setShowVoxelGrid(on);
            refreshView();
          });
  rightLayout->addWidget(probeBox);

  // --- Selection: viewport triangle picking for subset filters ---
  auto* selBox = new QWidget(right);
  auto* selLayout = new QVBoxLayout(selBox);
  selLayout->addWidget(new QLabel(
      QStringLiteral("<b>Selection</b> (S; click faces or amber edges)"),
      selBox));
  m_selectBtn = new QPushButton(QStringLiteral("Select (S)"), selBox);
  m_selectBtn->setCheckable(true);
  m_selectBtn->setToolTip(QStringLiteral(
      "Pick triangles on the active stage: click toggles one, drag selects "
      "a rectangle. Click an amber feature edge to select that edge. "
      "Leaving selection mode applies the selection as deletions in a new "
      "stage; the active stage is never modified."));
  selLayout->addWidget(m_selectBtn);
  m_selectLabel = new QLabel(QStringLiteral("No selection."), selBox);
  m_selectLabel->setWordWrap(true);
  selLayout->addWidget(m_selectLabel);
  auto* selBtns = new QHBoxLayout();
  auto* remeshSel = new QPushButton(QStringLiteral("Remesh selection..."), selBox);
  auto* detectSel = new QPushButton(QStringLiteral("Detect leaks"), selBox);
  selBtns->addWidget(remeshSel);
  selBtns->addWidget(detectSel);
  selLayout->addLayout(selBtns);
  auto* selBtns2 = new QHBoxLayout();
  auto* extractSel = new QPushButton(QStringLiteral("Extract as stage"), selBox);
  auto* deleteSelBtn = new QPushButton(QStringLiteral("Delete selected"), selBox);
  auto* clearSelBtn = new QPushButton(QStringLiteral("Clear"), selBox);
  selBtns2->addWidget(extractSel);
  selBtns2->addWidget(deleteSelBtn);
  selBtns2->addWidget(clearSelBtn);
  selLayout->addLayout(selBtns2);
  rightLayout->addWidget(selBox);
  connect(m_selectBtn, &QPushButton::toggled, this,
          &MainWindow::toggleSelectMode);
  connect(m_view, &VtkView::facesSelected, this, &MainWindow::onFacesSelected);
  connect(m_view, &VtkView::featureEdgePicked, this,
          &MainWindow::onFeatureEdgePicked);
  connect(m_view, &VtkView::featureEdgesSelected, this,
          &MainWindow::onFeatureEdgesSelected);
  connect(remeshSel, &QPushButton::clicked, this,
          &MainWindow::runRemeshSelection);
  connect(detectSel, &QPushButton::clicked, this,
          &MainWindow::runDetectLeaksSelection);
  connect(extractSel, &QPushButton::clicked, this,
          &MainWindow::runExtractSelection);
  connect(deleteSelBtn, &QPushButton::clicked, this, [this] {
    if (m_selection.isEmpty() && m_selectedEdges.isEmpty()) {
      QMessageBox::information(this, QStringLiteral("MeshBench"),
                               QStringLiteral("Select faces or feature edges "
                                              "first (S in the viewport)."));
      return;
    }
    if (m_selectBtn && m_selectBtn->isChecked())
      m_selectBtn->setChecked(false); // exits and commits the deletions
    else if (commitSelectionDeletions())
      updateSelectPanel();
  });
  connect(clearSelBtn, &QPushButton::clicked, this, &MainWindow::clearSelection);
  auto* selectShortcut = new QShortcut(QKeySequence(QStringLiteral("s")), m_view);
  selectShortcut->setContext(Qt::WidgetShortcut);
  connect(selectShortcut, &QShortcut::activated, this,
          [this] { m_selectBtn->toggle(); });

  auto* wrapBox = new QWidget(right);
  auto* wrapLayout = new QVBoxLayout(wrapBox);
  wrapLayout->addWidget(new QLabel(
      QStringLiteral("<b>Shrinkwrap</b> (click panels, paint weights)"), wrapBox));
  auto* wrapBtns = new QHBoxLayout();
  auto* newWrap = new QPushButton(QStringLiteral("New wrap..."), wrapBox);
  auto* rerunWrap = new QPushButton(QStringLiteral("Re-run"), wrapBox);
  m_pickBtn = new QPushButton(QStringLiteral("Select panels"), wrapBox);
  m_pickBtn->setCheckable(true);
  wrapBtns->addWidget(newWrap);
  wrapBtns->addWidget(rerunWrap);
  wrapBtns->addWidget(m_pickBtn);
  wrapLayout->addLayout(wrapBtns);
  connect(newWrap, &QPushButton::clicked, this, &MainWindow::runShrinkwrap);
  connect(rerunWrap, &QPushButton::clicked, this, &MainWindow::rerunShrinkwrap);
  connect(m_pickBtn, &QPushButton::toggled, this, &MainWindow::togglePickMode);
  connect(m_view, &VtkView::facePicked, this, &MainWindow::onFacePicked);
  m_selLabel = new QLabel(QStringLiteral("No wrap yet."), wrapBox);
  m_selLabel->setWordWrap(true);
  wrapLayout->addWidget(m_selLabel);
  auto* weightRow = new QHBoxLayout();
  weightRow->addWidget(new QLabel(QStringLiteral("Weight"), wrapBox));
  m_weightSlider = new QSlider(Qt::Horizontal, wrapBox);
  m_weightSlider->setRange(0, 300);
  m_weightSlider->setValue(100);
  m_weightSpin = new QDoubleSpinBox(wrapBox);
  m_weightSpin->setRange(0.0, 3.0);
  m_weightSpin->setSingleStep(0.05);
  m_weightSpin->setValue(1.0);
  weightRow->addWidget(m_weightSlider, 1);
  weightRow->addWidget(m_weightSpin);
  wrapLayout->addLayout(weightRow);
  connect(m_weightSlider, &QSlider::valueChanged, this,
          [this](int v) { m_weightSpin->setValue(v / 100.0); });
  connect(m_weightSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double v) {
            m_weightSlider->setValue(static_cast<int>(std::round(v * 100)));
          });
  auto* weightBtns = new QHBoxLayout();
  auto* applyW = new QPushButton(QStringLiteral("Apply to selection"), wrapBox);
  auto* clearSel = new QPushButton(QStringLiteral("Clear selection"), wrapBox);
  auto* resetW = new QPushButton(QStringLiteral("Reset weights"), wrapBox);
  weightBtns->addWidget(applyW);
  weightBtns->addWidget(clearSel);
  weightBtns->addWidget(resetW);
  wrapLayout->addLayout(weightBtns);
  connect(applyW, &QPushButton::clicked, this,
          &MainWindow::applyWeightToSelection);
  connect(clearSel, &QPushButton::clicked, this,
          &MainWindow::clearFaceSelection);
  connect(resetW, &QPushButton::clicked, this, &MainWindow::resetFaceWeights);
  rightLayout->addWidget(wrapBox);

  m_log = new QPlainTextEdit(right);
  m_log->setReadOnly(true);
  m_log->setMaximumBlockCount(500);
  rightLayout->addWidget(new QLabel(QStringLiteral("<b>Log</b>"), right));
  rightLayout->addWidget(m_log, 1);

  splitter->addWidget(m_tree);
  splitter->addWidget(m_view);
  splitter->addWidget(right);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 4);
  splitter->setStretchFactor(2, 2);

  connect(&m_doc, &MeshDocument::historyChanged, this, &MainWindow::refreshAll);
  connect(&m_doc, &MeshDocument::activeChanged, this, &MainWindow::refreshAll);

  log(QStringLiteral("MeshBench ready. File > Open mesh to begin."));
  log(QStringLiteral("Gmsh %1 available.").arg(gmshVersion()));
  startMcpServer();
}

MainWindow::~MainWindow() {
  // Stop the HTTP server (and join its thread) before any UI member is
  // destroyed: pending handlers must never touch a half-destructed window.
  m_mcpShuttingDown.store(true);
  if (m_gmshCad)
    m_gmshCad->abort();
  if (m_mcpServer)
    m_mcpServer->stop();
}

void MainWindow::log(const QString& msg) {
  m_log->appendPlainText(msg);
}

void MainWindow::showOperationsSearch() {
  if (!m_operationsMenu)
    return;
  QList<QAction*> actions;
  for (QAction* action : m_operationsMenu->actions()) {
    if (!action || action->isSeparator() || action == m_operationsSearchAction ||
        !action->isEnabled())
      continue;
    actions.append(action);
  }
  if (actions.isEmpty())
    return;

  QDialog dlg(this);
  dlg.setWindowTitle(QStringLiteral("Operations"));
  dlg.resize(560, 420);
  auto* layout = new QVBoxLayout(&dlg);
  auto* search = new OperationSearchLineEdit(&dlg);
  search->setPlaceholderText(QStringLiteral("Search operations..."));
  layout->addWidget(search);
  auto* hint = new QLabel(
      QStringLiteral("Type to filter. Enter or double-click runs an operation."),
      &dlg);
  hint->setEnabled(false);
  layout->addWidget(hint);
  auto* list = new QListWidget(&dlg);
  list->setSelectionMode(QAbstractItemView::SingleSelection);
  layout->addWidget(list, 1);
  search->results = list;

  for (int i = 0; i < actions.size(); ++i) {
    auto* item = new QListWidgetItem(actions[i]->text(), list);
    item->setData(Qt::UserRole, i);
  }

  QAction* chosen = nullptr;
  const auto choose = [&](QListWidgetItem* item) {
    if (!item || item->isHidden())
      return;
    const int index = item->data(Qt::UserRole).toInt();
    if (index >= 0 && index < actions.size()) {
      chosen = actions[index];
      dlg.accept();
    }
  };
  const auto chooseCurrent = [&] {
    QListWidgetItem* item = list->currentItem();
    if (!item || item->isHidden()) {
      for (int i = 0; i < list->count(); ++i) {
        if (!list->item(i)->isHidden()) {
          item = list->item(i);
          break;
        }
      }
    }
    choose(item);
  };
  const auto applyFilter = [&] {
    const QString query = search->text();
    QListWidgetItem* firstVisible = nullptr;
    for (int i = 0; i < list->count(); ++i) {
      QListWidgetItem* item = list->item(i);
      const bool match = operationSearchMatches(item->text(), query);
      item->setHidden(!match);
      if (match && !firstVisible)
        firstVisible = item;
    }
    if (firstVisible &&
        (!list->currentItem() || list->currentItem()->isHidden()))
      list->setCurrentItem(firstVisible);
  };

  connect(search, &QLineEdit::textChanged, &dlg,
          [&](const QString&) { applyFilter(); });
  connect(search, &QLineEdit::returnPressed, &dlg, chooseCurrent);
  connect(list, &QListWidget::itemActivated, &dlg,
          [&](QListWidgetItem* item) { choose(item); });
  applyFilter();
  search->setFocus();
  dlg.move(frameGeometry().center() -
           QPoint(dlg.width() / 2, dlg.height() / 2));
  if (dlg.exec() == QDialog::Accepted && chosen)
    chosen->trigger();
}

bool MainWindow::requireMesh() const {
  if (!m_doc.hasMesh() || m_doc.activeMesh().num_vertices() == 0) {
    QMessageBox::information(const_cast<MainWindow*>(this),
                             QStringLiteral("MeshBench"),
                             QStringLiteral("Open or activate a stage with a "
                                            "surface mesh first."));
    return false;
  }
  return true;
}

bool MainWindow::ensureActiveMesh(QString& error) const {
  error.clear();
  if (!m_doc.hasMesh() || m_doc.nodeCount() == 0) {
    error = QStringLiteral("No mesh loaded. Use file_open_mesh or "
                           "file_import_meshes first.");
    return false;
  }
  if (m_doc.activeIndex() < 0 || m_doc.activeIndex() >= m_doc.nodeCount()) {
    error = QStringLiteral("The active stage index is out of range.");
    return false;
  }
  if (m_doc.activeMesh().num_vertices() == 0) {
    error = QStringLiteral("The active stage has no surface mesh. Activate a "
                           "mesh stage first.");
    return false;
  }
  return true;
}

QSet<int> MainWindow::selectedStageIndices() const {
  QSet<int> treeSelection;
  for (const auto* item : m_tree->selectedItems())
    treeSelection.insert(item->data(0, Qt::UserRole).toInt());
  // Once the user has picked stages explicitly (right-click or the
  // Combine checkboxes), that set is authoritative. Otherwise fall back
  // to normal tree multi-selection.
  return resolveStageSelection(m_stageSelection, treeSelection);
}

void MainWindow::setStageSelection(const QSet<int>& stages) {
  if (!m_tree)
    return;
  m_stageSelection = stages;
  // Programmatic checkbox/selection updates must not recurse through
  // onTreeItemChanged.
  const QSignalBlocker blocker(m_tree);
  for (int i = 0; i < m_tree->topLevelItemCount(); ++i) {
    auto* item = m_tree->topLevelItem(i);
    const bool selected =
        m_stageSelection.contains(item->data(0, Qt::UserRole).toInt());
    item->setCheckState(2, selected ? Qt::Checked : Qt::Unchecked);
    item->setSelected(selected);
  }
}

void MainWindow::clearStageSelection() {
  setStageSelection({});
}

void MainWindow::pruneStageSelection() {
  m_stageSelection = validStageSelection(m_stageSelection, m_doc.nodeCount());
}

void MainWindow::openMesh() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Open mesh"), {},
      QStringLiteral("Meshes (*.stl *.off *.obj);;All files (*)"));
  if (path.isEmpty())
    return;
  QString error;
  if (!loadMeshFile(path, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::loadMeshFile(const QString& path, QString& error) {
  error.clear();
  if (path.isEmpty()) {
    error = QStringLiteral("No mesh path given.");
    return false;
  }
  Mesh mesh;
  if (!loadMesh(path, mesh, error))
    return false;
  const MeshStats s = computeStats(mesh);
  // Loading a new document invalidates any pending viewport selection.
  cancelSelectionSession();
  m_stageSelection.clear();
  m_tree->clearSelection();
  m_doc.setSource(std::move(mesh), QFileInfo(path).fileName());
  m_visible.clear();
  m_visible.insert(0);
  probesToTable({});
  log(QStringLiteral("Opened %1: %2 verts, %3 faces, %4 boundary edges.")
          .arg(path)
          .arg(s.vertices)
          .arg(s.faces)
          .arg(s.boundaryEdges));
  if (!s.closed)
    log(QStringLiteral("Note: mesh is not watertight; use Detect leaks / Patch holes."));
  // Exit check: the document must hold the newly loaded source stage.
  if (!m_doc.hasMesh() || m_doc.nodeCount() != 1) {
    error = QStringLiteral("Failed to install the loaded mesh.");
    return false;
  }
  return true;
}

void MainWindow::importMeshes() {
  const QStringList paths = QFileDialog::getOpenFileNames(
      this, QStringLiteral("Import meshes"), {},
      QStringLiteral("Meshes (*.stl *.off *.obj);;All files (*)"));
  if (paths.isEmpty())
    return;
  QString error;
  if (!importMeshFiles(paths, error) && !error.isEmpty())
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::importMeshFiles(const QStringList& paths, QString& error) {
  error.clear();
  if (paths.isEmpty()) {
    error = QStringLiteral("No mesh paths given.");
    return false;
  }
  int loaded = 0;
  QStringList failures;
  for (const QString& path : paths) {
    Mesh mesh;
    QString loadError;
    if (!loadMesh(path, mesh, loadError)) {
      failures.append(QStringLiteral("%1: %2").arg(path, loadError));
      continue;
    }
    const MeshStats s = computeStats(mesh);
    m_doc.appendSource(QFileInfo(path).fileName(), std::move(mesh));
    m_visible.insert(m_doc.activeIndex());
    refreshView();
    ++loaded;
    log(QStringLiteral("Imported %1: %2 verts, %3 faces, %4 boundary edges.")
            .arg(path)
            .arg(s.vertices)
            .arg(s.faces)
            .arg(s.boundaryEdges));
    if (!s.closed)
      log(QStringLiteral("Note: mesh is not watertight; use Detect leaks / Patch holes."));
  }
  if (loaded == 0) {
    error = failures.isEmpty() ? QStringLiteral("No meshes imported.")
                               : failures.join(QStringLiteral("\n"));
    return false;
  }
  if (!failures.isEmpty())
    log(QStringLiteral("Import warnings: %1").arg(failures.join(QStringLiteral("; "))));
  return true;
}

void MainWindow::importFeatureEdges() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Import feature edges (eMesh)"), {},
      QStringLiteral("OpenFOAM eMesh (*.eMesh *.emesh);;All files (*)"));
  if (path.isEmpty())
    return;
  QString error;
  if (!importFeatureEdgesFile(path, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::importFeatureEdgesFile(const QString& path, QString& error) {
  error.clear();
  if (path.isEmpty()) {
    error = QStringLiteral("No feature-edge path given.");
    return false;
  }
  FeatureEdges edges;
  if (!loadFeatureEdges(path, edges, error))
    return false;
  const int edgeCount = static_cast<int>(edges.edges.size());
  const QString fileName = QFileInfo(path).fileName();
  if (m_doc.hasMesh() && m_doc.activeMesh().num_vertices() > 0) {
    // Overlay imported edges on the active surface.
    const int source = m_doc.activeIndex();
    Mesh base = m_doc.activeMesh();
    m_doc.appendOperation(
        QStringLiteral("FeatureEdges"),
        QStringLiteral("imported %1 (%2 edges)").arg(fileName).arg(edgeCount),
        std::move(base), std::move(edges));
    m_visible.remove(source);
  } else {
    // No surface stage: import the eMesh as an independent edge object.
    m_doc.appendSource(QStringLiteral("eMesh: %1").arg(fileName), Mesh(),
                       std::move(edges));
  }
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Imported %1: %2 points, %3 feature edges.")
          .arg(fileName)
          .arg(m_doc.node(m_doc.activeIndex()).featureEdges.points.size())
          .arg(edgeCount));
  return true;
}

void MainWindow::runCombineSelected() {
  QSet<int> picked = selectedStageIndices();
  QList<int> idx;
  for (int i : picked) {
    if (i >= 0 && i < m_doc.nodeCount())
      idx.append(i);
  }
  std::sort(idx.begin(), idx.end());
  if (idx.size() < 2) {
    QMessageBox::information(
        this, QStringLiteral("MeshBench"),
        QStringLiteral("Select two or more tree stages first "
                       "(Ctrl/Cmd-click or Shift-click)."));
    return;
  }
  QDialog dlg(this);
  dlg.setWindowTitle(QStringLiteral("Combine selected stages"));
  auto* layout = new QVBoxLayout(&dlg);
  auto* combo = new QComboBox(&dlg);
  combo->addItems({QStringLiteral("Append (concatenate, always works)"),
                   QStringLiteral("Union (sequential boolean union)")});
  layout->addWidget(combo);
  layout->addWidget(new QLabel(
      QStringLiteral("%1 stages selected. Union needs closed meshes and "
                     "stops at the first pair that fails.")
          .arg(idx.size()),
      &dlg));
  auto* buttons =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  if (dlg.exec() != QDialog::Accepted)
    return;
  const bool unionMode = combo->currentIndex() == 1;
  QString error;
  if (!applyCombine(idx, unionMode, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyCombine(const QList<int>& stages, bool unionMode,
                              QString& error) {
  error.clear();
  if (!ensureActiveMesh(error) && stages.isEmpty())
    return false;
  QList<int> idx;
  for (int i : stages) {
    if (i >= 0 && i < m_doc.nodeCount())
      idx.append(i);
  }
  std::sort(idx.begin(), idx.end());
  if (idx.size() < 2) {
    error = QStringLiteral("Select two or more pipeline stages to combine.");
    return false;
  }

  Mesh out;
  QString details;
  if (!unionMode) {
    std::vector<const Mesh*> inputs;
    for (int i : idx)
      inputs.push_back(&m_doc.node(i).mesh);
    out = mergeMeshes(inputs);
    details = QStringLiteral("%1 stages appended").arg(idx.size());
  } else {
    out = m_doc.node(idx.front()).mesh;
    for (int k = 1; k < idx.size(); ++k) {
      Mesh next;
      if (!booleanOp(out, m_doc.node(idx[k]).mesh, BooleanOp::Union, next,
                     error)) {
        error = QStringLiteral("Union failed at stage %1: %2")
                    .arg(idx[k])
                    .arg(error);
        return false;
      }
      out = std::move(next);
    }
    details = QStringLiteral("union of %1 stages").arg(idx.size());
  }
  const MeshStats s = computeStats(out);
  if (s.faces == 0) {
    error = QStringLiteral("Combine produced an empty mesh.");
    return false;
  }
  m_doc.appendOperation(QStringLiteral("Combine"), details, std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Combined %1 stages (%2): %3 faces.")
          .arg(idx.size())
          .arg(details)
          .arg(s.faces));
  return true;
}

void MainWindow::deleteSelectedStages() {
  QSet<int> picked;
  for (const auto* item : m_tree->selectedItems())
    picked.insert(item->data(0, Qt::UserRole).toInt());
  if (picked.isEmpty()) {
    log(QStringLiteral("Delete: select one or more pipeline stages first."));
    return;
  }
  deleteStages(picked);
}

void MainWindow::deleteStages(const QSet<int>& stages) {
  if (!m_doc.hasMesh() || stages.isEmpty())
    return;
  QSet<int> requested;
  for (int stage : stages) {
    if (stage >= 0 && stage < m_doc.nodeCount())
      requested.insert(stage);
  }
  if (requested.isEmpty())
    return;

  const int oldCount = m_doc.nodeCount();
  const int oldActive = m_doc.activeIndex();
  // Leaving these modes on would leave them targeting stale indices.
  if (m_pickBtn && m_pickBtn->isChecked())
    m_pickBtn->setChecked(false);
  cancelSelectionSession();

  QMap<int, int> mapping;
  {
    // Remap every UI-side stage index before any refresh runs.
    const QSignalBlocker blocker(&m_doc);
    mapping = m_doc.removeNodes(requested);
  }
  const int removed = oldCount - mapping.size();
  if (removed <= 0)
    return; // safety: nothing was actually removed

  QSet<int> visible;
  for (int old : m_visible) {
    if (mapping.contains(old))
      visible.insert(mapping.value(old));
  }
  m_visible = visible;

  QSet<int> stageSelection;
  for (int old : m_stageSelection) {
    if (mapping.contains(old))
      stageSelection.insert(mapping.value(old));
  }
  m_stageSelection = stageSelection;

  if (m_selectionStage >= 0 && mapping.contains(m_selectionStage)) {
    m_selectionStage = mapping.value(m_selectionStage);
  } else {
    m_selectionStage = -1;
    m_selection.clear();
  }

  int wrapSource = m_wrapSource;
  int wrapStage = m_wrapStage;
  bool wrapLost = false;
  if (wrapSource >= 0) {
    if (mapping.contains(wrapSource))
      wrapSource = mapping.value(wrapSource);
    else
      wrapLost = true;
  }
  if (wrapStage >= 0) {
    if (mapping.contains(wrapStage))
      wrapStage = mapping.value(wrapStage);
    else
      wrapLost = true;
  }
  if (wrapLost) {
    m_wrapSource = -1;
    m_wrapStage = -1;
    m_faceWeights.clear();
    m_faceSelection.clear();
  } else {
    m_wrapSource = wrapSource;
    m_wrapStage = wrapStage;
  }

  if (!m_doc.hasMesh()) {
    m_visible.clear();
    m_stageSelection.clear();
    m_selection.clear();
    m_selectionStage = -1;
    m_view->clear();
    refreshAll();
    log(QStringLiteral("Deleted all %1 pipeline stage(s).").arg(removed));
    return;
  }

  const bool activeRemoved = oldActive >= 0 && !mapping.contains(oldActive);
  if (activeRemoved || m_visible.isEmpty())
    m_visible.insert(m_doc.activeIndex());

  refreshAll();
  // showDocument rebuilt actors against the new indices; re-apply the
  // remapped face-selection highlight.
  m_view->setSelection(m_selectionStage, m_selection);
  log(QStringLiteral("Deleted %1 pipeline stage(s), including downstream "
                     "dependants.")
          .arg(removed));
}

void MainWindow::saveActiveMesh() {
  if (!requireMesh())
    return;
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Save active stage"), {},
      QStringLiteral("STL (*.stl);;OFF (*.off);;OBJ (*.obj)"));
  if (path.isEmpty())
    return;
  QString error;
  if (!saveActiveMeshTo(path, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::saveActiveMeshTo(const QString& path, QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  if (path.isEmpty()) {
    error = QStringLiteral("No output path given.");
    return false;
  }
  if (!saveMesh(path, m_doc.activeMesh(), error))
    return false;
  log(QStringLiteral("Saved stage %1 to %2.").arg(m_doc.activeIndex()).arg(path));
  return true;
}

void MainWindow::saveActiveFeatureEdges() {
  if (!m_doc.hasMesh()) {
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Extract or import feature edges first."));
    return;
  }
  if (m_doc.node(m_doc.activeIndex()).featureEdges.empty()) {
    QMessageBox::information(
        this, QStringLiteral("MeshBench"),
        QStringLiteral("The active stage has no feature edges. Run "
                       "Operations > Extract feature edges first."));
    return;
  }
  QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Export feature edges as eMesh"), {},
      QStringLiteral("OpenFOAM eMesh (*.eMesh);;All files (*)"));
  if (path.isEmpty())
    return;
  QString error;
  if (!saveActiveFeatureEdgesTo(path, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::saveActiveFeatureEdgesTo(const QString& pathIn,
                                          QString& error) {
  error.clear();
  if (!m_doc.hasMesh()) {
    error = QStringLiteral("Extract or import feature edges first.");
    return false;
  }
  const auto& node = m_doc.node(m_doc.activeIndex());
  if (node.featureEdges.empty()) {
    error = QStringLiteral("The active stage has no feature edges. Run "
                           "Operations > Extract feature edges first.");
    return false;
  }
  if (pathIn.isEmpty()) {
    error = QStringLiteral("No output path given.");
    return false;
  }
  QString path = pathIn;
  if (QFileInfo(path).suffix().isEmpty())
    path += QStringLiteral(".eMesh");
  if (!writeFeatureEdgeMesh(path, node.featureEdges, error))
    return false;
  log(QStringLiteral("Wrote %1 feature edges to %2.")
          .arg(node.featureEdges.edges.size())
          .arg(path));
  return true;
}

void MainWindow::exportSceneImage() {
  QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Export scene image"), {},
      QStringLiteral("PNG image (*.png);;All files (*)"));
  if (path.isEmpty())
    return;
  if (QFileInfo(path).suffix().isEmpty())
    path += QStringLiteral(".png");
  QString error;
  if (!saveSceneImageTo(path, 0, 0, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  log(QStringLiteral("Exported scene image to %1.").arg(path));
}

bool MainWindow::saveSceneImageTo(const QString& path, int width, int height,
                                  QString& error) {
  error.clear();
  if (path.isEmpty()) {
    error = QStringLiteral("No output path given.");
    return false;
  }
  return m_view->saveSceneImage(path, width, height, error);
}

void MainWindow::clearAll() {
  cancelSelectionSession();
  m_doc.clear();
  m_visible.clear();
  m_stageSelection.clear();
  m_view->clear();
  m_selectMode = false;
  m_selectionStage = -1;
  m_selection.clear();
  m_selectedEdges.clear();
  rebuildTree();
  updateStagePanel();
  updateSelectPanel();
}

static bool paramDialog(QWidget* parent, const QString& title,
                        const QList<std::pair<QString, double>>& doubles,
                        const QList<std::pair<QString, int>>& ints,
                        QMap<QString, double>& dOut, QMap<QString, int>& iOut,
                        const QList<std::pair<QString, bool>>& bools = {},
                        QMap<QString, bool>* bOut = nullptr) {
  QDialog dlg(parent);
  dlg.setWindowTitle(title);
  auto* form = new QFormLayout(&dlg);
  QMap<QString, QDoubleSpinBox*> dBoxes;
  for (const auto& [name, val] : doubles) {
    auto* box = new QDoubleSpinBox(&dlg);
    box->setRange(-1e9, 1e9);
    box->setDecimals(6);
    box->setValue(val);
    form->addRow(name, box);
    dBoxes[name] = box;
  }
  QMap<QString, QSpinBox*> iBoxes;
  for (const auto& [name, val] : ints) {
    auto* box = new QSpinBox(&dlg);
    box->setRange(1, 1000000);
    box->setValue(val);
    form->addRow(name, box);
    iBoxes[name] = box;
  }
  QMap<QString, QCheckBox*> bBoxes;
  for (const auto& [name, val] : bools) {
    auto* box = new QCheckBox(&dlg);
    box->setChecked(val);
    form->addRow(name, box);
    bBoxes[name] = box;
  }
  auto* buttons =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
  form->addRow(buttons);
  QObject::connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  if (dlg.exec() != QDialog::Accepted)
    return false;
  for (auto it = dBoxes.begin(); it != dBoxes.end(); ++it)
    dOut[it.key()] = it.value()->value();
  for (auto it = iBoxes.begin(); it != iBoxes.end(); ++it)
    iOut[it.key()] = it.value()->value();
  if (bOut) {
    for (auto it = bBoxes.begin(); it != bBoxes.end(); ++it)
      (*bOut)[it.key()] = it.value()->isChecked();
  }
  return true;
}

void MainWindow::runBoolean() {
  if (!requireMesh())
    return;
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Second mesh for boolean"), {},
      QStringLiteral("Meshes (*.stl *.off *.obj);;All files (*)"));
  if (path.isEmpty())
    return;
  QDialog dlg(this);
  dlg.setWindowTitle(QStringLiteral("Boolean operation"));
  auto* layout = new QVBoxLayout(&dlg);
  auto* combo = new QComboBox(&dlg);
  combo->addItems({QStringLiteral("Union"), QStringLiteral("Intersection"),
                   QStringLiteral("Difference (active minus other)")});
  layout->addWidget(combo);
  auto* buttons =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  if (dlg.exec() != QDialog::Accepted)
    return;
  const auto op = static_cast<BooleanOp>(combo->currentIndex());
  QString error;
  if (!applyBoolean(path, op, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyBoolean(const QString& path, BooleanOp op,
                              QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  if (path.isEmpty()) {
    error = QStringLiteral("No second mesh path given.");
    return false;
  }
  Mesh other;
  if (!loadMesh(path, other, error))
    return false;
  Mesh out;
  if (!booleanOp(m_doc.activeMesh(), other, op, out, error))
    return false;
  static const char* names[] = {"Union", "Intersection", "Difference"};
  const int index = static_cast<int>(op);
  if (index < 0 || index > 2) {
    error = QStringLiteral("Invalid boolean operation.");
    return false;
  }
  const MeshStats s = computeStats(out);
  m_doc.appendOperation(QString::fromUtf8(names[index]),
                        QStringLiteral("with %1").arg(QFileInfo(path).fileName()),
                        std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Boolean %1: %2 faces, volume %3.")
          .arg(QString::fromUtf8(names[index]))
          .arg(s.faces)
          .arg(s.volume));
  return true;
}

void MainWindow::runCenter() {
  if (!requireMesh())
    return;
  QString error;
  if (!applyCenter(error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyCenter(QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  Mesh out = m_doc.activeMesh();
  if (!centerAtVolumeCentroid(out, error))
    return false;
  m_doc.appendOperation(QStringLiteral("Center"), QStringLiteral("at volume center"),
                        std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Centered geometry at center of volume."));
  return true;
}

void MainWindow::runTransform() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> i;
  QMap<QString, bool> bv;
  if (!paramDialog(this, QStringLiteral("Transform (translate + rotate)"),
                   {{QStringLiteral("Translate X"), 0.0},
                    {QStringLiteral("Translate Y"), 0.0},
                    {QStringLiteral("Translate Z"), 0.0},
                    {QStringLiteral("Rotate X (deg)"), 0.0},
                    {QStringLiteral("Rotate Y (deg)"), 0.0},
                    {QStringLiteral("Rotate Z (deg)"), 0.0}},
                   {}, d, i,
                   {{QStringLiteral("Rotate about mesh center"), true}}, &bv))
    return;
  TransformParams params;
  params.translate[0] = d[QStringLiteral("Translate X")];
  params.translate[1] = d[QStringLiteral("Translate Y")];
  params.translate[2] = d[QStringLiteral("Translate Z")];
  params.rotateDeg[0] = d[QStringLiteral("Rotate X (deg)")];
  params.rotateDeg[1] = d[QStringLiteral("Rotate Y (deg)")];
  params.rotateDeg[2] = d[QStringLiteral("Rotate Z (deg)")];
  params.rotateAboutCenter = bv.value(QStringLiteral("Rotate about mesh center"), true);
  QString error;
  if (!applyTransform(params, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyTransform(const TransformParams& params, QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  const std::size_t inFaces = m_doc.activeMesh().num_faces();
  Mesh out = m_doc.activeMesh();
  if (!transformMesh(out, params, error))
    return false;
  // Exit check: the filter must return the same topology it consumed.
  if (out.num_vertices() == 0 || out.num_faces() != inFaces) {
    error = QStringLiteral("Transform produced an invalid mesh.");
    return false;
  }
  const auto fmt = [](double v) { return QString::number(v, 'g', 6); };
  const QString pivot = params.rotateAboutCenter ? QStringLiteral("center")
                                                 : QStringLiteral("origin");
  const QString details =
      QStringLiteral("t=(%1,%2,%3) r=(%4,%5,%6) deg about %7")
          .arg(fmt(params.translate[0]), fmt(params.translate[1]),
               fmt(params.translate[2]), fmt(params.rotateDeg[0]),
               fmt(params.rotateDeg[1]), fmt(params.rotateDeg[2]), pivot);
  m_doc.appendOperation(QStringLiteral("Transform"), details, std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Transformed stage: %1.").arg(details));
  return true;
}

void MainWindow::runAlphaWrap() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> i;
  if (!paramDialog(this, QStringLiteral("CGAL alpha wrap"),
                   {{"Alpha denominator (bbox diag / alpha)", 500.0},
                    {"Offset denominator (bbox diag / offset)", 1000.0}},
                   {}, d, i))
    return;
  QString error;
  if (!applyAlphaWrap(d["Alpha denominator (bbox diag / alpha)"],
                      d["Offset denominator (bbox diag / offset)"], error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyAlphaWrap(double alphaDenominator,
                                double offsetDenominator, QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  // Docs-page style relative parameters: scale by the active mesh bbox.
  const MeshStats inStats = computeStats(m_doc.activeMesh());
  const double dx = inStats.bboxMax[0] - inStats.bboxMin[0];
  const double dy = inStats.bboxMax[1] - inStats.bboxMin[1];
  const double dz = inStats.bboxMax[2] - inStats.bboxMin[2];
  const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
  double alpha = 0.0, offset = 0.0;
  if (!alphaWrapParams(diag, alphaDenominator, offsetDenominator, alpha, offset,
                       error))
    return false;
  Mesh out;
  if (!alphaWrap(m_doc.activeMesh(), alpha, offset, out, error))
    return false;
  const MeshStats s = computeStats(out);
  m_doc.appendOperation(
      QStringLiteral("AlphaWrap"),
      QStringLiteral("alpha=diag/%1 (%2) offset=diag/%3 (%4)")
          .arg(alphaDenominator)
          .arg(alpha)
          .arg(offsetDenominator)
          .arg(offset),
      std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Alpha wrap: %1 faces, closed=%2.")
          .arg(s.faces)
          .arg(s.closed ? QStringLiteral("yes") : QStringLiteral("no")));
  return true;
}

void MainWindow::runIsotropicRemesh() {
  if (!requireMesh())
    return;
  const MeshStats inStats = computeStats(m_doc.activeMesh());
  const double dx = inStats.bboxMax[0] - inStats.bboxMin[0];
  const double dy = inStats.bboxMax[1] - inStats.bboxMin[1];
  const double dz = inStats.bboxMax[2] - inStats.bboxMin[2];
  const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double defaultSize = diag > 0.0 ? diag / 50.0 : 0.1;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("CGAL isotropic remesh"),
                   {{QStringLiteral("Target edge length"), defaultSize},
                    {QStringLiteral("Feature angle (deg)"), 30.0}},
                   {{QStringLiteral("Iterations"), 3}}, d, iv))
    return;
  QString error;
  if (!applyIsotropicRemesh(d[QStringLiteral("Target edge length")],
                            iv[QStringLiteral("Iterations")],
                            d[QStringLiteral("Feature angle (deg)")], error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyIsotropicRemesh(double targetEdgeLength, int iterations,
                                      double featureAngleDeg,
                                      QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  const MeshStats inStats = computeStats(m_doc.activeMesh());
  Mesh out;
  if (!isotropicRemesh(m_doc.activeMesh(), targetEdgeLength, iterations,
                       featureAngleDeg, out, error))
    return false;
  const auto fmt = [](double v) { return QString::number(v, 'g', 6); };
  const MeshStats s = computeStats(out);
  m_doc.appendOperation(
      QStringLiteral("IsotropicRemesh"),
      QStringLiteral("edge=%1 iters=%2 feature=%3 deg")
          .arg(fmt(targetEdgeLength))
          .arg(iterations)
          .arg(fmt(featureAngleDeg)),
      std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Isotropic remesh: %1 -> %2 faces, closed=%3.")
          .arg(inStats.faces)
          .arg(s.faces)
          .arg(s.closed ? QStringLiteral("yes") : QStringLiteral("no")));
  return true;
}

void MainWindow::runExtractFeatureEdges() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  QMap<QString, bool> bv;
  if (!paramDialog(this, QStringLiteral("Extract feature edges"),
                   {{QStringLiteral("Feature angle (deg)"), 30.0}}, {}, d, iv,
                   {{QStringLiteral("Include open boundary edges"), true}},
                   &bv))
    return;
  QString error;
  if (!applyExtractFeatureEdges(
          d[QStringLiteral("Feature angle (deg)")],
          bv.value(QStringLiteral("Include open boundary edges"), true), error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyExtractFeatureEdges(double featureAngleDeg,
                                          bool includeBoundary,
                                          QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  FeatureEdges edges;
  if (!extractFeatureEdges(m_doc.activeMesh(), featureAngleDeg, includeBoundary,
                           edges, error))
    return false;
  if (edges.empty()) {
    error = QStringLiteral("No feature edges found at this angle. Try a "
                           "smaller feature angle or include boundary edges.");
    return false;
  }
  const int edgeCount = static_cast<int>(edges.edges.size());
  const int source = m_doc.activeIndex();
  Mesh base = m_doc.activeMesh();
  m_doc.appendOperation(
      QStringLiteral("FeatureEdges"),
      QStringLiteral("%1 edges, angle=%2 deg, boundary=%3")
          .arg(edgeCount)
          .arg(featureAngleDeg)
          .arg(includeBoundary ? QStringLiteral("yes") : QStringLiteral("no")),
      std::move(base), std::move(edges));
  m_visible.remove(source);
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Extracted %1 feature edges (%2 points).")
          .arg(edgeCount)
          .arg(m_doc.node(m_doc.activeIndex()).featureEdges.points.size()));
  return true;
}

void MainWindow::runSnapToFeatureEdges() {
  if (!requireMesh())
    return;
  const int active = m_doc.activeIndex();
  QList<int> candidates;
  for (int i = 0; i < m_doc.nodeCount(); ++i) {
    if (!m_doc.node(i).featureEdges.empty())
      candidates.append(i);
  }
  if (candidates.isEmpty()) {
    QMessageBox::information(
        this, QStringLiteral("MeshBench"),
        QStringLiteral("No feature edge set is available. Extract feature "
                       "edges or import an eMesh first, then apply this "
                       "filter to the target stage."));
    return;
  }

  QDialog dlg(this);
  dlg.setWindowTitle(QStringLiteral("Snap to feature edges"));
  auto* layout = new QVBoxLayout(&dlg);
  auto* form = new QFormLayout();
  layout->addLayout(form);
  auto* combo = new QComboBox(&dlg);
  int preferred = 0;
  for (int k = 0; k < candidates.size(); ++k) {
    const int i = candidates[k];
    const auto& node = m_doc.node(i);
    combo->addItem(QStringLiteral("%1: %2 (%3 edges)")
                       .arg(i)
                       .arg(node.label)
                       .arg(node.featureEdges.edges.size()),
                   i);
    // Prefer the closest earlier feature-edge stage, which is normally the
    // one extracted from the original surface before alpha wrapping etc.
    if (i < active)
      preferred = k;
  }
  combo->setCurrentIndex(preferred);
  form->addRow(QStringLiteral("Feature edge set"), combo);

  const MeshStats stats = computeStats(m_doc.activeMesh());
  const double dx = stats.bboxMax[0] - stats.bboxMin[0];
  const double dy = stats.bboxMax[1] - stats.bboxMin[1];
  const double dz = stats.bboxMax[2] - stats.bboxMin[2];
  const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
  auto* sizeSpin = new QDoubleSpinBox(&dlg);
  sizeSpin->setRange(1e-6, 1e9);
  sizeSpin->setDecimals(6);
  sizeSpin->setValue(diag > 0.0 ? diag / 50.0 : 0.1);
  form->addRow(QStringLiteral("Target edge length"), sizeSpin);
  auto* angleSpin = new QDoubleSpinBox(&dlg);
  angleSpin->setRange(0.0, 30.0);
  angleSpin->setDecimals(1);
  angleSpin->setValue(20.0);
  angleSpin->setToolTip(QStringLiteral("Minimum facet angle. Higher values "
                                       "reduce sliver triangles but increase "
                                       "the triangle count."));
  form->addRow(QStringLiteral("Min facet angle (deg)"), angleSpin);

  auto* buttons =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                           &dlg);
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  if (dlg.exec() != QDialog::Accepted)
    return;

  const int source = combo->currentData().toInt();
  QString error;
  if (!applySnapToFeatureEdges(source, sizeSpin->value(), angleSpin->value(),
                               error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applySnapToFeatureEdges(int sourceStage,
                                         double targetEdgeLength,
                                         double minFacetAngleDeg,
                                         QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;

  // Resolve the feature-edge stage: explicit index, else the nearest earlier
  // stage carrying edges (normally the set extracted before alpha wrapping).
  QList<int> candidates;
  for (int i = 0; i < m_doc.nodeCount(); ++i) {
    if (!m_doc.node(i).featureEdges.empty())
      candidates.append(i);
  }
  if (candidates.isEmpty()) {
    error = QStringLiteral("No feature edge set is available. Extract feature "
                           "edges or import an eMesh first.");
    return false;
  }
  int source = sourceStage;
  if (source < 0 || source >= m_doc.nodeCount() ||
      m_doc.node(source).featureEdges.empty()) {
    source = candidates.front();
    for (int i : candidates) {
      if (i < m_doc.activeIndex())
        source = i;
    }
  }
  if (m_doc.node(source).featureEdges.empty()) {
    error = QStringLiteral("Stage %1 has no feature edges.").arg(sourceStage);
    return false;
  }
  // Copy the edge set before appending: a filter on an older active stage
  // truncates the redo tail (which can include the edge-set stage).
  const FeatureEdges edgeSet = m_doc.node(source).featureEdges;
  Mesh out;
  if (!snapToFeatureEdges(m_doc.activeMesh(), edgeSet, targetEdgeLength,
                          minFacetAngleDeg, out, error))
    return false;
  const auto fmt = [](double v) { return QString::number(v, 'g', 6); };
  const MeshStats s = computeStats(out);
  m_doc.appendOperation(
      QStringLiteral("SnapToFeatureEdges"),
      QStringLiteral("from stage %1 (%2 edges), edge=%3, angle=%4 deg")
          .arg(source)
          .arg(edgeSet.edges.size())
          .arg(fmt(targetEdgeLength))
          .arg(fmt(minFacetAngleDeg)),
      std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Snapped to %1 feature edges from stage %2: %3 faces.")
          .arg(edgeSet.edges.size())
          .arg(source)
          .arg(s.faces));
  return true;
}

void MainWindow::runAutoClean() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(
          this, QStringLiteral("Autoclean (experimental)"),
          {{QStringLiteral("Small triangle ratio (0-1)"), 0.35},
           {QStringLiteral("Max region size (% bbox diag)"), 5.0}},
          {{QStringLiteral("Min small faces"), 8},
           {QStringLiteral("Grow rings"), 1},
           {QStringLiteral("Max hole edges"), 256}},
          d, iv))
    return;

  AutoCleanParams params;
  params.detailSizeRatio = d[QStringLiteral("Small triangle ratio (0-1)")];
  params.maxRegionFraction =
      d[QStringLiteral("Max region size (% bbox diag)")] / 100.0;
  params.minSmallFaces = iv[QStringLiteral("Min small faces")];
  params.growRings = iv[QStringLiteral("Grow rings")];
  params.maxHoleEdges = iv[QStringLiteral("Max hole edges")];

  AutoCleanResult result;
  QString error;
  if (!applyAutoClean(params, result, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyAutoClean(const AutoCleanParams& params,
                                AutoCleanResult& result, QString& error) {
  error.clear();
  result = {};
  if (!ensureActiveMesh(error))
    return false;
  Mesh out = m_doc.activeMesh();
  if (!autoCleanMesh(out, params, result, error))
    return false;
  for (const auto& warning : result.warnings)
    log(QStringLiteral("Autoclean warning: %1").arg(warning));
  if (result.clustersRemoved == 0 || result.facesRemoved == 0) {
    error = QStringLiteral("No small complex clusters matched the current "
                           "parameters. Try a larger small-triangle ratio or "
                           "region size.");
    return false;
  }

  const QString details =
      QStringLiteral("%1 clusters, %2 faces removed, %3 holes patched")
          .arg(result.clustersRemoved)
          .arg(result.facesRemoved)
          .arg(result.holesPatched);
  m_doc.appendOperation(QStringLiteral("Autoclean (experimental)"), details,
                        std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Autoclean: %1 cluster(s), %2 face(s) removed, %3 hole(s) "
                     "patched smoothly, %4 hole(s) skipped.")
          .arg(result.clustersRemoved)
          .arg(result.facesRemoved)
          .arg(result.holesPatched)
          .arg(result.holesSkipped));
  return true;
}

void MainWindow::runClip() {
  if (!requireMesh())
    return;
  const MeshStats s = computeStats(m_doc.activeMesh());
  const double cx = (s.bboxMin[0] + s.bboxMax[0]) / 2;
  const double cy = (s.bboxMin[1] + s.bboxMax[1]) / 2;
  const double cz = (s.bboxMin[2] + s.bboxMax[2]) / 2;
  QMap<QString, double> d;
  QMap<QString, int> i;
  if (!paramDialog(this, QStringLiteral("Slice with plane (keeps negative side)"),
                   {{"Origin x", cx},
                    {"Origin y", cy},
                    {"Origin z", cz},
                    {"Normal x", 0.0},
                    {"Normal y", 0.0},
                    {"Normal z", 1.0}},
                   {}, d, i))
    return;
  QString error;
  if (!applyClip(makePoint(d["Origin x"], d["Origin y"], d["Origin z"]),
                 Vector3(d["Normal x"], d["Normal y"], d["Normal z"]), error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyClip(const Point3& origin, const Vector3& normal,
                           QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  Mesh out;
  if (!clipWithPlane(m_doc.activeMesh(), origin, normal, out, error))
    return false;
  const MeshStats sliced = computeStats(out);
  m_doc.appendOperation(
      QStringLiteral("Slice"),
      QStringLiteral("n=(%1,%2,%3)")
          .arg(toDouble(normal.x()))
          .arg(toDouble(normal.y()))
          .arg(toDouble(normal.z())),
      std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Sliced with plane; %1 faces remain.").arg(sliced.faces));
  return true;
}

void MainWindow::runDetectLeaks() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("Leak detection grid"), {},
                   {{"Voxels per axis", ProbeParams{}.gridRes}}, d, iv))
    return;
  QString error;
  if (!probeMeshAndAppend(m_doc.activeMesh(), QStringLiteral("LeakProbe"),
                          iv["Voxels per axis"], error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::probeMeshAndAppend(const Mesh& mesh, const QString& stageLabel,
                                    int gridRes, QString& error) {
  error.clear();
  if (mesh.num_vertices() == 0) {
    error = QStringLiteral("Cannot probe an empty mesh.");
    return false;
  }
  if (gridRes < 2) {
    error = QStringLiteral("Voxels per axis must be at least 2.");
    return false;
  }
  const auto probes = probesFromTable();
  m_doc.setProbes(probes);
  std::vector<std::array<double, 3>> inPts, outPts;
  for (const auto& pr : probes) {
    if (pr.inside)
      inPts.push_back({pr.x, pr.y, pr.z});
    else
      outPts.push_back({pr.x, pr.y, pr.z});
  }
  ProbeParams probeParams;
  probeParams.gridRes = gridRes;
  const auto res = detectLeaks(mesh, inPts, outPts, probeParams);
  const auto loops = findHoleLoops(mesh);
  LeakReport report;
  report.probed = true;
  report.hasLeak = res.hasLeak;
  report.holeLoops = loops.size();
  report.regions = res.regions;
  for (const auto& p : res.paths) {
    QVector<std::array<double, 3>> line;
    for (const auto& q : p.polyline)
      line.append(q);
    report.leakPaths.append(line);
  }
  for (const auto& p : res.walkPaths) {
    QVector<std::array<double, 3>> line;
    for (const auto& q : p.polyline)
      line.append(q);
    report.walkPaths.append(line);
  }
  report.gridRes = res.gridRes;
  report.gridOrigin = res.gridOrigin;
  report.cellSize = res.cellSize;
  report.warnings = res.warnings;
  // ParaView-style: probing applies a filter, so it appends a new tree item
  // carrying the mesh plus this report, leaving the source stage untouched.
  // The input eye goes off (as in ParaView): the probe duplicates the input
  // geometry, and drawing both would z-fight instead of showing a solid.
  const int leakSource = m_doc.activeIndex();
  Mesh probed = mesh;
  m_doc.appendOperation(
      stageLabel,
      QStringLiteral("%1 loops, %2 leak(s)").arg(loops.size()).arg(res.paths.size()),
      std::move(probed));
  m_doc.setLeakReport(report);
  m_visible.remove(leakSource);
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  for (const auto& w : res.warnings)
    log(QStringLiteral("Probe warning: %1").arg(w));
  log(QStringLiteral("Leak probe: %1 cells (%2 faces blocked), %3 regions, "
                     "%4 hole loops, %5 leak(s).")
          .arg(res.totalCells)
          .arg(res.blockedFaces)
          .arg(res.regions)
          .arg(loops.size())
          .arg(res.paths.size()));
  log(res.hasLeak ? QStringLiteral("LEAK: an inside point reaches an outside "
                                   "point (red path). Patch the highlighted "
                                   "hole loops.")
                  : QStringLiteral("No leak: inside and outside points are "
                                   "separated."));
  // Exit check: the probe report must be attached to the new active stage.
  if (!m_doc.hasMesh() || !m_doc.node(m_doc.activeIndex()).leaks.probed) {
    error = QStringLiteral("Leak probe did not attach a report to the stage.");
    return false;
  }
  return true;
}

void MainWindow::runPatchHoles() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  QMap<QString, bool> bv;
  if (!paramDialog(this, QStringLiteral("Patch holes"), {},
                   {{"Max loop edges", 64}}, d, iv,
                   {{"Patch every boundary loop (ignore leak paths)", false}},
                   &bv))
    return;
  const bool patchAll = bv.value(QStringLiteral("Patch every boundary loop (ignore leak paths)"), false);
  QString error;
  if (!applyPatchHoles(iv["Max loop edges"], patchAll, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyPatchHoles(int maxLoopEdges, bool patchAll,
                                 QString& error, int* loopsFilled) {
  error.clear();
  if (loopsFilled)
    *loopsFilled = 0;
  if (!ensureActiveMesh(error))
    return false;
  if (maxLoopEdges < 3) {
    error = QStringLiteral("Max loop edges must be at least 3.");
    return false;
  }
  // Default: only patch holes that leak paths actually traverse. That
  // needs probes, so re-probe silently here.
  QSet<int> leakingLoops;
  LeakProbeResult silentProbe; // kept so the PatchHoles stage can show
                               // the routes/grid its patching was based on
  if (!patchAll) {
    const auto probes = probesFromTable();
    if (probes.isEmpty()) {
      error = QStringLiteral("Path-driven patching needs inside/outside "
                             "probes. Add probes, or set patch_all=true.");
      return false;
    }
    std::vector<std::array<double, 3>> inPts, outPts;
    for (const auto& pr : probes) {
      if (pr.inside)
        inPts.push_back({pr.x, pr.y, pr.z});
      else
        outPts.push_back({pr.x, pr.y, pr.z});
    }
    silentProbe = detectLeaks(m_doc.activeMesh(), inPts, outPts, ProbeParams{});
    const auto& res = silentProbe;
    for (const auto& w : res.warnings)
      log(QStringLiteral("Probe warning: %1").arg(w));
    if (!res.hasLeak) {
      log(QStringLiteral("Patch holes: no leak found, nothing to patch."));
      return true; // valid no-op result
    }
    // Bridge analysis (complete: every hole on some inside->outside route)
    // instead of sampling whichever holes routes happened to cross.
    leakingLoops = res.bridgingLoops;
    log(QStringLiteral("Patch holes: %1 BFS path(s), %2 walk route(s); bridge "
                       "analysis implicates %3 of %4 hole loop(s).")
            .arg(res.paths.size())
            .arg(res.walkPaths.size())
            .arg(leakingLoops.size())
            .arg(res.loops.size()));
    if (leakingLoops.isEmpty()) {
      log(QStringLiteral("Patch holes: no hole loop bridges inside to outside; "
                         "set patch_all=true to patch anyway."));
      return true; // valid no-op result
    }
  }
  Mesh out = m_doc.activeMesh();
  const PatchResult r = patchHoles(out, maxLoopEdges, leakingLoops);
  if (loopsFilled)
    *loopsFilled = r.loopsFilled;
  for (const auto& w : r.warnings)
    log(QStringLiteral("Patch warning: %1").arg(w));
  log(QStringLiteral("Patch holes: %1 loops found, %2 filled (%3 faces), %4 "
                     "skipped as too large, %5 skipped (not on a leak path).")
          .arg(r.loopsFound)
          .arg(r.loopsFilled)
          .arg(r.facesAdded)
          .arg(r.loopsSkippedTooLarge)
          .arg(r.loopsSkippedNotLeaking));
  if (r.loopsFilled > 0) {
    // Same input-hiding as LeakProbe: the patched mesh overlaps the input
    // almost everywhere, so the input eye goes off to keep it solid.
    const int patchSource = m_doc.activeIndex();
    m_doc.appendOperation(
        QStringLiteral("PatchHoles"),
        QStringLiteral("%1 loops filled").arg(r.loopsFilled), std::move(out));
    m_visible.remove(patchSource);
    m_visible.insert(m_doc.activeIndex());
    refreshView();
    if (!patchAll) {
      // Keep the routes/grid this patching was based on (silent re-probe
      // above) on the new stage so the view toggles can display them even
      // when no Detect-leaks run precedes patching. probed stays false:
      // the patched mesh is verdict-free, a plain stage with display data.
      LeakReport routes;
      for (const auto& p : silentProbe.paths) {
        QVector<std::array<double, 3>> line;
        for (const auto& q : p.polyline)
          line.append(q);
        routes.leakPaths.append(line);
      }
      for (const auto& p : silentProbe.walkPaths) {
        QVector<std::array<double, 3>> line;
        for (const auto& q : p.polyline)
          line.append(q);
        routes.walkPaths.append(line);
      }
      routes.gridRes = silentProbe.gridRes;
      routes.gridOrigin = silentProbe.gridOrigin;
      routes.cellSize = silentProbe.cellSize;
      m_doc.setLeakReport(routes);
      if (routes.hasStoredRoutes() || routes.hasStoredGrid())
        log(QStringLiteral("Stored %1 route(s) on this stage; tick 'Show leak "
                           "paths' / 'Show voxel grid' to display them.")
                .arg(routes.leakPaths.size() + routes.walkPaths.size()));
    }
  }
  // Exit check: a successful patch call either appended a stage or was a
  // reported no-op (handled above).
  if (r.loopsFilled > 0) {
    const auto& node = m_doc.node(m_doc.activeIndex());
    if (node.label != QStringLiteral("PatchHoles")) {
      error = QStringLiteral("Patch holes did not append the result stage.");
      return false;
    }
  }
  return true;
}

void MainWindow::runGmshRemesh() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("Gmsh remesh"),
                   {{"Target element size", 0.1}}, {}, d, iv))
    return;
  QString error;
  if (!applyGmshRemesh(d["Target element size"], error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyGmshRemesh(double targetElementSize, QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  Mesh out;
  if (!remeshMesh(m_doc.activeMesh(), targetElementSize, out, error))
    return false;
  const MeshStats remeshed = computeStats(out);
  m_doc.appendOperation(
      QStringLiteral("GmshRemesh"),
      QStringLiteral("size=%1").arg(targetElementSize), std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Gmsh remesh done: %1 faces.").arg(remeshed.faces));
  return true;
}

bool MainWindow::remeshMesh(const Mesh& in, double size, Mesh& out,
                            QString& error) {
  const QString tmpIn = QDir::temp().filePath(QStringLiteral("meshbench_in.stl"));
  const QString tmpOut = QDir::temp().filePath(QStringLiteral("meshbench_out.stl"));
  if (!saveMesh(tmpIn, in, error))
    return false;
  if (!remeshStlWithGmsh(tmpIn, tmpOut, size, error))
    return false;
  return loadMesh(tmpOut, out, error);
}

void MainWindow::runCreatePrimitive() {
  QDialog dlg(this);
  dlg.setWindowTitle(QStringLiteral("Create primitive shape (Gmsh)"));
  auto* layout = new QVBoxLayout(&dlg);
  auto* form = new QFormLayout();
  layout->addLayout(form);
  auto* shapeCombo = new QComboBox(&dlg);
  const QList<PrimitiveShape> shapes = {
      PrimitiveShape::Box, PrimitiveShape::Sphere, PrimitiveShape::Ellipsoid,
      PrimitiveShape::Cylinder, PrimitiveShape::Cone, PrimitiveShape::Torus};
  for (auto s : shapes)
    shapeCombo->addItem(primitiveShapeName(s));
  form->addRow(QStringLiteral("Shape"), shapeCombo);
  // Dimension editors rebuilt whenever the shape changes.
  auto* dimBox = new QWidget(&dlg);
  auto* dimForm = new QFormLayout(dimBox);
  form->addRow(dimBox);
  QList<QDoubleSpinBox*> dimEdits;
  auto rebuildDims = [&] {
    // takeAt() transfers ownership of the item to the caller; the guard
    // frees it even on early exit, and the widget is owned by dimBox.
    while (QLayoutItem* raw = dimForm->takeAt(0)) {
      std::unique_ptr<QLayoutItem> item(raw);
      delete item->widget();
    }
    dimEdits.clear();
    const auto shape = shapes[shapeCombo->currentIndex()];
    const double defaults[3] = {1.0, 1.0, 1.0};
    int i = 0;
    for (const auto& name : primitiveDimNames(shape)) {
      auto* spin = new QDoubleSpinBox(dimBox);
      spin->setRange(1e-6, 1e6);
      spin->setDecimals(4);
      spin->setValue(defaults[i++ % 3]);
      dimForm->addRow(name, spin);
      dimEdits.append(spin);
    }
  };
  rebuildDims();
  connect(shapeCombo, &QComboBox::currentIndexChanged, &dlg,
          [&](int) { rebuildDims(); });
  auto* sizeSpin = new QDoubleSpinBox(&dlg);
  sizeSpin->setRange(1e-4, 1e3);
  sizeSpin->setDecimals(4);
  sizeSpin->setValue(0.1);
  form->addRow(QStringLiteral("Mesh size"), sizeSpin);
  auto* buttons =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  if (dlg.exec() != QDialog::Accepted)
    return;

  PrimitiveParams params;
  params.shape = shapes[shapeCombo->currentIndex()];
  params.dims.clear();
  for (auto* spin : dimEdits)
    params.dims.append(spin->value());
  params.meshSize = sizeSpin->value();

  QString error;
  if (!applyCreatePrimitive(params, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyCreatePrimitive(const PrimitiveParams& params,
                                      QString& error) {
  error.clear();
  if (params.dims.isEmpty()) {
    error = QStringLiteral("No primitive dimensions given.");
    return false;
  }
  if (!(params.meshSize > 0.0) || !std::isfinite(params.meshSize)) {
    error = QStringLiteral("Primitive mesh size must be positive.");
    return false;
  }
  const QString tmp =
      QDir::temp().filePath(QStringLiteral("meshbench_primitive.stl"));
  if (!createPrimitiveStl(params, tmp, error))
    return false;
  Mesh mesh;
  if (!loadMesh(tmp, mesh, error))
    return false;
  if (mesh.num_vertices() == 0) {
    error = QStringLiteral("Primitive generation produced an empty mesh.");
    return false;
  }
  QStringList dimStr;
  for (double v : params.dims)
    dimStr.append(QString::number(v));
  const MeshStats s = computeStats(mesh);
  m_doc.appendSource(QStringLiteral("%1(%2)")
                         .arg(primitiveShapeName(params.shape),
                              dimStr.join(QStringLiteral(","))),
                     std::move(mesh));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  probesToTable({});
  QString probeError;
  fillDefaultProbes(probeError);
  log(QStringLiteral("Created %1 primitive: %2 faces, closed=%3.")
          .arg(primitiveShapeName(params.shape))
          .arg(s.faces)
          .arg(s.closed ? QStringLiteral("yes") : QStringLiteral("no")));
  return true;
}

void MainWindow::runIntroduceHoles() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("Introduce random holes"),
                   {}, {{"Triangles to remove", 10}, {"Seed", 42}}, d, iv))
    return;
  QString error;
  if (!applyIntroduceHoles(iv["Triangles to remove"],
                           static_cast<unsigned>(iv["Seed"]), error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyIntroduceHoles(int triangles, unsigned seed,
                                     QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  if (triangles <= 0) {
    error = QStringLiteral("Triangles to remove must be positive.");
    return false;
  }
  Mesh out = m_doc.activeMesh();
  const int removed = removeRandomTriangles(out, triangles, seed, error);
  if (removed == 0) {
    if (error.isEmpty())
      error = QStringLiteral("Nothing removed.");
    return false;
  }
  const MeshStats s = computeStats(out);
  m_doc.appendOperation(QStringLiteral("Puncture"),
                        QStringLiteral("removed %1 tris (seed %2)")
                            .arg(removed)
                            .arg(seed),
                        std::move(out));
  m_visible.insert(m_doc.activeIndex());
  refreshView();
  log(QStringLiteral("Removed %1 triangles; now %2 hole loop(s), %3 boundary "
                     "edges. Run Detect leaks to see them.")
          .arg(removed)
          .arg(findHoleLoops(m_doc.activeMesh()).size())
          .arg(s.boundaryEdges));
  return true;
}

void MainWindow::runShrinkwrap() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("Shrinkwrap from enclosing sphere"),
                   {{"Step (0-1)", m_shrinkParams.step},
                    {"Smoothing (0-0.9)", m_shrinkParams.smooth},
                    {"Even spacing (0-1)", m_shrinkParams.uniformity}},
                   {{"Sphere subdivisions (0-6)", m_shrinkParams.subdivisions},
                    {"Iterations", m_shrinkParams.iterations}},
                   d, iv))
    return;
  m_shrinkParams.step = d["Step (0-1)"];
  m_shrinkParams.smooth = d["Smoothing (0-0.9)"];
  m_shrinkParams.uniformity = d["Even spacing (0-1)"];
  m_shrinkParams.subdivisions = iv["Sphere subdivisions (0-6)"];
  m_shrinkParams.iterations = iv["Iterations"];

  QString error;
  if (!applyShrinkwrap(m_shrinkParams, error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::applyShrinkwrap(const ShrinkParams& params, QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  if (params.subdivisions < 0 || params.subdivisions > 6) {
    error = QStringLiteral("Sphere subdivisions must be between 0 and 6.");
    return false;
  }
  if (params.iterations <= 0) {
    error = QStringLiteral("Shrinkwrap iterations must be positive.");
    return false;
  }
  if (!(params.step > 0.0) || params.step > 1.0) {
    error = QStringLiteral("Shrinkwrap step must be in (0, 1].");
    return false;
  }
  if (params.smooth < 0.0 || params.smooth > 0.9) {
    error = QStringLiteral("Shrinkwrap smoothing must be in [0, 0.9].");
    return false;
  }
  m_shrinkParams = params;

  const int source = m_doc.activeIndex();
  Mesh sphere = makeEnclosingSphere(m_doc.activeMesh(), m_shrinkParams.subdivisions);
  if (!snapShrinkwrap(m_doc.activeMesh(), {}, sphere, m_shrinkParams, error))
    return false;
  m_wrapSource = source;
  m_faceWeights.clear();
  m_faceSelection.clear();
  m_doc.appendOperation(
      QStringLiteral("Shrinkwrap"),
      QStringLiteral("subdiv=%1 iters=%2 step=%3 smooth=%4 even=%5")
          .arg(m_shrinkParams.subdivisions)
          .arg(m_shrinkParams.iterations)
          .arg(m_shrinkParams.step)
          .arg(m_shrinkParams.smooth)
          .arg(m_shrinkParams.uniformity),
      std::move(sphere));
  m_wrapStage = m_doc.activeIndex();
  // Keep the baseline visible: the wrap renders translucent over it.
  m_visible.insert(m_wrapSource);
  m_visible.insert(m_wrapStage);
  refreshView();
  updateShrinkPanel();
  log(QStringLiteral("Shrinkwrap stage %1 from stage %2 (%3 faces). Enable "
                     "'Select panels', click target triangles, set a weight, "
                     "Apply.")
          .arg(m_wrapStage)
          .arg(m_wrapSource)
          .arg(computeStats(m_doc.node(m_wrapStage).mesh).faces));
  return true;
}

void MainWindow::rerunShrinkwrap() {
  if (m_wrapSource < 0 || m_wrapStage < 0 ||
      m_wrapSource >= m_doc.nodeCount() || m_wrapStage >= m_doc.nodeCount()) {
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Create a shrinkwrap first."));
    return;
  }
  std::unordered_map<std::size_t, double> weights;
  for (auto it = m_faceWeights.begin(); it != m_faceWeights.end(); ++it)
    weights[static_cast<std::size_t>(it.key())] = it.value();
  // Always restart from a fresh enclosing sphere so weight edits reshape
  // the result instead of accumulating onto the old one.
  Mesh sphere =
      makeEnclosingSphere(m_doc.node(m_wrapSource).mesh, m_shrinkParams.subdivisions);
  QString error;
  if (!snapShrinkwrap(m_doc.node(m_wrapSource).mesh, weights, sphere,
                      m_shrinkParams, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  const MeshStats s = computeStats(sphere);
  m_doc.replaceMesh(m_wrapStage, std::move(sphere));
  updateShrinkPanel();
  log(QStringLiteral("Shrinkwrap re-snapped with %1 weighted panels: %2 faces.")
          .arg(m_faceWeights.size())
          .arg(s.faces));
}

void MainWindow::togglePickMode(bool on) {
  if (on && (m_wrapSource < 0 || m_wrapSource >= m_doc.nodeCount())) {
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Create a shrinkwrap first."));
    m_pickBtn->setChecked(false);
    return;
  }
  if (on) {
    // Picking targets the baseline actor, so it must be drawn.
    m_visible.insert(m_wrapSource);
    refreshView();
  }
  m_view->setPickMode(on, m_wrapSource);
  updateShrinkPanel();
}

void MainWindow::toggleSelectMode(bool on) {
  m_selectMode = on;
  if (!on) {
    m_view->setSelectMode(false);
    // Normal exits commit the pending deletions as a new stage. Internal
    // shutdowns (tree rebuild, stage deletion) discard them instead.
    if (!m_suppressSelectionCommit)
      commitSelectionDeletions();
    else {
      m_selection.clear();
      m_selectedEdges.clear();
      m_view->setSelection(m_selectionStage, {});
      m_view->setEdgeSelection(m_selectionStage, {});
      m_selectionStage = -1;
    }
    updateSelectPanel();
    return;
  }
  const bool hasSurface =
      m_doc.hasMesh() && m_doc.activeMesh().num_vertices() > 0;
  const bool hasEdges =
      m_doc.hasMesh() && m_doc.activeIndex() >= 0 &&
      !m_doc.node(m_doc.activeIndex()).featureEdges.empty();
  if (!hasSurface && !hasEdges) {
    // Imported eMesh-only stages are still selectable: the user can delete
    // their feature edges even though there is no surface to pick.
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Open or activate a stage with a "
                                            "surface mesh or feature edges "
                                            "first."));
    m_selectBtn->setChecked(false);
    m_selectMode = false;
    return;
  }
  // Shrinkwrap picking and rectangle selection both swallow the mouse.
  if (m_pickBtn->isChecked())
    m_pickBtn->setChecked(false);
  if (m_selectionStage != m_doc.activeIndex()) {
    // Faces/edges belong to one stage; switching stages restarts the set.
    m_selectionStage = m_doc.activeIndex();
    m_selection.clear();
    m_selectedEdges.clear();
  }
  m_visible.insert(m_selectionStage);
  refreshView();
  m_view->setSelectMode(true, m_selectionStage);
  m_view->setSelection(m_selectionStage, m_selection);
  m_view->setEdgeSelection(m_selectionStage, m_selectedEdges);
  log(QStringLiteral("Select mode on stage %1: click toggles a triangle, "
                     "click an amber feature edge to toggle that edge, drag "
                     "selects a rectangle. Exit S to apply deletions as a "
                     "new stage.")
          .arg(m_selectionStage));
  updateSelectPanel();
}

void MainWindow::onFacesSelected(const QSet<int>& faces) {
  if (!m_selectMode)
    return;
  for (int f : faces)
    m_selection.insert(f);
  m_view->setSelection(m_selectionStage, m_selection);
  log(QStringLiteral("Rectangle added %1 triangles (%2 selected).")
          .arg(faces.size())
          .arg(m_selection.size()));
  updateSelectPanel();
}

void MainWindow::onFeatureEdgesSelected(const QSet<int>& edges) {
  if (!m_selectMode)
    return;
  for (int e : edges)
    m_selectedEdges.insert(e);
  m_view->setEdgeSelection(m_selectionStage, m_selectedEdges);
  log(QStringLiteral("Rectangle added %1 feature edges (%2 selected).")
          .arg(edges.size())
          .arg(m_selectedEdges.size()));
  updateSelectPanel();
}

void MainWindow::runExtractSelection() {
  if (m_selection.isEmpty() || m_selectionStage < 0 ||
      m_selectionStage >= m_doc.nodeCount()) {
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Select triangles first (S in "
                                            "the viewport, then click/drag)."));
    return;
  }
  Mesh sub = extractSubmesh(m_doc.node(m_selectionStage).mesh, m_selection);
  const std::size_t faces = sub.num_faces();
  if (faces == 0) {
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Selection holds no faces."));
    return;
  }
  const int source = m_selectionStage;
  m_doc.appendOperation(
      QStringLiteral("Selection"),
      QStringLiteral("%1 faces from stage %2").arg(faces).arg(source),
      std::move(sub));
  m_visible.insert(m_doc.activeIndex());
  // The filter consumed the selection: leaving selection mode must not also
  // delete those faces from the source stage.
  clearSelection();
  refreshView();
  log(QStringLiteral("Extracted %1 selected faces as a new stage.").arg(faces));
}

void MainWindow::runRemeshSelection() {
  if (m_selection.isEmpty() || m_selectionStage < 0 ||
      m_selectionStage >= m_doc.nodeCount()) {
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Select triangles first (S in "
                                            "the viewport, then click/drag)."));
    return;
  }
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("Gmsh remesh (selection)"),
                   {{"Target element size", 0.1}}, {}, d, iv))
    return;
  Mesh sub = extractSubmesh(m_doc.node(m_selectionStage).mesh, m_selection);
  QString error;
  Mesh out;
  if (!remeshMesh(sub, d["Target element size"], out, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  const MeshStats remeshed = computeStats(out);
  m_doc.appendOperation(
      QStringLiteral("GmshRemesh (selection)"),
      QStringLiteral("size=%1, %2 faces in").arg(d["Target element size"]).arg(sub.num_faces()),
      std::move(out));
  m_visible.insert(m_doc.activeIndex());
  clearSelection();
  refreshView();
  log(QStringLiteral("Gmsh remesh of selection done: %1 faces.")
          .arg(remeshed.faces));
}

void MainWindow::runDetectLeaksSelection() {
  if (m_selection.isEmpty() || m_selectionStage < 0 ||
      m_selectionStage >= m_doc.nodeCount()) {
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Select triangles first (S in "
                                            "the viewport, then click/drag)."));
    return;
  }
  const Mesh sub =
      extractSubmesh(m_doc.node(m_selectionStage).mesh, m_selection);
  if (sub.num_faces() == 0) {
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Selection holds no faces."));
    return;
  }
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("Leak detection grid"), {},
                   {{"Voxels per axis", ProbeParams{}.gridRes}}, d, iv))
    return;
  QString error;
  if (!probeMeshAndAppend(sub, QStringLiteral("LeakProbe (selection)"),
                          iv["Voxels per axis"], error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  // The probe consumed the selection: do not also delete it on exit.
  clearSelection();
}

void MainWindow::clearSelection() {
  m_selection.clear();
  m_selectedEdges.clear();
  m_view->setSelection(m_selectionStage, m_selection);
  m_view->setEdgeSelection(m_selectionStage, m_selectedEdges);
  updateSelectPanel();
}

void MainWindow::cancelSelectionSession() {
  m_suppressSelectionCommit = true;
  if (m_selectBtn && m_selectBtn->isChecked()) {
    m_selectBtn->setChecked(false);
  } else {
    m_selectMode = false;
    m_view->setSelectMode(false);
    m_selection.clear();
    m_selectedEdges.clear();
    m_view->setSelection(m_selectionStage, {});
    m_view->setEdgeSelection(m_selectionStage, {});
    m_selectionStage = -1;
    updateSelectPanel();
  }
  m_suppressSelectionCommit = false;
}

bool MainWindow::commitSelectionDeletions() {
  const int source = m_selectionStage;
  const bool hasSelection =
      !m_selection.isEmpty() || !m_selectedEdges.isEmpty();
  const auto clearSession = [this] {
    m_selection.clear();
    m_selectedEdges.clear();
    if (m_view) {
      m_view->setSelection(m_selectionStage, {});
      m_view->setEdgeSelection(m_selectionStage, {});
    }
    m_selectionStage = -1;
  };
  if (!hasSelection) {
    clearSession();
    return true;
  }
  if (source < 0 || source >= m_doc.nodeCount()) {
    clearSession();
    return false;
  }

  // Never mutate the active node: copy its data and append the deletions as
  // a new pipeline stage.
  const PipelineNode& src = m_doc.node(source);
  Mesh mesh = src.mesh;
  FeatureEdges edges = src.featureEdges;
  int removedFaces = 0;
  int removedEdges = 0;
  QString error;
  if (!m_selection.isEmpty()) {
    Mesh withoutFaces;
    if (!removeMeshFaces(src.mesh, m_selection, withoutFaces, removedFaces,
                         error)) {
      log(QStringLiteral("Delete selection failed: %1").arg(error));
      clearSession();
      return false;
    }
    mesh = std::move(withoutFaces);
  }
  if (!m_selectedEdges.isEmpty()) {
    FeatureEdges withoutEdges;
    if (!removeFeatureEdges(src.featureEdges, m_selectedEdges, withoutEdges,
                            removedEdges, error)) {
      log(QStringLiteral("Delete feature edges failed: %1").arg(error));
      clearSession();
      return false;
    }
    edges = std::move(withoutEdges);
  }
  if (removedFaces == 0 && removedEdges == 0) {
    clearSession();
    return true;
  }

  QStringList parts;
  if (removedFaces > 0)
    parts << QStringLiteral("%1 faces").arg(removedFaces);
  if (removedEdges > 0)
    parts << QStringLiteral("%1 feature edges").arg(removedEdges);
  if (m_doc.activeIndex() != source)
    m_doc.setActive(source);
  // appendOperation inserts directly after the active stage, so update
  // visibility before it emits historyChanged (which rebuilds the tree).
  int newStage = source + 1;
  m_visible.remove(source);
  m_visible.insert(newStage);
  m_doc.appendOperation(
      QStringLiteral("DeleteSelection"),
      QStringLiteral("removed %1 from stage %2")
          .arg(parts.join(QStringLiteral(", ")))
          .arg(source),
      std::move(mesh), std::move(edges));
  if (m_doc.activeIndex() != newStage) {
    m_visible.remove(newStage);
    newStage = m_doc.activeIndex();
    m_visible.insert(newStage);
  }
  clearSession();
  updateSelectPanel();
  refreshView();
  log(QStringLiteral("Deleted %1 from stage %2 as new stage %3.")
          .arg(parts.join(QStringLiteral(", ")))
          .arg(source)
          .arg(newStage));
  return true;
}

void MainWindow::updateSelectPanel() {
  if (!m_selectLabel)
    return;
  QStringList parts;
  if (m_selectionStage >= 0)
    parts << QStringLiteral("Stage %1.").arg(m_selectionStage);
  parts << QStringLiteral("%1 triangles, %2 feature edges selected.")
               .arg(m_selection.size())
               .arg(m_selectedEdges.size());
  parts << QStringLiteral("Select mode %1.").arg(
      m_selectBtn && m_selectBtn->isChecked() ? QStringLiteral("ON")
                                             : QStringLiteral("off"));
  parts << QStringLiteral("Exit selection mode to apply deletions as a new "
                          "stage.");
  m_selectLabel->setText(parts.join(QStringLiteral(" ")));
}

void MainWindow::validateSelection() {
  if (m_selectionStage < 0 || m_selectionStage >= m_doc.nodeCount()) {
    // Selected stage was reverted away: drop the session without deleting.
    if (m_selectMode || (m_selectBtn && m_selectBtn->isChecked()))
      cancelSelectionSession();
    m_selectionStage = -1;
    m_selection.clear();
    m_selectedEdges.clear();
  }
  updateSelectPanel();
}

void MainWindow::onFacePicked(int faceIdx) {
  if (faceIdx < 0)
    return;
  if (m_selectMode) {
    // Rectangle-selection session: clicks toggle single triangles.
    if (m_selection.contains(faceIdx))
      m_selection.remove(faceIdx);
    else
      m_selection.insert(faceIdx);
    m_view->setSelection(m_selectionStage, m_selection);
    updateSelectPanel();
    return;
  }
  // Click toggles membership so stray clicks are easy to undo.
  if (m_faceSelection.contains(faceIdx))
    m_faceSelection.remove(faceIdx);
  else
    m_faceSelection.insert(faceIdx);
  // Show the clicked panel's current weight in the editors.
  m_weightSpin->setValue(m_faceWeights.value(faceIdx, 1.0));
  updateShrinkPanel();
}

void MainWindow::onFeatureEdgePicked(int edgeIdx) {
  if (!m_selectMode || edgeIdx < 0)
    return;
  if (m_selectionStage < 0 || m_selectionStage >= m_doc.nodeCount())
    return;
  if (static_cast<std::size_t>(edgeIdx) >=
      m_doc.node(m_selectionStage).featureEdges.edges.size())
    return;
  if (m_selectedEdges.contains(edgeIdx))
    m_selectedEdges.remove(edgeIdx);
  else
    m_selectedEdges.insert(edgeIdx);
  m_view->setEdgeSelection(m_selectionStage, m_selectedEdges);
  log(QStringLiteral("Toggled feature edge %1 (%2 selected).")
          .arg(edgeIdx)
          .arg(m_selectedEdges.size()));
  updateSelectPanel();
}

void MainWindow::applyWeightToSelection() {
  if (m_faceSelection.isEmpty()) {
    QMessageBox::information(this, QStringLiteral("MeshBench"),
                             QStringLiteral("Select panels in the viewport first "
                                            "(enable 'Select panels')."));
    return;
  }
  const double w = m_weightSpin->value();
  for (int f : m_faceSelection)
    m_faceWeights[f] = w;
  log(QStringLiteral("Weight %1 applied to %2 panels; re-snapping.")
          .arg(w)
          .arg(m_faceSelection.size()));
  rerunShrinkwrap();
}

void MainWindow::clearFaceSelection() {
  m_faceSelection.clear();
  updateShrinkPanel();
}

void MainWindow::resetFaceWeights() {
  m_faceWeights.clear();
  m_faceSelection.clear();
  updateShrinkPanel();
  rerunShrinkwrap();
}

void MainWindow::updateShrinkPanel() {
  if (m_wrapStage < 0) {
    m_selLabel->setText(QStringLiteral("No wrap yet."));
    return;
  }
  QStringList parts;
  parts << QStringLiteral("Wrap stage %1 from baseline stage %2.")
               .arg(m_wrapStage)
               .arg(m_wrapSource);
  parts << (m_faceSelection.isEmpty()
                ? QStringLiteral("No panels selected.")
                : QStringLiteral("%1 panels selected.").arg(m_faceSelection.size()));
  parts << QStringLiteral("%1 weighted panels.").arg(m_faceWeights.size());
  if (m_view)
    parts << QStringLiteral("Pick mode %1.").arg(
        m_pickBtn->isChecked() ? QStringLiteral("ON") : QStringLiteral("off"));
  m_selLabel->setText(parts.join(QStringLiteral(" ")));
}

void MainWindow::validateShrinkState() {
  const auto valid = [&](int i) {
    return i >= 0 && i < m_doc.nodeCount() &&
           m_doc.node(i).label.startsWith(QStringLiteral("Shrinkwrap"));
  };
  if (m_wrapStage >= 0 && !valid(m_wrapStage)) {
    // Wrap stage was reverted away: drop the whole shrink session.
    if (m_pickBtn->isChecked())
      m_pickBtn->setChecked(false);
    else
      m_view->setPickMode(false);
    m_wrapStage = -1;
    m_wrapSource = -1;
    m_faceWeights.clear();
    m_faceSelection.clear();
  }
  if (m_wrapSource < 0 || m_wrapSource >= m_doc.nodeCount())
    m_wrapSource = (m_wrapStage >= 0) ? m_wrapSource : -1;
  updateShrinkPanel();
}

void MainWindow::setDefaultProbes() {
  if (!requireMesh())
    return;
  QString error;
  if (!fillDefaultProbes(error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
}

bool MainWindow::fillDefaultProbes(QString& error) {
  error.clear();
  if (!ensureActiveMesh(error))
    return false;
  const MeshStats s = computeStats(m_doc.activeMesh());
  const double cx = (s.bboxMin[0] + s.bboxMax[0]) / 2;
  const double cy = (s.bboxMin[1] + s.bboxMax[1]) / 2;
  const double cz = (s.bboxMin[2] + s.bboxMax[2]) / 2;
  const double dx = s.bboxMax[0] - s.bboxMin[0];
  const double dy = s.bboxMax[1] - s.bboxMin[1];
  const double dz = s.bboxMax[2] - s.bboxMin[2];
  const double pad = std::max({dx, dy, dz}) * 0.2 + 1e-6;
  probesToTable({{cx, cy, cz, true},
                 {s.bboxMax[0] + pad, s.bboxMax[1] + pad, s.bboxMax[2] + pad, false}});
  log(QStringLiteral("Default probes: inside = bbox center, outside = beyond bbox."));
  // Exit check: the editor must now hold an inside and an outside probe.
  if (probesFromTable().size() != 2) {
    error = QStringLiteral("Failed to install default probes.");
    return false;
  }
  return true;
}

void MainWindow::addProbeRow(bool inside) {
  const int row = m_probeTable->rowCount();
  m_probeTable->insertRow(row);
  for (int c = 0; c < 3; ++c) {
    auto* item = new QTableWidgetItem(QStringLiteral("0"));
    m_probeTable->setItem(row, c, item);
  }
  m_probeTable->setItem(row, 3, new QTableWidgetItem(
                                    inside ? QStringLiteral("inside")
                                           : QStringLiteral("outside")));
}

void MainWindow::removeSelectedProbes() {
  for (const auto& range : m_probeTable->selectedRanges()) {
    for (int r = range.bottomRow(); r >= range.topRow(); --r)
      m_probeTable->removeRow(r);
  }
}

QVector<ProbePoint> MainWindow::probesFromTable() const {
  QVector<ProbePoint> probes;
  for (int r = 0; r < m_probeTable->rowCount(); ++r) {
    ProbePoint p;
    bool ok = true;
    p.x = m_probeTable->item(r, 0) ? m_probeTable->item(r, 0)->text().toDouble(&ok)
                                   : 0.0;
    if (!ok)
      continue;
    p.y = m_probeTable->item(r, 1) ? m_probeTable->item(r, 1)->text().toDouble(&ok)
                                   : 0.0;
    if (!ok)
      continue;
    p.z = m_probeTable->item(r, 2) ? m_probeTable->item(r, 2)->text().toDouble(&ok)
                                   : 0.0;
    if (!ok)
      continue;
    const QString side =
        m_probeTable->item(r, 3) ? m_probeTable->item(r, 3)->text() : QString();
    p.inside = (side.trimmed().toLower() != QStringLiteral("outside"));
    probes.append(p);
  }
  return probes;
}

void MainWindow::probesToTable(const QVector<ProbePoint>& probes) {
  m_probeTable->setRowCount(0);
  for (const auto& p : probes) {
    const int row = m_probeTable->rowCount();
    m_probeTable->insertRow(row);
    m_probeTable->setItem(row, 0, new QTableWidgetItem(QString::number(p.x)));
    m_probeTable->setItem(row, 1, new QTableWidgetItem(QString::number(p.y)));
    m_probeTable->setItem(row, 2, new QTableWidgetItem(QString::number(p.z)));
    m_probeTable->setItem(row, 3, new QTableWidgetItem(p.inside
                                                          ? QStringLiteral("inside")
                                                          : QStringLiteral("outside")));
  }
}

void MainWindow::onTreeItemChanged(QTreeWidgetItem* item, int column) {
  if (m_updatingTree || !item)
    return;
  const int idx = item->data(0, Qt::UserRole).toInt();
  if (column == 2) {
    // Dedicated Combine checkbox: independent of visibility (column 0)
    // and of Qt's transient tree selection.
    setStageSelection(toggleStageSelection(
        m_stageSelection, idx, item->checkState(2) == Qt::Checked));
    return;
  }
  if (column != 0)
    return;
  if (item->checkState(0) == Qt::Checked)
    m_visible.insert(idx);
  else
    m_visible.remove(idx);
  refreshView();
}

void MainWindow::onTreeItemActivated(QTreeWidgetItem* item, int /*column*/) {
  if (!item)
    return;
  // Switching the active stage discards a viewport selection instead of
  // risking a delete against the wrong stage.
  cancelSelectionSession();
  m_doc.setActive(item->data(0, Qt::UserRole).toInt());
}

void MainWindow::revertToSelectedStage() {
  auto* item = m_tree->currentItem();
  if (!item || !requireMesh())
    return;
  const int idx = item->data(0, Qt::UserRole).toInt();
  cancelSelectionSession();
  m_doc.truncateAfter(idx);
  m_visible.clear();
  m_visible.insert(m_doc.activeIndex());
  refreshView();
}

void MainWindow::refreshAll() {
  validateGmshCadState();
  pruneStageSelection();
  validateShrinkState();
  validateSelection();
  rebuildTree();
  updateStagePanel();
  refreshView();
}

void MainWindow::rebuildTree() {
  m_updatingTree = true;
  // Rebuilding drops Qt selection: remember the selected stages first.
  QSet<int> kept;
  for (const auto* sel : m_tree->selectedItems())
    kept.insert(sel->data(0, Qt::UserRole).toInt());
  kept = mergeStageSelections(kept, m_stageSelection);
  m_tree->clear();
  for (int i = 0; i < m_doc.nodeCount(); ++i) {
    const auto& node = m_doc.node(i);
    auto* item = new QTreeWidgetItem(m_tree);
    QString label = QStringLiteral("%1: %2").arg(i).arg(node.label);
    if (!node.details.isEmpty())
      label += QStringLiteral(" (%1)").arg(node.details);
    item->setText(0, label);
    item->setData(0, Qt::UserRole, i);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(0, m_visible.contains(i) ? Qt::Checked : Qt::Unchecked);
    item->setCheckState(2, m_stageSelection.contains(i) ? Qt::Checked
                                                       : Qt::Unchecked);
    item->setToolTip(2, QStringLiteral("Include this stage in Combine"));
    const MeshStats s = computeStats(node.mesh);
    if (!node.featureEdges.empty()) {
      item->setText(1,
                    QStringLiteral("%1 edges").arg(node.featureEdges.edges.size()));
    } else {
      item->setText(1, QStringLiteral("%1 f / %2 bnd")
                           .arg(s.faces)
                           .arg(s.boundaryEdges));
    }
    if (kept.contains(i))
      item->setSelected(true);
    if (i == m_doc.activeIndex()) {
      QFont f = item->font(0);
      f.setBold(true);
      item->setFont(0, f);
    }
    if (node.leaks.probed)
      item->setText(1, item->text(1) + (node.leaks.hasLeak ? QStringLiteral(" LEAK")
                                                           : QStringLiteral(" sealed")));
  }
  m_tree->expandAll();
  m_updatingTree = false;
}

void MainWindow::updateStagePanel() {
  if (!m_doc.hasMesh()) {
    m_stageInfo->setText(QStringLiteral("No mesh loaded."));
    m_leakSummary->setText({});
    m_opacitySpin->blockSignals(true);
    m_opacitySpin->setValue(1.0);
    m_opacitySpin->setEnabled(false);
    m_opacitySpin->blockSignals(false);
    return;
  }
  const auto& node = m_doc.node(m_doc.activeIndex());
  // Show the active stage's effective opacity without pinning it: a stage
  // with no pinned value displays the automatic one (opaque when active).
  m_opacitySpin->blockSignals(true);
  m_opacitySpin->setValue(node.opacity >= 0.0 ? node.opacity : 1.0);
  m_opacitySpin->setEnabled(true);
  m_opacitySpin->blockSignals(false);
  const MeshStats s = computeStats(node.mesh);
  m_stageInfo->setText(
      QStringLiteral("Stage %1: %2\n%3\n verts=%4 faces=%5 bnd=%6 closed=%7\n"
                     "area=%8 vol=%9\nbbox [%10,%11,%12] - [%13,%14,%15]")
          .arg(m_doc.activeIndex())
          .arg(node.label)
          .arg(node.details.isEmpty() ? QStringLiteral("(source)") : node.details)
          .arg(s.vertices)
          .arg(s.faces)
          .arg(s.boundaryEdges)
          .arg(s.closed ? QStringLiteral("yes") : QStringLiteral("no"))
          .arg(s.area)
          .arg(s.volume)
          .arg(s.bboxMin[0])
          .arg(s.bboxMin[1])
          .arg(s.bboxMin[2])
          .arg(s.bboxMax[0])
          .arg(s.bboxMax[1])
          .arg(s.bboxMax[2]));
  if (!node.featureEdges.empty()) {
    m_stageInfo->setText(
        m_stageInfo->text() +
        QStringLiteral("\nfeature edges=%1 points=%2")
            .arg(node.featureEdges.edges.size())
            .arg(node.featureEdges.points.size()));
  }
  if (node.leaks.probed) {
    m_leakSummary->setText(
        node.leaks.hasLeak
            ? QStringLiteral("LEAK at this stage: %1 hole loop(s), %2 path(s).")
                  .arg(node.leaks.holeLoops)
                  .arg(node.leaks.leakPaths.size())
            : QStringLiteral("Sealed: %1 hole loop(s), no inside->outside path.")
                  .arg(node.leaks.holeLoops));
  } else {
    m_leakSummary->setText(QStringLiteral("Not probed yet."));
  }
}

void MainWindow::refreshView() {
  if (!m_doc.hasMesh())
    return;
  m_view->showDocument(m_doc, m_visible, m_doc.activeIndex());
}

} // namespace meshbench
