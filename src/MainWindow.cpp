#include "MainWindow.h"
#include "LeakFix.h"
#include "MeshOps.h"

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
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace meshbench {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle(QStringLiteral("MeshBench"));
  resize(1280, 800);

  auto* fileMenu = menuBar()->addMenu(QStringLiteral("&File"));
  fileMenu->addAction(QStringLiteral("Open mesh..."), QKeySequence::Open, this,
                      &MainWindow::openMesh);
  fileMenu->addAction(QStringLiteral("Save active stage..."), QKeySequence::Save,
                      this, &MainWindow::saveActiveMesh);
  fileMenu->addSeparator();
  fileMenu->addAction(QStringLiteral("Clear"), this, &MainWindow::clearAll);
  fileMenu->addAction(QStringLiteral("Quit"), QKeySequence::Quit, this,
                      &QWidget::close);

  auto* opsMenu = menuBar()->addMenu(QStringLiteral("&Operations"));
  opsMenu->addAction(QStringLiteral("Boolean with another mesh..."), this,
                     &MainWindow::runBoolean);
  opsMenu->addAction(QStringLiteral("Center at volume center"), this,
                     &MainWindow::runCenter);
  opsMenu->addAction(QStringLiteral("Alpha wrap..."), this, &MainWindow::runAlphaWrap);
  opsMenu->addAction(QStringLiteral("Slice with plane..."), this, &MainWindow::runClip);
  opsMenu->addSeparator();
  opsMenu->addAction(QStringLiteral("Detect leaks"), this, &MainWindow::runDetectLeaks);
  opsMenu->addAction(QStringLiteral("Patch holes..."), this, &MainWindow::runPatchHoles);
  opsMenu->addAction(QStringLiteral("Introduce random holes..."), this,
                     &MainWindow::runIntroduceHoles);
  opsMenu->addAction(QStringLiteral("Shrinkwrap..."), this,
                     &MainWindow::runShrinkwrap);
#ifdef MESHBENCH_HAVE_GMSH
  opsMenu->addSeparator();
  opsMenu->addAction(QStringLiteral("Create primitive shape..."), this,
                     &MainWindow::runCreatePrimitive);
  opsMenu->addAction(QStringLiteral("Remesh with Gmsh..."), this,
                     &MainWindow::runGmshRemesh);
#endif
  auto* viewMenu = menuBar()->addMenu(QStringLiteral("&View"));
  viewMenu->addAction(QStringLiteral("Reset camera"), this,
                      [this] { m_view->resetCamera(); });

  auto* splitter = new QSplitter(this);
  setCentralWidget(splitter);

  // --- Left: pipeline history sidebar ---
  m_tree = new QTreeWidget(splitter);
  m_tree->setHeaderLabels({QStringLiteral("Pipeline"), QStringLiteral("Info")});
  m_tree->setColumnWidth(0, 220);
  m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(m_tree, &QTreeWidget::itemChanged, this, &MainWindow::onTreeItemChanged);
  connect(m_tree, &QTreeWidget::itemDoubleClicked, this,
          &MainWindow::onTreeItemActivated);
  connect(m_tree, &QWidget::customContextMenuRequested, this, [this](const QPoint& p) {
    if (auto* item = m_tree->itemAt(p)) {
      QMenu menu;
      menu.addAction(QStringLiteral("Show only this stage"), this,
                     [this, item] {
                       m_doc.setActive(item->data(0, Qt::UserRole).toInt());
                       for (int i = 0; i < m_doc.nodeCount(); ++i)
                         m_visible.remove(i);
                       m_visible.insert(m_doc.activeIndex());
                       refreshAll();
                     });
      menu.addAction(QStringLiteral("Revert to this stage (drop later ops)"),
                     this, &MainWindow::revertToSelectedStage);
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
#ifdef MESHBENCH_HAVE_GMSH
  log(QStringLiteral("Gmsh %1 available.").arg(gmshVersion()));
#else
  log(QStringLiteral("Gmsh support not compiled in; remeshing disabled."));
#endif
}

void MainWindow::log(const QString& msg) {
  m_log->appendPlainText(msg);
}

bool MainWindow::requireMesh() const {
  if (!m_doc.hasMesh()) {
    QMessageBox::information(const_cast<MainWindow*>(this),
                             QStringLiteral("MeshBench"),
                             QStringLiteral("Open a mesh first."));
    return false;
  }
  return true;
}

void MainWindow::openMesh() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Open mesh"), {},
      QStringLiteral("Meshes (*.stl *.off *.obj);;All files (*)"));
  if (path.isEmpty())
    return;
  Mesh mesh;
  QString error;
  if (!loadMesh(path, mesh, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  const MeshStats s = computeStats(mesh);
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
  if (!saveMesh(path, m_doc.activeMesh(), error))
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
  else
    log(QStringLiteral("Saved stage %1 to %2.").arg(m_doc.activeIndex()).arg(path));
}

void MainWindow::clearAll() {
  m_doc.clear();
  m_visible.clear();
  m_view->clear();
  rebuildTree();
  updateStagePanel();
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
  Mesh other;
  QString error;
  if (!loadMesh(path, other, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
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
  Mesh out;
  if (!booleanOp(m_doc.activeMesh(), other, op, out, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  static const char* names[] = {"Union", "Intersection", "Difference"};
  const MeshStats s = computeStats(out);
  m_doc.appendOperation(QString::fromUtf8(names[combo->currentIndex()]),
                        QStringLiteral("with %1").arg(QFileInfo(path).fileName()),
                        std::move(out));
  m_visible.insert(m_doc.activeIndex());
  log(QStringLiteral("Boolean %1: %2 faces, volume %3.")
          .arg(QString::fromUtf8(names[combo->currentIndex()]))
          .arg(s.faces)
          .arg(s.volume));
}

void MainWindow::runCenter() {
  if (!requireMesh())
    return;
  Mesh out = m_doc.activeMesh();
  QString error;
  if (!centerAtVolumeCentroid(out, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  m_doc.appendOperation(QStringLiteral("Center"), QStringLiteral("at volume center"),
                        std::move(out));
  m_visible.insert(m_doc.activeIndex());
  log(QStringLiteral("Centered geometry at center of volume."));
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
  // Docs-page style relative parameters: scale by the active mesh bbox.
  const MeshStats inStats = computeStats(m_doc.activeMesh());
  const double dx = inStats.bboxMax[0] - inStats.bboxMin[0];
  const double dy = inStats.bboxMax[1] - inStats.bboxMin[1];
  const double dz = inStats.bboxMax[2] - inStats.bboxMin[2];
  const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
  double alpha = 0.0, offset = 0.0;
  QString error;
  if (!alphaWrapParams(diag, d["Alpha denominator (bbox diag / alpha)"],
                       d["Offset denominator (bbox diag / offset)"], alpha,
                       offset, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  Mesh out;
  if (!alphaWrap(m_doc.activeMesh(), alpha, offset, out, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  const MeshStats s = computeStats(out);
  m_doc.appendOperation(
      QStringLiteral("AlphaWrap"),
      QStringLiteral("alpha=diag/%1 (%2) offset=diag/%3 (%4)")
          .arg(d["Alpha denominator (bbox diag / alpha)"])
          .arg(alpha)
          .arg(d["Offset denominator (bbox diag / offset)"])
          .arg(offset),
      std::move(out));
  m_visible.insert(m_doc.activeIndex());
  log(QStringLiteral("Alpha wrap: %1 faces, closed=%2.")
          .arg(s.faces)
          .arg(s.closed ? QStringLiteral("yes") : QStringLiteral("no")));
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
  Mesh out;
  QString error;
  if (!clipWithPlane(m_doc.activeMesh(),
                     makePoint(d["Origin x"], d["Origin y"], d["Origin z"]),
                     Vector3(d["Normal x"], d["Normal y"], d["Normal z"]), out,
                     error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  m_doc.appendOperation(
      QStringLiteral("Slice"),
      QStringLiteral("n=(%1,%2,%3)").arg(d["Normal x"]).arg(d["Normal y"]).arg(d["Normal z"]),
      std::move(out));
  m_visible.insert(m_doc.activeIndex());
  log(QStringLiteral("Sliced with plane; %1 faces remain.").arg(computeStats(out).faces));
}

void MainWindow::runDetectLeaks() {
  if (!requireMesh())
    return;
  const auto probes = probesFromTable();
  m_doc.setProbes(probes);
  std::vector<std::array<double, 3>> inPts, outPts;
  for (const auto& pr : probes) {
    if (pr.inside)
      inPts.push_back({pr.x, pr.y, pr.z});
    else
      outPts.push_back({pr.x, pr.y, pr.z});
  }
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("Leak detection grid"), {},
                   {{"Voxels per axis", ProbeParams{}.gridRes}}, d, iv))
    return;
  ProbeParams probeParams;
  probeParams.gridRes = iv["Voxels per axis"];
  const auto res = detectLeaks(m_doc.activeMesh(), inPts, outPts, probeParams);
  const auto loops = findHoleLoops(m_doc.activeMesh());
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
  Mesh probed = m_doc.activeMesh();
  m_doc.appendOperation(
      QStringLiteral("LeakProbe"),
      QStringLiteral("%1 loops, %2 leak(s)").arg(loops.size()).arg(res.paths.size()),
      std::move(probed));
  m_doc.setLeakReport(report);
  m_visible.remove(leakSource);
  m_visible.insert(m_doc.activeIndex());
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
  // Default: only patch holes that leak paths actually traverse. That
  // needs probes, so re-probe silently here.
  QSet<int> leakingLoops;
  LeakProbeResult silentProbe; // kept so the PatchHoles stage can show
                               // the routes/grid its patching was based on
  if (!patchAll) {
    const auto probes = probesFromTable();
    if (probes.isEmpty()) {
      QMessageBox::information(
          this, QStringLiteral("MeshBench"),
          QStringLiteral("Path-driven patching needs inside/outside probes. "
                         "Add probes, or tick 'Patch every boundary loop'."));
      return;
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
      return;
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
                         "tick 'Patch every boundary loop' to patch anyway."));
      return;
    }
  }
  Mesh out = m_doc.activeMesh();
  const PatchResult r = patchHoles(out, iv["Max loop edges"], leakingLoops);
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
}

void MainWindow::runGmshRemesh() {
#ifdef MESHBENCH_HAVE_GMSH
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("Gmsh remesh"),
                   {{"Target element size", 0.1}}, {}, d, iv))
    return;
  const QString tmpIn = QDir::temp().filePath(QStringLiteral("meshbench_in.stl"));
  const QString tmpOut = QDir::temp().filePath(QStringLiteral("meshbench_out.stl"));
  QString error;
  if (!saveMesh(tmpIn, m_doc.activeMesh(), error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  if (!remeshStlWithGmsh(tmpIn, tmpOut, d["Target element size"], error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  Mesh out;
  if (!loadMesh(tmpOut, out, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  m_doc.appendOperation(
      QStringLiteral("GmshRemesh"),
      QStringLiteral("size=%1").arg(d["Target element size"]), std::move(out));
  m_visible.insert(m_doc.activeIndex());
  log(QStringLiteral("Gmsh remesh done: %1 faces.").arg(computeStats(out).faces));
#else
  QMessageBox::information(this, QStringLiteral("MeshBench"),
                           QStringLiteral("Gmsh support not compiled in."));
#endif
}

void MainWindow::runCreatePrimitive() {
#ifdef MESHBENCH_HAVE_GMSH
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
    while (dimForm->count() > 0) {
      auto* item = dimForm->takeAt(0);
      delete item->widget();
      delete item;
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

  const QString tmp =
      QDir::temp().filePath(QStringLiteral("meshbench_primitive.stl"));
  QString error;
  if (!createPrimitiveStl(params, tmp, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  Mesh mesh;
  if (!loadMesh(tmp, mesh, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
  QStringList dimStr;
  for (double v : params.dims)
    dimStr.append(QString::number(v));
  const MeshStats s = computeStats(mesh);
  m_doc.setSource(std::move(mesh),
                  QStringLiteral("%1(%2)").arg(primitiveShapeName(params.shape),
                                              dimStr.join(QStringLiteral(","))));
  m_visible.clear();
  m_visible.insert(0);
  probesToTable({});
  setDefaultProbes();
  log(QStringLiteral("Created %1 primitive: %2 faces, closed=%3.")
          .arg(primitiveShapeName(params.shape))
          .arg(s.faces)
          .arg(s.closed ? QStringLiteral("yes") : QStringLiteral("no")));
#else
  QMessageBox::information(this, QStringLiteral("MeshBench"),
                           QStringLiteral("Gmsh support not compiled in."));
#endif
}

void MainWindow::runIntroduceHoles() {
  if (!requireMesh())
    return;
  QMap<QString, double> d;
  QMap<QString, int> iv;
  if (!paramDialog(this, QStringLiteral("Introduce random holes"),
                   {}, {{"Triangles to remove", 10}, {"Seed", 42}}, d, iv))
    return;
  Mesh out = m_doc.activeMesh();
  QString error;
  const int removed = removeRandomTriangles(out, iv["Triangles to remove"],
                                            static_cast<unsigned>(iv["Seed"]),
                                            error);
  if (removed == 0) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"),
                         error.isEmpty() ? QStringLiteral("Nothing removed.")
                                         : error);
    return;
  }
  const MeshStats s = computeStats(out);
  m_doc.appendOperation(QStringLiteral("Puncture"),
                        QStringLiteral("removed %1 tris (seed %2)")
                            .arg(removed)
                            .arg(iv["Seed"]),
                        std::move(out));
  m_visible.insert(m_doc.activeIndex());
  log(QStringLiteral("Removed %1 triangles; now %2 hole loop(s), %3 boundary "
                     "edges. Run Detect leaks to see them.")
          .arg(removed)
          .arg(findHoleLoops(m_doc.activeMesh()).size())
          .arg(s.boundaryEdges));
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

  const int source = m_doc.activeIndex();
  Mesh sphere = makeEnclosingSphere(m_doc.activeMesh(), m_shrinkParams.subdivisions);
  QString error;
  if (!snapShrinkwrap(m_doc.activeMesh(), {}, sphere, m_shrinkParams, error)) {
    QMessageBox::warning(this, QStringLiteral("MeshBench"), error);
    return;
  }
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
  updateShrinkPanel();
  log(QStringLiteral("Shrinkwrap stage %1 from stage %2 (%3 faces). Enable "
                     "'Select panels', click target triangles, set a weight, "
                     "Apply.")
          .arg(m_wrapStage)
          .arg(m_wrapSource)
          .arg(computeStats(m_doc.node(m_wrapStage).mesh).faces));
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

void MainWindow::onFacePicked(int faceIdx) {
  if (faceIdx < 0)
    return;
  // Click toggles membership so stray clicks are easy to undo.
  if (m_faceSelection.contains(faceIdx))
    m_faceSelection.remove(faceIdx);
  else
    m_faceSelection.insert(faceIdx);
  // Show the clicked panel's current weight in the editors.
  m_weightSpin->setValue(m_faceWeights.value(faceIdx, 1.0));
  updateShrinkPanel();
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
  if (m_updatingTree || !item || column != 0)
    return;
  const int idx = item->data(0, Qt::UserRole).toInt();
  if (item->checkState(0) == Qt::Checked)
    m_visible.insert(idx);
  else
    m_visible.remove(idx);
  refreshView();
}

void MainWindow::onTreeItemActivated(QTreeWidgetItem* item, int /*column*/) {
  if (!item)
    return;
  m_doc.setActive(item->data(0, Qt::UserRole).toInt());
}

void MainWindow::revertToSelectedStage() {
  auto* item = m_tree->currentItem();
  if (!item || !requireMesh())
    return;
  const int idx = item->data(0, Qt::UserRole).toInt();
  m_doc.truncateAfter(idx);
  m_visible.clear();
  m_visible.insert(m_doc.activeIndex());
}

void MainWindow::refreshAll() {
  validateShrinkState();
  rebuildTree();
  updateStagePanel();
  refreshView();
}

void MainWindow::rebuildTree() {
  m_updatingTree = true;
  m_tree->clear();
  for (int i = 0; i < m_doc.nodeCount(); ++i) {
    const auto& node = m_doc.node(i);
    auto* item = new QTreeWidgetItem(m_tree);
    QString label = QStringLiteral("%1: %2").arg(i).arg(node.label);
    if (!node.details.isEmpty())
      label += QStringLiteral(" (%1)").arg(node.details);
    item->setText(0, label);
    item->setData(0, Qt::UserRole, i);
    item->setCheckState(0, m_visible.contains(i) ? Qt::Checked : Qt::Unchecked);
    const MeshStats s = computeStats(node.mesh);
    item->setText(1, QStringLiteral("%1 f / %2 bnd")
                         .arg(s.faces)
                         .arg(s.boundaryEdges));
    if (i == m_doc.activeIndex()) {
      item->setSelected(true);
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
