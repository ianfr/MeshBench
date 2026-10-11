#include "VtkView.h"
#include "LeakFix.h"

#include <cmath>
#include <unordered_map>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCellPicker.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkTriangle.h>

namespace meshbench {

// Click-to-select interactor style: a left click (no drag) picks a cell
// from the registered actor and forwards the mapped face index. Camera
// motion is swallowed while picking so clicks never rotate the scene.
class VtkPickStyle : public vtkInteractorStyleTrackballCamera {
public:
  static VtkPickStyle* New();
  vtkTypeMacro(VtkPickStyle, vtkInteractorStyleTrackballCamera);

  vtkActor* PickActor = nullptr;
  vtkRenderer* PickRenderer = nullptr;
  const std::vector<std::size_t>* PickFaces = nullptr;
  std::function<void(int)> OnPick;
  int PressX = 0, PressY = 0;

  void OnLeftButtonDown() override {
    PressX = Interactor->GetEventPosition()[0];
    PressY = Interactor->GetEventPosition()[1];
  }

  void OnLeftButtonUp() override {
    const int* pos = Interactor->GetEventPosition();
    const int dx = pos[0] - PressX, dy = pos[1] - PressY;
    if (dx * dx + dy * dy <= 9 && PickActor && PickRenderer && PickFaces &&
        OnPick) {
      auto picker = vtkSmartPointer<vtkCellPicker>::New();
      picker->PickFromListOn();
      picker->AddPickList(PickActor);
      picker->SetTolerance(0.0);
      if (picker->Pick(pos[0], pos[1], 0.0, PickRenderer) &&
          picker->GetCellId() >= 0) {
        const vtkIdType cell = picker->GetCellId();
        if (cell < static_cast<vtkIdType>(PickFaces->size()))
          OnPick(static_cast<int>((*PickFaces)[static_cast<std::size_t>(cell)]));
      }
    }
  }
};
vtkStandardNewMacro(VtkPickStyle);

VtkView::VtkView(QWidget* parent) : QVTKOpenGLNativeWidget(parent) {
  auto window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  setRenderWindow(window);
  m_renderer = vtkSmartPointer<vtkRenderer>::New();
  m_renderer->SetBackground(0.13, 0.15, 0.2);
  // Translucent shrinkwrap over an opaque baseline needs correct sorting.
  m_renderer->SetUseDepthPeeling(1);
  m_renderer->SetMaximumNumberOfPeels(4);
  m_renderer->SetOcclusionRatio(0.0);
  window->AddRenderer(m_renderer);
}

VtkView::~VtkView() = default;

vtkSmartPointer<vtkActor> VtkView::meshActor(const Mesh& mesh, double r,
                                             double g, double b,
                                             double opacity,
                                             std::vector<std::size_t>& cellFaces) {
  cellFaces.clear();
  auto points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToDouble();
  // NB: Surface_mesh indices are only dense when nothing was removed, so
  // map them explicitly instead of assuming density.
  std::unordered_map<std::size_t, vtkIdType> idOf;
  for (auto v : mesh.vertices()) {
    const Point3& p = mesh.point(v);
    idOf[v.idx()] = points->InsertNextPoint(toDouble(p.x()), toDouble(p.y()),
                                            toDouble(p.z()));
  }
  auto tris = vtkSmartPointer<vtkCellArray>::New();
  for (auto f : mesh.faces()) {
    // Fan-triangulate so polygonal faces display (and pick) correctly.
    std::vector<Mesh::Vertex_index> ring;
    auto h = mesh.halfedge(f);
    const auto start = h;
    do {
      ring.push_back(mesh.target(h));
      h = mesh.next(h);
    } while (h != start);
    for (std::size_t i = 1; i + 1 < ring.size(); ++i) {
      auto tri = vtkSmartPointer<vtkTriangle>::New();
      tri->GetPointIds()->SetId(0, idOf[ring[0].idx()]);
      tri->GetPointIds()->SetId(1, idOf[ring[i].idx()]);
      tri->GetPointIds()->SetId(2, idOf[ring[i + 1].idx()]);
      tris->InsertNextCell(tri);
      cellFaces.push_back(f.idx());
    }
  }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetPolys(tris);
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(poly);
  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(r, g, b);
  actor->GetProperty()->SetOpacity(opacity);
  actor->GetProperty()->EdgeVisibilityOn();
  actor->GetProperty()->SetEdgeColor(0.1, 0.1, 0.12);
  return actor;
}

void VtkView::addHoleLoopActors(const Mesh& mesh, double r, double g,
                              double b) {
  const auto loops = findHoleLoops(mesh);
  for (const auto& loop : loops) {
    if (loop.points.isEmpty())
      continue;
    auto points = vtkSmartPointer<vtkPoints>::New();
    for (const auto& p : loop.points)
      points->InsertNextPoint(p[0], p[1], p[2]);
    auto lines = vtkSmartPointer<vtkCellArray>::New();
    const vtkIdType n = points->GetNumberOfPoints();
    for (vtkIdType i = 0; i < n; ++i) {
      auto line = vtkSmartPointer<vtkLine>::New();
      line->GetPointIds()->SetId(0, i);
      line->GetPointIds()->SetId(1, (i + 1) % n);
      lines->InsertNextCell(line);
    }
    auto poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(points);
    poly->SetLines(lines);
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(poly);
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(r, g, b);
    actor->GetProperty()->SetLineWidth(4.0);
    m_renderer->AddActor(actor);
    m_overlayActors.push_back(actor);
  }
}

void VtkView::addProbeActors(const QVector<ProbePoint>& probes) {
  // Size probe spheres relative to the scene before creating them.
  double bounds[6] = {0, 1, 0, 1, 0, 1};
  m_renderer->ComputeVisiblePropBounds(bounds);
  const double diag =
      std::sqrt(std::pow(bounds[1] - bounds[0], 2) +
                std::pow(bounds[3] - bounds[2], 2) +
                std::pow(bounds[5] - bounds[4], 2));
  m_probeMarkerRad = std::max(diag * 0.01, 1e-6);
  for (const auto& pr : probes) {
    auto src = vtkSmartPointer<vtkSphereSource>::New();
    src->SetCenter(pr.x, pr.y, pr.z);
    src->SetRadius(m_probeMarkerRad);
    src->SetPhiResolution(16);
    src->SetThetaResolution(16);
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(src->GetOutputPort());
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    if (pr.inside)
      actor->GetProperty()->SetColor(0.2, 1.0, 0.3);
    else
      actor->GetProperty()->SetColor(0.3, 0.5, 1.0);
    m_renderer->AddActor(actor);
    m_overlayActors.push_back(actor);
  }
}

void VtkView::addLeakPathActors(const LeakReport& leaks) {
  // BFS shortest paths in red (with endpoint markers), seeded random-walk
  // routes in thinner orange: together these are every route the probe
  // found, i.e. what path-driven patching is based on.
  const double rad = m_probeMarkerRad > 0 ? m_probeMarkerRad : 1e-6;
  for (int pass = 0; pass < 2; ++pass) {
    const auto& group = pass == 0 ? leaks.leakPaths : leaks.walkPaths;
    for (const auto& line : group) {
      if (line.size() < 2)
        continue;
      auto points = vtkSmartPointer<vtkPoints>::New();
      for (const auto& p : line)
        points->InsertNextPoint(p[0], p[1], p[2]);
      auto lines = vtkSmartPointer<vtkCellArray>::New();
      const vtkIdType n = points->GetNumberOfPoints();
      for (vtkIdType i = 0; i + 1 < n; ++i) {
        auto seg = vtkSmartPointer<vtkLine>::New();
        seg->GetPointIds()->SetId(0, i);
        seg->GetPointIds()->SetId(1, i + 1);
        lines->InsertNextCell(seg);
      }
      auto poly = vtkSmartPointer<vtkPolyData>::New();
      poly->SetPoints(points);
      poly->SetLines(lines);
      auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputData(poly);
      auto actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      if (pass == 0) {
        actor->GetProperty()->SetColor(1.0, 0.1, 0.1);
        actor->GetProperty()->SetLineWidth(5.0);
      } else {
        actor->GetProperty()->SetColor(1.0, 0.55, 0.1);
        actor->GetProperty()->SetLineWidth(2.0);
      }
      m_renderer->AddActor(actor);
      m_overlayActors.push_back(actor);
      if (pass != 0)
        continue;
      // Endpoint markers so each leak is visible even at a glance: red
      // sphere where the path leaves the inside probe, larger red sphere
      // where it reaches the outside.
      const std::array<double, 3> ends[2] = {line.front(), line.back()};
      for (int e = 0; e < 2; ++e) {
        auto marker = vtkSmartPointer<vtkSphereSource>::New();
        marker->SetCenter(ends[e][0], ends[e][1], ends[e][2]);
        marker->SetRadius(rad * (e == 0 ? 1.5 : 2.5));
        marker->SetPhiResolution(16);
        marker->SetThetaResolution(16);
        auto markerMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        markerMapper->SetInputConnection(marker->GetOutputPort());
        auto markerActor = vtkSmartPointer<vtkActor>::New();
        markerActor->SetMapper(markerMapper);
        markerActor->GetProperty()->SetColor(1.0, 0.1, 0.1);
        m_renderer->AddActor(markerActor);
        m_overlayActors.push_back(markerActor);
      }
    }
  }
}

void VtkView::addVoxelGridActors(const LeakReport& leaks) {
  const int n = leaks.gridRes;
  if (n < 4 || leaks.cellSize <= 0.0)
    return;
  // Cap the drawn lattice at ~48 lines per axis so high-res grids stay
  // interactive; the grid itself is unchanged, only its display strides.
  const int stride = std::max(1, (n + 47) / 48);
  std::vector<int> planes;
  for (int i = 0; i <= n; i += stride)
    planes.push_back(i);
  if (planes.back() != n)
    planes.push_back(n);
  const double ox = leaks.gridOrigin[0], oy = leaks.gridOrigin[1],
               oz = leaks.gridOrigin[2];
  const double dx = leaks.cellSize;
  const vtkIdType m = static_cast<vtkIdType>(planes.size());
  auto points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToDouble();
  for (vtkIdType a = 0; a < m; ++a)
    for (vtkIdType b = 0; b < m; ++b)
      for (vtkIdType c = 0; c < m; ++c)
        points->InsertNextPoint(ox + planes[a] * dx, oy + planes[b] * dx,
                                oz + planes[c] * dx);
  auto at = [&](vtkIdType a, vtkIdType b, vtkIdType c) {
    return (a * m + b) * m + c;
  };
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  for (vtkIdType a = 0; a < m; ++a)
    for (vtkIdType b = 0; b < m; ++b)
      for (vtkIdType c = 0; c < m; ++c) {
        if (a + 1 < m) {
          auto seg = vtkSmartPointer<vtkLine>::New();
          seg->GetPointIds()->SetId(0, at(a, b, c));
          seg->GetPointIds()->SetId(1, at(a + 1, b, c));
          lines->InsertNextCell(seg);
        }
        if (b + 1 < m) {
          auto seg = vtkSmartPointer<vtkLine>::New();
          seg->GetPointIds()->SetId(0, at(a, b, c));
          seg->GetPointIds()->SetId(1, at(a, b + 1, c));
          lines->InsertNextCell(seg);
        }
        if (c + 1 < m) {
          auto seg = vtkSmartPointer<vtkLine>::New();
          seg->GetPointIds()->SetId(0, at(a, b, c));
          seg->GetPointIds()->SetId(1, at(a, b, c + 1));
          lines->InsertNextCell(seg);
        }
      }
  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetLines(lines);
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(poly);
  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(0.6, 0.62, 0.68);
  actor->GetProperty()->SetOpacity(0.3);
  m_renderer->AddActor(actor);
  m_overlayActors.push_back(actor);
}

void VtkView::showDocument(const MeshDocument& doc,
                           const QSet<int>& visibleStages, int activeStage) {
  // Rebuilding actors invalidates the pick actor; restore the mode after.
  const bool keepPick = m_pickMode;
  const int keepPickStage = m_pickStage;
  if (m_pickMode)
    setPickMode(false);
  m_renderer->RemoveAllViewProps();
  m_stageActors.clear();
  m_stageCellFaces.clear();
  m_overlayActors.clear();
  if (!doc.hasMesh()) {
    renderWindow()->Render();
    return;
  }
  // Distinct palette per stage so multiple visible stages are telling apart.
  static const double palette[][3] = {
      {0.85, 0.75, 0.55}, {0.55, 0.75, 0.9}, {0.6, 0.9, 0.6},
      {0.9, 0.6, 0.6}, {0.8, 0.7, 0.95}, {0.7, 0.9, 0.9},
  };
  int color = 0;
  for (int i = 0; i < doc.nodeCount(); ++i) {
    if (!visibleStages.contains(i))
      continue;
    const bool active = (i == activeStage);
    const bool isWrap = doc.node(i).label.startsWith(QStringLiteral("Shrinkwrap"));
    const double* c = isWrap ? nullptr : palette[color++ % 6];
    std::vector<std::size_t> cellFaces;
    // The wrap is always translucent cyan so the baseline shows through.
    // Other stages default to opaque when active, faint otherwise; an
    // opacity pinned in the inspector overrides the default.
    const double pinned = doc.node(i).opacity;
    const double op =
        isWrap ? 0.3 : (pinned >= 0.0 ? pinned : (active ? 1.0 : 0.35));
    auto actor = isWrap ? meshActor(doc.node(i).mesh, 0.35, 0.9, 0.95, op, cellFaces)
                        : meshActor(doc.node(i).mesh, c[0], c[1], c[2],
                                    op, cellFaces);
    m_renderer->AddActor(actor);
    m_stageActors[i] = actor;
    m_stageCellFaces[i] = std::move(cellFaces);
  }
  // Editing highlights (hole loops, probes, leak paths) belong to the
  // active stage only, and vanish entirely when its eye is off — an
  // unchecked tree item contributes zero actors, like hiding a ParaView
  // pipeline object.
  if (activeStage >= 0 && activeStage < doc.nodeCount() &&
      visibleStages.contains(activeStage)) {
    const auto& node = doc.node(activeStage);
    // LeakProbe outlines glow green: these loops are the patch candidates.
    // Everywhere else holes stay red (damage to inspect, not to patch).
    const bool isProbe = showsProbeOverlays(node.label);
    if (isProbe)
      addHoleLoopActors(node.mesh, 0.15, 0.9, 0.3);
    else
      addHoleLoopActors(node.mesh, 1.0, 0.15, 0.15);
    // Probe spheres belong to the LeakProbe filter: they render only
    // while that stage is the activated one. (Probe points still carry
    // forward in the data for re-probing; they just don't draw on other
    // stages.) Leak paths and the voxel grid are opt-in overlays, both
    // default off: they render on the active LeakProbe stage, and also on
    // a PatchHoles stage carrying the routes/grid its patching was based
    // on (stored even when no Detect-leaks run precedes patching).
    if (isProbe)
      addProbeActors(node.probes);
    // Gated on stored data, not the stage label: LeakProbe stages always
    // carry it, and PatchHoles stages keep the routes/grid their patching
    // was based on. Empty reports draw nothing.
    if (m_showLeakPaths && node.leaks.hasStoredRoutes())
      addLeakPathActors(node.leaks);
    if (m_showVoxelGrid && node.leaks.hasStoredGrid())
      addVoxelGridActors(node.leaks);
  }
  m_renderer->ResetCamera();
  renderWindow()->Render();
  if (keepPick && keepPickStage >= 0)
    setPickMode(true, keepPickStage);
}

void VtkView::setPickMode(bool on, int stage) {
  auto* interactor = renderWindow()->GetInteractor();
  if (!on) {
    m_pickMode = false;
    m_pickActor = nullptr;
    if (interactor && m_savedStyle)
      interactor->SetInteractorStyle(m_savedStyle);
    m_savedStyle = nullptr;
    return;
  }
  auto it = m_stageActors.find(stage);
  if (it == m_stageActors.end() || !interactor)
    return;
  if (!m_pickMode)
    m_savedStyle =
        vtkInteractorStyle::SafeDownCast(interactor->GetInteractorStyle());
  m_pickMode = true;
  m_pickStage = stage;
  m_pickActor = it.value();
  m_pickFaces = m_stageCellFaces.value(stage);
  if (!m_pickStyle)
    m_pickStyle = vtkSmartPointer<VtkPickStyle>::New();
  m_pickStyle->PickActor = m_pickActor;
  m_pickStyle->PickRenderer = m_renderer;
  m_pickStyle->PickFaces = &m_pickFaces;
  m_pickStyle->OnPick = [this](int faceIdx) { emit facePicked(faceIdx); };
  interactor->SetInteractorStyle(m_pickStyle);
}

void VtkView::clear() {
  if (m_pickMode)
    setPickMode(false);
  m_renderer->RemoveAllViewProps();
  m_stageActors.clear();
  m_stageCellFaces.clear();
  m_overlayActors.clear();
  renderWindow()->Render();
}

void VtkView::resetCamera() {
  m_renderer->ResetCamera();
  renderWindow()->Render();
}

} // namespace meshbench
