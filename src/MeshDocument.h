#pragma once

// MeshDocument: owns the current surface mesh plus the ParaView-style
// pipeline history shown in the left-hand sidebar.
//
// Node 0 is always the loaded source mesh. Every operation appends a new
// node holding a full snapshot of the resulting mesh. Selecting a node in
// the sidebar makes it the active (displayed/operated-on) stage; applying
// a new operation while an older stage is active truncates the redo tail
// first. Each node also carries the inside/outside probe points and the
// leak report produced at that stage so results stay attached to history.

#include "MeshTypes.h"

#include <QDateTime>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>
#include <array>

namespace meshbench {

struct ProbePoint {
  double x = 0.0, y = 0.0, z = 0.0;
  bool inside = true; // true = locationInMesh, false = locationOutsideMesh
};

struct LeakReport {
  bool probed = false;
  bool hasLeak = false;
  int holeLoops = 0;
  int regions = 0;
  // One polyline (cell centers) per inside->outside leak path found.
  QVector<QVector<std::array<double, 3>>> leakPaths;
  // Extra routes from the seeded random walks (same polyline format).
  QVector<QVector<std::array<double, 3>>> walkPaths;
  // Background voxel grid the probe ran on (for the grid overlay).
  int gridRes = 0;
  std::array<double, 3> gridOrigin = {0.0, 0.0, 0.0};
  double cellSize = 0.0;
  QStringList warnings;
  // Display data outlives the probe verdict: a PatchHoles stage stores
  // the routes/grid its patching was based on (with probed=false, so it
  // stays a plain mesh with no LEAK/sealed tag), and the view toggles
  // render them there too.
  bool hasStoredRoutes() const {
    return !leakPaths.isEmpty() || !walkPaths.isEmpty();
  }
  bool hasStoredGrid() const { return gridRes >= 4 && cellSize > 0.0; }
};

struct PipelineNode {
  QString label;        // e.g. "AlphaWrap", "Source"
  QString details;      // parameter summary
  QDateTime timestamp;
  Mesh mesh;
  QVector<ProbePoint> probes;
  LeakReport leaks;
  double opacity = -1.0; // pinned stage opacity, <0 = auto (opaque when
                         // active, faint otherwise)
};

class MeshDocument : public QObject {
  Q_OBJECT
public:
  explicit MeshDocument(QObject* parent = nullptr);

  bool hasMesh() const { return !m_nodes.isEmpty(); }
  int nodeCount() const { return m_nodes.size(); }
  int activeIndex() const { return m_active; }

  const PipelineNode& node(int i) const { return m_nodes.at(i); }
  PipelineNode& node(int i) { return m_nodes[i]; }
  const Mesh& activeMesh() const { return m_nodes.at(m_active).mesh; }

  // Replace everything with a freshly loaded source mesh.
  void setSource(Mesh&& mesh, const QString& label);

  // Append an operation result (truncates any redo tail first).
  // Returns the index of the new active node.
  int appendOperation(const QString& label, const QString& details, Mesh&& mesh);

  void setActive(int i);
  // Replace the mesh stored at node i in place (used to refresh a
  // shrinkwrap stage after weight edits without spamming history).
  // Returns false for an out-of-range index.
  bool replaceMesh(int i, Mesh&& mesh);
  // Drop every node after i (used by "Revert to this stage").
  void truncateAfter(int i);
  void clear();

  void setProbes(const QVector<ProbePoint>& probes);
  void setLeakReport(const LeakReport& report);

signals:
  void historyChanged();
  void activeChanged(int index);

private:
  QVector<PipelineNode> m_nodes;
  int m_active = -1;
};

} // namespace meshbench
