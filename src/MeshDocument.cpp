#include "MeshDocument.h"

#include <algorithm>

namespace meshbench {

MeshDocument::MeshDocument(QObject* parent) : QObject(parent) {}

void MeshDocument::setSource(Mesh&& mesh, const QString& label,
                             FeatureEdges featureEdges) {
  m_nodes.clear();
  PipelineNode node;
  node.label = label.isEmpty() ? QStringLiteral("Source") : label;
  node.timestamp = QDateTime::currentDateTime();
  node.mesh = std::move(mesh);
  node.featureEdges = std::move(featureEdges);
  m_nodes.append(std::move(node));
  m_active = 0;
  emit historyChanged();
  emit activeChanged(m_active);
}

int MeshDocument::appendOperation(const QString& label, const QString& details,
                                  Mesh&& mesh, FeatureEdges featureEdges) {
  if (m_nodes.isEmpty()) {
    setSource(std::move(mesh), label);
    return m_active;
  }
  // Branching like ParaView: a new filter on an old stage drops the redo tail.
  while (m_nodes.size() > m_active + 1)
    m_nodes.removeLast();
  PipelineNode node;
  node.label = label;
  node.details = details;
  node.timestamp = QDateTime::currentDateTime();
  node.mesh = std::move(mesh);
  node.featureEdges = std::move(featureEdges);
  node.parent = m_active; // operation depends on the active stage
  // Probe points carry forward so leak checks stay comparable across stages.
  node.probes = m_nodes.at(m_active).probes;
  m_nodes.append(std::move(node));
  m_active = m_nodes.size() - 1;
  emit historyChanged();
  emit activeChanged(m_active);
  return m_active;
}

int MeshDocument::appendSource(const QString& label, Mesh&& mesh,
                               FeatureEdges featureEdges) {
  if (m_nodes.isEmpty()) {
    setSource(std::move(mesh), label, std::move(featureEdges));
    return m_active;
  }
  // Imports extend the tip: jumping the active pointer there first keeps
  // every existing node instead of dropping a redo tail.
  m_active = m_nodes.size() - 1;
  PipelineNode node;
  node.label = label.isEmpty() ? QStringLiteral("Source") : label;
  node.timestamp = QDateTime::currentDateTime();
  node.mesh = std::move(mesh);
  node.featureEdges = std::move(featureEdges);
  m_nodes.append(std::move(node));
  m_active = m_nodes.size() - 1;
  emit historyChanged();
  emit activeChanged(m_active);
  return m_active;
}

void MeshDocument::setActive(int i) {
  if (i < 0 || i >= m_nodes.size() || i == m_active)
    return;
  m_active = i;
  emit activeChanged(m_active);
}

bool MeshDocument::replaceMesh(int i, Mesh&& mesh) {
  if (i < 0 || i >= m_nodes.size())
    return false;
  m_nodes[i].mesh = std::move(mesh);
  m_nodes[i].timestamp = QDateTime::currentDateTime();
  emit historyChanged();
  return true;
}

QMap<int, int> MeshDocument::removeNodes(const QSet<int>& indices) {
  QMap<int, int> mapping;
  const int oldCount = m_nodes.size();
  for (int i = 0; i < oldCount; ++i)
    mapping.insert(i, i);
  if (oldCount == 0)
    return mapping;

  QSet<int> remove;
  for (int i : indices) {
    if (i >= 0 && i < oldCount)
      remove.insert(i);
  }
  if (remove.isEmpty())
    return mapping;

  // Expand the request to every descendant. Operation nodes record their
  // producing stage in parent; independent sources are roots.
  QMap<int, QVector<int>> children;
  for (int i = 0; i < oldCount; ++i) {
    if (m_nodes[i].parent >= 0)
      children[m_nodes[i].parent].append(i);
  }
  QVector<int> pending;
  for (int i : remove)
    pending.append(i);
  while (!pending.isEmpty()) {
    const int parent = pending.takeLast();
    for (int child : children.value(parent)) {
      if (!remove.contains(child)) {
        remove.insert(child);
        pending.append(child);
      }
    }
  }

  const int oldActive = m_active;
  QVector<PipelineNode> kept;
  kept.reserve(oldCount - remove.size());
  mapping.clear();
  int next = 0;
  for (int i = 0; i < oldCount; ++i) {
    if (remove.contains(i))
      continue;
    mapping.insert(i, next++);
    kept.append(std::move(m_nodes[i]));
  }
  // Any surviving node whose parent was removed would have been pulled
  // into the descendant set; remap the rest onto their new indices.
  for (auto& node : kept) {
    if (node.parent >= 0)
      node.parent = mapping.value(node.parent, -1);
  }
  m_nodes = std::move(kept);

  if (m_nodes.isEmpty()) {
    m_active = -1;
  } else if (mapping.contains(oldActive)) {
    m_active = mapping.value(oldActive);
  } else {
    // Deleted the active stage: land on the nearest surviving stage.
    int survivorsBefore = 0;
    for (int i = 0; i < oldActive && i < oldCount; ++i) {
      if (!remove.contains(i))
        ++survivorsBefore;
    }
    m_active = std::min(survivorsBefore, static_cast<int>(m_nodes.size()) - 1);
  }
  emit historyChanged();
  emit activeChanged(m_active);
  return mapping;
}

void MeshDocument::truncateAfter(int i) {
  if (i < 0 || i >= m_nodes.size())
    return;
  while (m_nodes.size() > i + 1)
    m_nodes.removeLast();
  m_active = i;
  emit historyChanged();
  emit activeChanged(m_active);
}

void MeshDocument::clear() {
  m_nodes.clear();
  m_active = -1;
  emit historyChanged();
  emit activeChanged(m_active);
}

void MeshDocument::setProbes(const QVector<ProbePoint>& probes) {
  if (m_active < 0)
    return;
  m_nodes[m_active].probes = probes;
  emit historyChanged();
}

void MeshDocument::setLeakReport(const LeakReport& report) {
  if (m_active < 0)
    return;
  m_nodes[m_active].leaks = report;
  emit historyChanged();
}

} // namespace meshbench
