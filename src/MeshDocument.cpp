#include "MeshDocument.h"

namespace meshbench {

MeshDocument::MeshDocument(QObject* parent) : QObject(parent) {}

void MeshDocument::setSource(Mesh&& mesh, const QString& label) {
  m_nodes.clear();
  PipelineNode node;
  node.label = label.isEmpty() ? QStringLiteral("Source") : label;
  node.timestamp = QDateTime::currentDateTime();
  node.mesh = std::move(mesh);
  m_nodes.append(std::move(node));
  m_active = 0;
  emit historyChanged();
  emit activeChanged(m_active);
}

int MeshDocument::appendOperation(const QString& label, const QString& details, Mesh&& mesh) {
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
  // Probe points carry forward so leak checks stay comparable across stages.
  node.probes = m_nodes.at(m_active).probes;
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
