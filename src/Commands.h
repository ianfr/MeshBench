#pragma once

// Command layer shared by the GUI dialogs and the MCP server.
//
// Every File, Operations and View menu action has a scriptable equivalent
// (`MainWindow::executeCommand`). The catalog below is also the MCP tool
// list: names, descriptions and JSON Schemas are kept together so the GUI
// and the LLM-facing API cannot drift apart.

#include "MeshDocument.h"

#include <QByteArray>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>

#include <array>

namespace meshbench {

// Result of a scripted menu command. `image` is non-empty for image tools
// (PNG bytes) and is returned to MCP clients as an image content block.
struct CommandReply {
  bool ok = false;
  QString error;
  QString message;
  QJsonObject data;
  QByteArray image;
  QString imageMimeType = QStringLiteral("image/png");
};

// One MCP tool / scriptable menu command.
struct CommandInfo {
  QString name;
  QString description;
  QJsonObject inputSchema;
};

// The full tool catalog (File, Operations, View, Scene helpers).
QVector<CommandInfo> commandCatalog();

// Serialize the pipeline ("object tree") state. Exposed as
// scene_get_object_tree and as the GET /api/tree endpoint.
QJsonObject documentTreeStateJson(const MeshDocument& doc,
                                  const QSet<int>& visibleStages,
                                  const QSet<int>& combineSelected,
                                  int selectionStage, const QSet<int>& selection,
                                  const QVector<ProbePoint>& editorProbes);

QJsonArray probesJson(const QVector<ProbePoint>& probes);

// Small typed argument reader for JSON tool calls. Every getter records the
// first type/requirement error; callers check ok() once at the end.
class CommandArgs {
public:
  CommandArgs(const QJsonObject& args, QString commandName);

  bool ok() const { return m_error.isEmpty(); }
  QString error() const { return m_error; }
  bool has(const QString& key) const { return m_args.contains(key) && !m_args.value(key).isNull(); }

  QString string(const QString& key, const QString& fallback = {});
  QString requiredString(const QString& key);
  double number(const QString& key, double fallback);
  double requiredNumber(const QString& key);
  int integer(const QString& key, int fallback);
  int requiredInteger(const QString& key);
  bool boolean(const QString& key, bool fallback);
  QVector<double> doubles(const QString& key, const QVector<double>& fallback);
  QStringList strings(const QString& key, const QStringList& fallback = {});
  QJsonObject object(const QString& key);
  QJsonArray array(const QString& key, bool required = false);

private:
  void fail(const QString& message);

  QJsonObject m_args;
  QString m_command;
  QString m_error;
};

} // namespace meshbench
