#pragma once

// Minimal MCP (Model Context Protocol) JSON-RPC layer.
//
// Transport is handled by McpHttpServer; this class only maps JSON-RPC MCP
// messages to a tool catalog + command callback. Keeping it independent of
// sockets and of MainWindow makes the protocol unit-testable with a fake
// backend (tests/tst_mcp.cpp).

#include "Commands.h"

#include <QJsonObject>
#include <QString>
#include <QVector>

#include <functional>

namespace meshbench {

class McpProtocol {
public:
  using ToolsFn = std::function<QVector<CommandInfo>()>;
  using CallFn = std::function<CommandReply(const QString&, const QJsonObject&)>;

  McpProtocol(ToolsFn tools, CallFn call);

  // Handle one JSON-RPC message. Returns an empty object for notifications
  // (messages without an id), which the HTTP layer sends as 202 Accepted.
  QJsonObject handle(const QJsonObject& message);

  QString serverName() const { return QStringLiteral("meshbench"); }
  QString serverVersion() const { return QStringLiteral("0.1.0"); }

private:
  QJsonObject handleInitialize(const QJsonValue& id, const QJsonObject& params);
  QJsonObject handleToolsCall(const QJsonValue& id, const QJsonObject& params);
  QJsonObject result(const QJsonValue& id, QJsonObject value) const;
  QJsonObject error(const QJsonValue& id, int code, const QString& message) const;

  ToolsFn m_tools;
  CallFn m_call;
};

} // namespace meshbench
