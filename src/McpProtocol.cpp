#include "McpProtocol.h"

#include <QJsonArray>
#include <QJsonValue>

namespace meshbench {

namespace {
constexpr const char* kJsonRpcVersion = "2.0";
// Latest protocol revision supported by this server. Clients that speak an
// older revision still work for the subset implemented here.
constexpr const char* kMcpProtocolVersion = "2025-03-26";
} // namespace

McpProtocol::McpProtocol(ToolsFn tools, CallFn call)
    : m_tools(std::move(tools)), m_call(std::move(call)) {
  Q_ASSERT(m_tools);
  Q_ASSERT(m_call);
}

QJsonObject McpProtocol::result(const QJsonValue& id, QJsonObject value) const {
  return QJsonObject{{QStringLiteral("jsonrpc"), QString::fromLatin1(kJsonRpcVersion)},
                     {QStringLiteral("id"), id},
                     {QStringLiteral("result"), std::move(value)}};
}

QJsonObject McpProtocol::error(const QJsonValue& id, int code,
                               const QString& message) const {
  return QJsonObject{
      {QStringLiteral("jsonrpc"), QString::fromLatin1(kJsonRpcVersion)},
      {QStringLiteral("id"), id},
      {QStringLiteral("error"),
       QJsonObject{{QStringLiteral("code"), code},
                   {QStringLiteral("message"), message}}}};
}

QJsonObject McpProtocol::handleInitialize(const QJsonValue& id,
                                          const QJsonObject& params) {
  Q_UNUSED(params);
  QJsonObject resultJson;
  resultJson.insert(QStringLiteral("protocolVersion"),
                    QString::fromLatin1(kMcpProtocolVersion));
  resultJson.insert(
      QStringLiteral("capabilities"),
      QJsonObject{{QStringLiteral("tools"),
                   QJsonObject{{QStringLiteral("listChanged"), false}}}});
  resultJson.insert(QStringLiteral("serverInfo"),
                    QJsonObject{{QStringLiteral("name"), serverName()},
                                {QStringLiteral("version"), serverVersion()}});
  resultJson.insert(
      QStringLiteral("instructions"),
      QStringLiteral("MeshBench GUI control. Use scene_get_object_tree to "
                     "inspect the pipeline and scene_get_image to see the "
                     "viewport. Every File, Operations and View menu action is "
                     "available as a tool."));
  return result(id, std::move(resultJson));
}

QJsonObject McpProtocol::handleToolsCall(const QJsonValue& id,
                                         const QJsonObject& params) {
  const QJsonValue nameValue = params.value(QStringLiteral("name"));
  if (!nameValue.isString() || nameValue.toString().isEmpty())
    return error(id, -32602, QStringLiteral("tools/call requires a tool name."));
  const QJsonValue argsValue = params.value(QStringLiteral("arguments"));
  if (!argsValue.isUndefined() && !argsValue.isNull() && !argsValue.isObject())
    return error(id, -32602, QStringLiteral("tool arguments must be an object."));
  const QJsonObject args = argsValue.toObject();

  CommandReply reply;
  try {
    reply = m_call(nameValue.toString(), args);
  } catch (const std::exception& e) {
    return error(id, -32603,
                 QStringLiteral("Tool execution failed: %1")
                     .arg(QString::fromUtf8(e.what())));
  } catch (...) {
    return error(id, -32603, QStringLiteral("Tool execution failed."));
  }

  QJsonArray content;
  const QString text = reply.ok
                           ? (reply.message.isEmpty() ? QStringLiteral("OK")
                                                      : reply.message)
                           : (reply.error.isEmpty()
                                  ? QStringLiteral("Command failed.")
                                  : reply.error);
  content.append(QJsonObject{{QStringLiteral("type"), QStringLiteral("text")},
                             {QStringLiteral("text"), text}});
  if (reply.ok && !reply.image.isEmpty()) {
    content.append(
        QJsonObject{{QStringLiteral("type"), QStringLiteral("image")},
                    {QStringLiteral("mimeType"),
                     reply.imageMimeType.isEmpty() ? QStringLiteral("image/png")
                                                   : reply.imageMimeType},
                    {QStringLiteral("data"),
                     QString::fromLatin1(reply.image.toBase64())}});
  }

  QJsonObject resultJson;
  resultJson.insert(QStringLiteral("content"), content);
  resultJson.insert(QStringLiteral("isError"), !reply.ok);
  if (!reply.data.isEmpty())
    resultJson.insert(QStringLiteral("structuredContent"), reply.data);
  return result(id, std::move(resultJson));
}

QJsonObject McpProtocol::handle(const QJsonObject& message) {
  // Runtime safety checks: a message must at least carry a method name.
  if (message.isEmpty())
    return error(QJsonValue(QJsonValue::Null), -32600,
                 QStringLiteral("Empty JSON-RPC request."));
  const QJsonValue id = message.value(QStringLiteral("id"));
  const bool hasId = message.contains(QStringLiteral("id")) && !id.isUndefined();
  const QJsonValue methodValue = message.value(QStringLiteral("method"));
  if (!methodValue.isString() || methodValue.toString().isEmpty()) {
    if (!hasId)
      return {};
    return error(id, -32600, QStringLiteral("Missing JSON-RPC method."));
  }
  const QString method = methodValue.toString();

  // Notifications (no id) never get a response.
  if (!hasId)
    return {};

  const QJsonValue paramsValue = message.value(QStringLiteral("params"));
  const QJsonObject params =
      paramsValue.isObject() ? paramsValue.toObject() : QJsonObject{};

  if (method == QStringLiteral("initialize"))
    return handleInitialize(id, params);
  if (method == QStringLiteral("ping"))
    return result(id, {});
  if (method == QStringLiteral("tools/list")) {
    QJsonArray tools;
    for (const CommandInfo& info : m_tools()) {
      QJsonObject tool;
      tool.insert(QStringLiteral("name"), info.name);
      tool.insert(QStringLiteral("description"), info.description);
      tool.insert(QStringLiteral("inputSchema"),
                  info.inputSchema.isEmpty()
                      ? QJsonObject{{QStringLiteral("type"), QStringLiteral("object")}}
                      : info.inputSchema);
      tools.append(tool);
    }
    return result(id, QJsonObject{{QStringLiteral("tools"), tools}});
  }
  if (method == QStringLiteral("tools/call"))
    return handleToolsCall(id, params);
  // Discovery methods some clients probe even when not advertised.
  if (method == QStringLiteral("resources/list"))
    return result(id, QJsonObject{{QStringLiteral("resources"), QJsonArray{}}});
  if (method == QStringLiteral("resources/templates/list"))
    return result(id,
                  QJsonObject{{QStringLiteral("resourceTemplates"), QJsonArray{}}});
  if (method == QStringLiteral("prompts/list"))
    return result(id, QJsonObject{{QStringLiteral("prompts"), QJsonArray{}}});
  if (method == QStringLiteral("logging/setLevel"))
    return result(id, {});

  return error(id, -32601,
               QStringLiteral("Method not found: %1").arg(method));
}

} // namespace meshbench
