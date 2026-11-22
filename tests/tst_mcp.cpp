// Unit tests for the MCP JSON-RPC protocol and the Beast HTTP transport.
//
// The protocol is tested against a fake command backend, so no GUI or mesh
// document is required. The HTTP test starts the real server on an
// ephemeral loopback port and performs one request/response round trip.

#include "Commands.h"
#include "McpHttpServer.h"
#include "McpProtocol.h"

#include <QtTest>

#include <QJsonArray>
#include <QJsonObject>

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include <string>

using namespace meshbench;
namespace beast = boost::beast;
namespace http = beast::http;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace {

CommandInfo fakeTool(const QString& name) {
  return CommandInfo{
      name, QStringLiteral("fake tool"),
      QJsonObject{{QStringLiteral("type"), QStringLiteral("object")},
                  {QStringLiteral("properties"), QJsonObject{}}}};
}

McpProtocol makeProtocol(QStringList* calls = nullptr) {
  return McpProtocol{
      [] {
        return QVector<CommandInfo>{fakeTool(QStringLiteral("echo")),
                                    fakeTool(QStringLiteral("image")),
                                    fakeTool(QStringLiteral("fail"))};
      },
      [calls](const QString& name, const QJsonObject& args) {
        if (calls)
          calls->append(name);
        CommandReply reply;
        if (name == QStringLiteral("echo")) {
          reply.ok = true;
          reply.message = QStringLiteral("echo ok");
          reply.data = QJsonObject{{QStringLiteral("echo"), args}};
        } else if (name == QStringLiteral("image")) {
          reply.ok = true;
          reply.message = QStringLiteral("image ok");
          reply.image = QByteArray("\x89PNG\r\n\x1a\nfake", 12);
        } else if (name == QStringLiteral("fail")) {
          reply.ok = false;
          reply.error = QStringLiteral("deliberate failure");
        } else {
          reply.ok = false;
          reply.error = QStringLiteral("unknown fake tool");
        }
        return reply;
      }};
}

QJsonObject request(const QString& method, const QJsonValue& id,
                    const QJsonObject& params = {}) {
  return QJsonObject{{QStringLiteral("jsonrpc"), QStringLiteral("2.0")},
                     {QStringLiteral("id"), id},
                     {QStringLiteral("method"), method},
                     {QStringLiteral("params"), params}};
}

} // namespace

class TstMcp : public QObject {
  Q_OBJECT
private slots:
  void initializeAdvertisesTools() {
    McpProtocol protocol = makeProtocol();
    const QJsonObject reply = protocol.handle(request(
        QStringLiteral("initialize"), 1,
        QJsonObject{{QStringLiteral("protocolVersion"),
                     QStringLiteral("2025-03-26")}}));
    const QJsonObject result = reply.value(QStringLiteral("result")).toObject();
    QCOMPARE(result.value(QStringLiteral("protocolVersion")).toString(),
             QStringLiteral("2025-03-26"));
    QCOMPARE(result.value(QStringLiteral("serverInfo"))
                 .toObject()
                 .value(QStringLiteral("name"))
                 .toString(),
             QStringLiteral("meshbench"));
    QCOMPARE(result.value(QStringLiteral("capabilities"))
                 .toObject()
                 .value(QStringLiteral("tools"))
                 .toObject()
                 .value(QStringLiteral("listChanged"))
                 .toBool(),
             false);
  }

  void toolsListExposesSchemas() {
    McpProtocol protocol = makeProtocol();
    const QJsonObject reply =
        protocol.handle(request(QStringLiteral("tools/list"), 2));
    const QJsonArray tools = reply.value(QStringLiteral("result"))
                                 .toObject()
                                 .value(QStringLiteral("tools"))
                                 .toArray();
    QCOMPARE(tools.size(), 3);
    const QJsonObject first = tools.at(0).toObject();
    QCOMPARE(first.value(QStringLiteral("name")).toString(),
             QStringLiteral("echo"));
    QCOMPARE(first.value(QStringLiteral("inputSchema"))
                 .toObject()
                 .value(QStringLiteral("type"))
                 .toString(),
             QStringLiteral("object"));
  }

  void toolsCallReturnsTextAndStructuredContent() {
    QStringList calls;
    McpProtocol protocol = makeProtocol(&calls);
    const QJsonObject reply = protocol.handle(request(
        QStringLiteral("tools/call"), 3,
        QJsonObject{{QStringLiteral("name"), QStringLiteral("echo")},
                    {QStringLiteral("arguments"),
                     QJsonObject{{QStringLiteral("value"), 42}}}}));
    QCOMPARE(calls, QStringList{QStringLiteral("echo")});
    const QJsonObject result = reply.value(QStringLiteral("result")).toObject();
    QCOMPARE(result.value(QStringLiteral("isError")).toBool(), false);
    const QJsonArray content = result.value(QStringLiteral("content")).toArray();
    QCOMPARE(content.size(), 1);
    QCOMPARE(content.at(0).toObject().value(QStringLiteral("type")).toString(),
             QStringLiteral("text"));
    QCOMPARE(content.at(0).toObject().value(QStringLiteral("text")).toString(),
             QStringLiteral("echo ok"));
    QCOMPARE(result.value(QStringLiteral("structuredContent"))
                 .toObject()
                 .value(QStringLiteral("echo"))
                 .toObject()
                 .value(QStringLiteral("value"))
                 .toInt(),
             42);
  }

  void toolsCallFailureSetsIsError() {
    McpProtocol protocol = makeProtocol();
    const QJsonObject reply = protocol.handle(request(
        QStringLiteral("tools/call"), 4,
        QJsonObject{{QStringLiteral("name"), QStringLiteral("fail")}}));
    const QJsonObject result = reply.value(QStringLiteral("result")).toObject();
    QCOMPARE(result.value(QStringLiteral("isError")).toBool(), true);
    QCOMPARE(result.value(QStringLiteral("content"))
                 .toArray()
                 .at(0)
                 .toObject()
                 .value(QStringLiteral("text"))
                 .toString(),
             QStringLiteral("deliberate failure"));
    QVERIFY(!result.contains(QStringLiteral("structuredContent")));
  }

  void imageToolReturnsImageContentBlock() {
    McpProtocol protocol = makeProtocol();
    const QJsonObject reply = protocol.handle(request(
        QStringLiteral("tools/call"), 5,
        QJsonObject{{QStringLiteral("name"), QStringLiteral("image")}}));
    const QJsonArray content = reply.value(QStringLiteral("result"))
                                   .toObject()
                                   .value(QStringLiteral("content"))
                                   .toArray();
    QCOMPARE(content.size(), 2);
    const QJsonObject image = content.at(1).toObject();
    QCOMPARE(image.value(QStringLiteral("type")).toString(),
             QStringLiteral("image"));
    QCOMPARE(image.value(QStringLiteral("mimeType")).toString(),
             QStringLiteral("image/png"));
    const QByteArray decoded = QByteArray::fromBase64(
        image.value(QStringLiteral("data")).toString().toLatin1());
    QCOMPARE(decoded, QByteArray("\x89PNG\r\n\x1a\nfake", 12));
  }

  void notificationsGetNoResponse() {
    McpProtocol protocol = makeProtocol();
    const QJsonObject reply = protocol.handle(QJsonObject{
        {QStringLiteral("jsonrpc"), QStringLiteral("2.0")},
        {QStringLiteral("method"), QStringLiteral("notifications/initialized")}});
    QVERIFY(reply.isEmpty());
  }

  void unknownMethodIsAnError() {
    McpProtocol protocol = makeProtocol();
    const QJsonObject reply =
        protocol.handle(request(QStringLiteral("nope/nope"), 6));
    QCOMPARE(reply.value(QStringLiteral("error"))
                 .toObject()
                 .value(QStringLiteral("code"))
                 .toInt(),
             -32601);
  }

  void missingToolNameIsInvalidParams() {
    McpProtocol protocol = makeProtocol();
    const QJsonObject reply =
        protocol.handle(request(QStringLiteral("tools/call"), 7));
    QCOMPARE(reply.value(QStringLiteral("error"))
                 .toObject()
                 .value(QStringLiteral("code"))
                 .toInt(),
             -32602);
  }

  void httpServerServesPostRequests() {
    McpHttpServer server;
    std::string error;
    const bool started = server.start(
        "127.0.0.1", 0,
        [](const McpHttpRequest& req) {
          McpHttpResponse response;
          response.contentType = "application/json";
          response.body = "{\"method\":\"" + req.method + "\",\"body\":\"" +
                          req.body + "\"}";
          return response;
        },
        error);
    QVERIFY2(started, error.c_str());
    QVERIFY(server.running());
    QVERIFY(server.port() != 0);

    asio::io_context ioc;
    tcp::resolver resolver(ioc);
    beast::tcp_stream stream(ioc);
    const auto endpoints =
        resolver.resolve("127.0.0.1", std::to_string(server.port()));
    stream.connect(endpoints);

    http::request<http::string_body> req{http::verb::post, "/mcp", 11};
    req.set(http::field::host, "127.0.0.1");
    req.set(http::field::content_type, "application/json");
    req.body() = "{\"hello\":true}";
    req.prepare_payload();
    http::write(stream, req);

    beast::flat_buffer buffer;
    http::response<http::string_body> res;
    http::read(stream, buffer, res);
    QCOMPARE(res.result_int(), 200);
    const std::string body = res.body();
    QVERIFY(body.find("\"method\":\"POST\"") != std::string::npos);
    QVERIFY(body.find("hello") != std::string::npos);

    beast::error_code ec;
    stream.socket().shutdown(tcp::socket::shutdown_both, ec);
    server.stop();
    QVERIFY(!server.running());
  }

  void httpServerRejectsBadBindAddress() {
    McpHttpServer server;
    std::string error;
    QVERIFY(!server.start("not an address", 0,
                          [](const McpHttpRequest&) { return McpHttpResponse{}; },
                          error));
    QVERIFY(!error.empty());
    QVERIFY(!server.running());
  }
};

QTEST_GUILESS_MAIN(TstMcp)
#include "tst_mcp.moc"
