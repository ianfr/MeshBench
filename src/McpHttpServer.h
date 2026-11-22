#pragma once

// Local, unauthenticated HTTP server for the MeshBench MCP endpoint.
//
// Bound to loopback by default. The server runs Boost.Beast in its own
// thread; the request handler is invoked on that thread, so callers that
// touch the GUI must marshal to the Qt main thread themselves (MainWindow
// does this with a queued invocation).

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>

namespace meshbench {

struct McpHttpRequest {
  std::string method;
  std::string target;
  std::string body;
  std::string contentType;
};

struct McpHttpResponse {
  int status = 200;
  std::string contentType = "application/json";
  std::string body;
  std::map<std::string, std::string> headers;
};

using McpHttpHandler = std::function<McpHttpResponse(const McpHttpRequest&)>;

class McpHttpServer {
public:
  McpHttpServer();
  ~McpHttpServer();

  McpHttpServer(const McpHttpServer&) = delete;
  McpHttpServer& operator=(const McpHttpServer&) = delete;

  // Bind `host`:port and start serving. Pass port 0 to pick an ephemeral
  // port (reported by port()). Returns false with a human-readable error on
  // bind/listen failure.
  bool start(const std::string& host, std::uint16_t port,
             McpHttpHandler handler, std::string& error);
  // Idempotent; joins the server thread.
  void stop();
  bool running() const;
  std::uint16_t port() const;

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace meshbench
