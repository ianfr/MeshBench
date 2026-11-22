#include "McpHttpServer.h"

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/system/error_code.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <utility>

namespace meshbench {

namespace beast = boost::beast;
namespace http = beast::http;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace {

constexpr std::size_t kMaxRequestBody = 32u * 1024u * 1024u;
constexpr auto kReadTimeout = std::chrono::seconds(30);

// One keep-alive HTTP connection. All work happens on the server's
// io_context thread; the handler is copied into the session.
class HttpSession : public std::enable_shared_from_this<HttpSession> {
public:
  HttpSession(tcp::socket&& socket, McpHttpHandler handler,
              std::shared_ptr<std::atomic<bool>> alive)
      : m_stream(std::move(socket)),
        m_handler(std::move(handler)),
        m_alive(std::move(alive)) {
    m_parser = std::make_unique<http::request_parser<http::string_body>>();
    m_parser->body_limit(kMaxRequestBody);
  }

  void run() {
    m_stream.expires_after(kReadTimeout);
    http::async_read(
        m_stream, m_buffer, *m_parser,
        beast::bind_front_handler(&HttpSession::onRead, shared_from_this()));
  }

private:
  void onRead(beast::error_code ec, std::size_t /*bytes*/) {
    if (ec)
      return; // timeout, malformed HTTP, or peer closed
    if (!m_alive || !m_alive->load())
      return;

    const http::request<http::string_body>& req = m_parser->get();
    McpHttpRequest in;
    in.method = std::string(req.method_string());
    in.target = std::string(req.target());
    in.body = req.body();
    in.contentType = std::string(req[http::field::content_type]);

    McpHttpResponse out;
    try {
      out = m_handler(in);
    } catch (...) {
      out.status = 500;
      out.contentType = "application/json";
      out.body = "{\"error\":\"internal server error\"}";
    }
    if (!m_alive || !m_alive->load())
      return;

    auto res = std::make_shared<http::response<http::string_body>>(
        static_cast<http::status>(out.status), req.version());
    res->set(http::field::server, "MeshBench-MCP");
    res->set(http::field::content_type, out.contentType);
    for (const auto& [name, value] : out.headers)
      res->set(name, value);
    res->keep_alive(req.keep_alive());
    res->body() = std::move(out.body);
    res->prepare_payload();

    // The handler runs synchronously and may take minutes (meshing, CAD
    // booleans). The read deadline must not abort the response afterwards.
    m_stream.expires_never();
    const bool close = !res->keep_alive();
    http::async_write(
        m_stream, *res,
        [self = shared_from_this(), res, close](beast::error_code writeEc,
                                                std::size_t /*bytes*/) {
          self->onWrite(close, writeEc);
        });
  }

  void onWrite(bool close, beast::error_code ec) {
    if (ec)
      return;
    if (close) {
      beast::error_code ignored;
      beast::get_lowest_layer(m_stream).socket().shutdown(
          tcp::socket::shutdown_send, ignored);
      return;
    }
    m_parser = std::make_unique<http::request_parser<http::string_body>>();
    m_parser->body_limit(kMaxRequestBody);
    run();
  }

  beast::tcp_stream m_stream;
  beast::flat_buffer m_buffer;
  std::unique_ptr<http::request_parser<http::string_body>> m_parser;
  McpHttpHandler m_handler;
  std::shared_ptr<std::atomic<bool>> m_alive;
};

} // namespace

struct McpHttpServer::Impl {
  asio::io_context io;
  std::unique_ptr<tcp::acceptor> acceptor;
  std::thread thread;
  McpHttpHandler handler;
  std::shared_ptr<std::atomic<bool>> alive =
      std::make_shared<std::atomic<bool>>(false);
  std::atomic<std::uint16_t> boundPort{0};

  void doAccept() {
    if (!acceptor || !alive->load())
      return;
    acceptor->async_accept(
        asio::make_strand(io),
        [this](beast::error_code ec, tcp::socket socket) {
          if (!ec && alive->load()) {
            std::make_shared<HttpSession>(std::move(socket), handler, alive)
                ->run();
          }
          doAccept();
        });
  }
};

McpHttpServer::McpHttpServer() : m_impl(std::make_unique<Impl>()) {}

McpHttpServer::~McpHttpServer() { stop(); }

bool McpHttpServer::start(const std::string& host, std::uint16_t port,
                          McpHttpHandler handler, std::string& error) {
  error.clear();
  if (!m_impl)
    m_impl = std::make_unique<Impl>();
  if (running()) {
    error = "MCP server is already running.";
    return false;
  }
  if (!handler) {
    error = "MCP server handler must not be empty.";
    return false;
  }
  if (host.empty()) {
    error = "MCP server host must not be empty.";
    return false;
  }

  beast::error_code ec;
  const asio::ip::address address = asio::ip::make_address(host, ec);
  if (ec) {
    error = "Invalid MCP bind address '" + host + "': " + ec.message();
    return false;
  }
  const tcp::endpoint endpoint(address, port);

  m_impl->handler = std::move(handler);
  // A stopped server leaves its io_context stopped; make start() reusable.
  m_impl->io.restart();
  m_impl->acceptor = std::make_unique<tcp::acceptor>(m_impl->io);
  m_impl->acceptor->open(endpoint.protocol(), ec);
  if (ec) {
    error = "MCP acceptor open failed: " + ec.message();
    m_impl->acceptor.reset();
    return false;
  }
  m_impl->acceptor->set_option(asio::socket_base::reuse_address(true), ec);
  if (ec) {
    error = "MCP reuse_address failed: " + ec.message();
    m_impl->acceptor.reset();
    return false;
  }
  m_impl->acceptor->bind(endpoint, ec);
  if (ec) {
    error = "MCP bind to " + host + ":" + std::to_string(port) +
            " failed: " + ec.message();
    m_impl->acceptor.reset();
    return false;
  }
  m_impl->acceptor->listen(asio::socket_base::max_listen_connections, ec);
  if (ec) {
    error = "MCP listen failed: " + ec.message();
    m_impl->acceptor.reset();
    return false;
  }
  const std::uint16_t bound =
      m_impl->acceptor->local_endpoint(ec).port();
  if (ec) {
    error = "MCP local_endpoint failed: " + ec.message();
    m_impl->acceptor.reset();
    return false;
  }
  m_impl->boundPort.store(bound);
  m_impl->alive->store(true);
  m_impl->doAccept();
  m_impl->thread = std::thread([impl = m_impl.get()] { impl->io.run(); });
  return true;
}

void McpHttpServer::stop() {
  if (!m_impl)
    return;
  m_impl->alive->store(false);
  if (m_impl->acceptor) {
    beast::error_code ignored;
    m_impl->acceptor->close(ignored);
    m_impl->acceptor.reset();
  }
  m_impl->io.stop();
  if (m_impl->thread.joinable())
    m_impl->thread.join();
  m_impl->boundPort.store(0);
}

bool McpHttpServer::running() const {
  return m_impl && m_impl->alive && m_impl->alive->load() &&
         m_impl->thread.joinable();
}

std::uint16_t McpHttpServer::port() const {
  return m_impl ? m_impl->boundPort.load() : 0;
}

} // namespace meshbench
