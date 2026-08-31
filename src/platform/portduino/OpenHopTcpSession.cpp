#include "OpenHopTcpSession.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility>

#ifdef _WIN32
#include <fcntl.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace openhop {

namespace {

#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket INVALID_NATIVE_SOCKET = INVALID_SOCKET;
int socketError() { return WSAGetLastError(); }
bool wouldBlock(int error) {
  return error == WSAEWOULDBLOCK || error == WSAEINPROGRESS;
}
void closeNativeSocket(NativeSocket socket) { closesocket(socket); }
#else
using NativeSocket = int;
constexpr NativeSocket INVALID_NATIVE_SOCKET = -1;
int socketError() { return errno; }
bool wouldBlock(int error) {
  return error == EAGAIN || error == EWOULDBLOCK || error == EINPROGRESS;
}
void closeNativeSocket(NativeSocket socket) { ::close(socket); }
#endif

bool setNonBlocking(NativeSocket socket) {
#ifdef _WIN32
  u_long enabled = 1;
  return ioctlsocket(socket, FIONBIO, &enabled) == 0;
#else
  const int flags = fcntl(socket, F_GETFL, 0);
  return flags >= 0 && fcntl(socket, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

bool waitForSocket(NativeSocket socket, bool write, uint32_t timeout_ms) {
  fd_set set;
  FD_ZERO(&set);
  FD_SET(socket, &set);
  timeval timeout;
  timeout.tv_sec = static_cast<long>(timeout_ms / 1000);
  timeout.tv_usec = static_cast<decltype(timeout.tv_usec)>(
      (timeout_ms % 1000u) * 1000u);
#ifdef _WIN32
  const int result = select(0, write ? nullptr : &set, write ? &set : nullptr,
                            nullptr, &timeout);
#else
  const int result = select(socket + 1, write ? nullptr : &set,
                            write ? &set : nullptr, nullptr, &timeout);
#endif
  return result > 0 && FD_ISSET(socket, &set);
}

} // namespace

TcpSession::TcpSession(TcpSessionConfig config) : config_(std::move(config)) {
  if (config_.port == 0)
    config_.port = 5055;
  if (config_.connect_timeout_ms == 0)
    config_.connect_timeout_ms = 5000;
  if (config_.reconnect_initial_ms == 0)
    config_.reconnect_initial_ms = 1000;
  if (config_.reconnect_max_ms < config_.reconnect_initial_ms)
    config_.reconnect_max_ms = config_.reconnect_initial_ms;
}

TcpSession::~TcpSession() { stop(); }

bool TcpSession::start() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (started_)
    return true;
  stopping_ = false;
  stop_requested_.store(false);
  started_ = true;
  worker_ = std::thread(&TcpSession::workerLoop, this);
  return true;
}

bool TcpSession::connect(uint32_t timeout_ms) {
  if (!start())
    return false;
  if (timeout_ms == 0)
    timeout_ms = config_.connect_timeout_ms;
  return waitUntilConnected(timeout_ms);
}

void TcpSession::stop() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!started_ && !worker_.joinable())
      return;
    stopping_ = true;
    stop_requested_.store(true);
  }
  closeSocket();
  state_cv_.notify_all();
  request_cv_.notify_all();
  if (worker_.joinable())
    worker_.join();
  std::lock_guard<std::mutex> lock(state_mutex_);
  started_ = false;
  connected_ = false;
  stopping_ = false;
}

void TcpSession::disconnect() {
  closeSocket();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    connected_ = false;
  }
  state_cv_.notify_all();
  request_cv_.notify_all();
}

void TcpSession::reconnect() { disconnect(); }

bool TcpSession::waitUntilConnected(uint32_t timeout_ms) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  return state_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                            [this] {
                              return connected_ || stopping_ ||
                                     stop_requested_.load();
                            }) &&
         connected_;
}

bool TcpSession::isConnected() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connected_;
}

void TcpSession::setRxPacketCallback(RxPacketCallback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  rx_callback_ = std::move(callback);
}

bool TcpSession::setRadioConfig(const RadioConfig &config) {
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    radio_config_ = config;
    has_radio_config_ = true;
  }
  if (!isConnected())
    return true;
  std::vector<uint8_t> response;
  return request(CMD_SET_CONFIG, encodeRadioConfig(config), CMD_CONFIG_RESP,
                 response);
}

bool TcpSession::request(uint8_t command, const std::vector<uint8_t> &payload,
                         uint8_t expected_command,
                         std::vector<uint8_t> &response, uint32_t timeout_ms) {
  if (isWorkerThread())
    return false;

  std::unique_lock<std::mutex> request_lock(request_mutex_);
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (!connected_ || stopping_ || stop_requested_.load())
      return false;
  }

  std::vector<uint8_t> frame;
  if (!buildFrame(command, payload, frame))
    return false;

  request_pending_ = true;
  request_success_ = false;
  request_expected_ = expected_command;
  request_response_.clear();
  if (!sendFrame(frame)) {
    request_pending_ = false;
    request_lock.unlock();
    markDisconnected();
    return false;
  }

  const bool completed = request_cv_.wait_for(
      request_lock, std::chrono::milliseconds(timeout_ms), [this] {
        return !request_pending_ || !isConnected() || stop_requested_.load();
      });
  if (!completed || request_pending_) {
    request_pending_ = false;
    // There is no request ID in the protocol. Reset the stream on a
    // timeout so a late response cannot satisfy the next request.
    disconnect();
    return false;
  }
  response = request_response_;
  return request_success_;
}

void TcpSession::workerLoop() {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    worker_thread_id_ = std::this_thread::get_id();
  }
  uint32_t retry_ms = config_.reconnect_initial_ms;
  FrameParser parser;
  while (!stop_requested_.load()) {
    if (!isConnected()) {
      if (!connectAndHandshake()) {
        std::unique_lock<std::mutex> lock(state_mutex_);
        state_cv_.wait_for(lock, std::chrono::milliseconds(retry_ms), [this] {
          return stop_requested_.load() || connected_;
        });
        const uint32_t next_retry = retry_ms > config_.reconnect_max_ms / 2
                                        ? config_.reconnect_max_ms
                                        : retry_ms * 2;
        retry_ms = std::min(config_.reconnect_max_ms,
                            std::max(config_.reconnect_initial_ms, next_retry));
        continue;
      }
      retry_ms = config_.reconnect_initial_ms;
      parser.reset();
    }

    uint8_t data[4096];
    Socket socket = -1;
    NativeSocket native_socket = INVALID_NATIVE_SOCKET;
    std::size_t received_size = 0;
    bool socket_failed = false;
    {
      std::lock_guard<std::mutex> io_lock(socket_io_mutex_);
      socket = currentSocket();
      if (socket < 0) {
        // The socket may have been closed by a request timeout or stop().
        // Leave the I/O critical section before attempting reconnection.
        continue;
      }
      native_socket = static_cast<NativeSocket>(socket);
      if (!waitForSocket(native_socket, false, 100))
        continue;
#ifdef _WIN32
      const int received = recv(native_socket, reinterpret_cast<char *>(data),
                                sizeof(data), 0);
#else
      const ssize_t received = recv(native_socket, data, sizeof(data), 0);
#endif
      if (received <= 0) {
        socket_failed = true;
      } else {
        received_size = static_cast<std::size_t>(received);
      }
    }
    if (socket_failed) {
      markDisconnected();
      continue;
    }
    if (received_size == 0) {
      markDisconnected();
      continue;
    }
    parser.feed(data, received_size,
                [this](const Frame &frame) { processRuntimeFrame(frame); });
  }
  disconnect();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    worker_thread_id_ = std::thread::id();
  }
}

bool TcpSession::connectAndHandshake() {
  if (!openSocket())
    return false;

  std::vector<uint8_t> response;
  if (!config_.token.empty() &&
      !handshakeCommand(
          CMD_AUTH,
          std::vector<uint8_t>(config_.token.begin(), config_.token.end()),
          CMD_AUTH_OK, response, config_.connect_timeout_ms)) {
    closeSocket();
    return false;
  }
  response.clear();
  if (!handshakeCommand(CMD_PING, {}, CMD_PONG, response,
                        config_.connect_timeout_ms)) {
    closeSocket();
    return false;
  }

  RadioConfig radio_config;
  bool has_config = false;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    has_config = has_radio_config_;
    radio_config = radio_config_;
  }
  if (has_config) {
    response.clear();
    if (!handshakeCommand(CMD_SET_CONFIG, encodeRadioConfig(radio_config),
                          CMD_CONFIG_RESP, response,
                          config_.connect_timeout_ms)) {
      closeSocket();
      return false;
    }
  }
  response.clear();
  if (!handshakeCommand(CMD_RX_START, {}, CMD_RX_STARTED, response,
                        config_.connect_timeout_ms)) {
    closeSocket();
    return false;
  }
  if (stop_requested_.load()) {
    closeSocket();
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    connected_ = true;
  }
  state_cv_.notify_all();
  return true;
}

bool TcpSession::handshakeCommand(uint8_t command,
                                  const std::vector<uint8_t> &payload,
                                  uint8_t expected_command,
                                  std::vector<uint8_t> &response,
                                  uint32_t timeout_ms) {
  std::vector<uint8_t> frame;
  if (!buildFrame(command, payload, frame) || !sendFrame(frame))
    return false;

  bool expected_received = false;
  bool request_failed = false;
  FrameParser parser;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (!expected_received && !request_failed && !stop_requested_.load()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline)
      return false;
    const auto remaining = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
            .count());
    uint8_t data[4096];
    Socket handshake_socket = -1;
    std::size_t received_size = 0;
    {
      std::lock_guard<std::mutex> io_lock(socket_io_mutex_);
      handshake_socket = currentSocket();
      if (handshake_socket < 0 ||
          !waitForSocket(static_cast<NativeSocket>(handshake_socket), false,
                         remaining))
        return false;
#ifdef _WIN32
    const int received = recv(static_cast<NativeSocket>(handshake_socket),
                              reinterpret_cast<char *>(data), sizeof(data), 0);
#else
    const ssize_t received =
        recv(static_cast<NativeSocket>(handshake_socket), data, sizeof(data), 0);
#endif
      if (received <= 0)
        return false;
      received_size = static_cast<std::size_t>(received);
    }
    parser.feed(data, received_size,
                [&](const Frame &received_frame) {
                  if (received_frame.command == CMD_RX_PACKET) {
                    processRuntimeFrame(received_frame);
                  } else if (received_frame.command == expected_command) {
                    response = received_frame.payload;
                    expected_received = true;
                  } else if (received_frame.command == CMD_ERROR ||
                             received_frame.command == CMD_TX_FAIL) {
                    request_failed = true;
                  }
                });
  }
  return expected_received && !request_failed;
}

bool TcpSession::sendFrame(const std::vector<uint8_t> &frame) {
  return sendAll(frame.data(), frame.size());
}

bool TcpSession::sendAll(const uint8_t *data, std::size_t length) {
  std::lock_guard<std::mutex> io_lock(socket_io_mutex_);
  const Socket socket = currentSocket();
  if (socket < 0)
    return false;
  std::size_t sent = 0;
  while (sent < length && !stop_requested_.load()) {
    if (!waitForSocket(static_cast<NativeSocket>(socket), true,
                       config_.connect_timeout_ms))
      return false;
#ifdef _WIN32
    const int count = send(static_cast<NativeSocket>(socket),
                           reinterpret_cast<const char *>(data + sent),
                           static_cast<int>(length - sent), 0);
#else
    int flags = 0;
#ifdef MSG_NOSIGNAL
    flags |= MSG_NOSIGNAL;
#endif
    const ssize_t count = send(static_cast<NativeSocket>(socket), data + sent,
                               length - sent, flags);
#endif
    if (count > 0) {
      sent += static_cast<std::size_t>(count);
    } else if (count < 0 && wouldBlock(socketError())) {
      continue;
    } else {
      return false;
    }
  }
  return sent == length;
}

void TcpSession::processRuntimeFrame(const Frame &frame) {
  if (frame.command == CMD_RX_PACKET) {
    RxPacket packet;
    if (!decodeRxPacket(frame.payload, packet))
      return;
    RxPacketCallback callback;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback = rx_callback_;
    }
    if (callback)
      callback(packet);
    return;
  }

  std::lock_guard<std::mutex> lock(request_mutex_);
  if (!request_pending_)
    return;
  if (frame.command == request_expected_) {
    request_response_ = frame.payload;
    request_success_ = true;
    request_pending_ = false;
    request_cv_.notify_all();
  } else if (frame.command == CMD_ERROR || (frame.command == CMD_TX_FAIL &&
                                            request_expected_ == CMD_TX_DONE)) {
    request_success_ = false;
    request_pending_ = false;
    request_cv_.notify_all();
  }
}

void TcpSession::markDisconnected() {
  {
    std::lock_guard<std::mutex> request_lock(request_mutex_);
    if (request_pending_) {
      request_success_ = false;
      request_pending_ = false;
    }
  }
  closeSocket();
  state_cv_.notify_all();
  request_cv_.notify_all();
}

void TcpSession::closeSocket() {
  std::lock_guard<std::mutex> io_lock(socket_io_mutex_);
  Socket socket = -1;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    socket = socket_;
    socket_ = -1;
    connected_ = false;
  }
  if (socket >= 0)
    closeNativeSocket(static_cast<NativeSocket>(socket));
}

TcpSession::Socket TcpSession::currentSocket() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return socket_;
}

bool TcpSession::openSocket() {
  if (config_.host.empty())
    return false;

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  const std::string service = std::to_string(config_.port);
  addrinfo *addresses = nullptr;
  if (getaddrinfo(config_.host.c_str(), service.c_str(), &hints, &addresses) !=
      0)
    return false;

  NativeSocket connected_socket = INVALID_NATIVE_SOCKET;
  for (addrinfo *address = addresses; address != nullptr;
       address = address->ai_next) {
    NativeSocket candidate = static_cast<NativeSocket>(
        socket(address->ai_family, address->ai_socktype, address->ai_protocol));
    if (candidate == INVALID_NATIVE_SOCKET)
      continue;
    if (!setNonBlocking(candidate)) {
      closeNativeSocket(candidate);
      continue;
    }
#ifdef SO_NOSIGPIPE
    int no_sigpipe = 1;
    setsockopt(candidate, SOL_SOCKET, SO_NOSIGPIPE, &no_sigpipe,
               sizeof(no_sigpipe));
#endif
    int no_delay = 1;
#ifdef _WIN32
    setsockopt(candidate, IPPROTO_TCP, TCP_NODELAY,
               reinterpret_cast<const char *>(&no_delay), sizeof(no_delay));
#else
    setsockopt(candidate, IPPROTO_TCP, TCP_NODELAY, &no_delay,
               sizeof(no_delay));
#endif
#ifdef _WIN32
    const int result = ::connect(candidate, address->ai_addr,
                                 static_cast<int>(address->ai_addrlen));
#else
    const int result = ::connect(candidate, address->ai_addr,
                                 address->ai_addrlen);
#endif
    if (result == 0 || (result < 0 && wouldBlock(socketError()))) {
      if (result == 0 ||
          waitForSocket(candidate, true, config_.connect_timeout_ms)) {
        int socket_error = 0;
#ifdef _WIN32
        int error_length = sizeof(socket_error);
#else
        socklen_t error_length = sizeof(socket_error);
#endif
        getsockopt(candidate, SOL_SOCKET, SO_ERROR,
                   reinterpret_cast<char *>(&socket_error), &error_length);
        if (socket_error == 0) {
          connected_socket = candidate;
          break;
        }
      }
    }
    closeNativeSocket(candidate);
  }
  freeaddrinfo(addresses);
  if (connected_socket == INVALID_NATIVE_SOCKET)
    return false;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (stop_requested_.load()) {
      closeNativeSocket(connected_socket);
      return false;
    }
    socket_ = static_cast<Socket>(connected_socket);
  }
  return true;
}

bool TcpSession::isWorkerThread() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return worker_thread_id_ == std::this_thread::get_id();
}

} // namespace openhop
