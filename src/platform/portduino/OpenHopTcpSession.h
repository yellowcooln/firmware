#pragma once

#include "OpenHopProtocol.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace openhop {

struct TcpSessionConfig {
  std::string host;
  uint16_t port = 5055;
  std::string token;
  uint32_t connect_timeout_ms = 5000;
  uint32_t reconnect_initial_ms = 1000;
  uint32_t reconnect_max_ms = 30000;
};

// A transport-only, worker-driven session for the openHop modem protocol.
// It owns one reader and serializes all request/response exchanges because
// the wire protocol has no request identifier. It intentionally does not
// implement RadioInterface or OpenHopRadio.
class TcpSession {
public:
  using RxPacketCallback = std::function<void(const RxPacket &)>;

  explicit TcpSession(TcpSessionConfig config);
  ~TcpSession();

  TcpSession(const TcpSession &) = delete;
  TcpSession &operator=(const TcpSession &) = delete;

  // Starts the worker and returns after the worker has been created. The
  // first connection attempt and all reconnects happen asynchronously.
  bool start();
  // Convenience form for callers that need an initial connection before
  // continuing. Later reconnects remain worker-driven.
  bool connect(uint32_t timeout_ms = 0);
  void stop();
  void disconnect();
  void reconnect();
  bool waitUntilConnected(uint32_t timeout_ms);
  bool isConnected() const;

  void setRxPacketCallback(RxPacketCallback callback);

  // SET_CONFIG is replayed after every successful reconnect. If already
  // connected, this also sends the new config immediately.
  bool setRadioConfig(const RadioConfig &config);

  // Sends one command and waits for its matching response. Only one call
  // may be active at a time; this is intentional and required by the
  // protocol's response-less request framing.
  bool request(uint8_t command, const std::vector<uint8_t> &payload,
               uint8_t expected_command, std::vector<uint8_t> &response,
               uint32_t timeout_ms = 5000);

private:
  using Socket = std::intptr_t;

  void workerLoop();
  bool connectAndHandshake();
  bool handshakeCommand(uint8_t command, const std::vector<uint8_t> &payload,
                        uint8_t expected_command,
                        std::vector<uint8_t> &response, uint32_t timeout_ms);
  bool sendFrame(const std::vector<uint8_t> &frame);
  bool sendAll(const uint8_t *data, std::size_t length);
  void processRuntimeFrame(const Frame &frame);
  void markDisconnected();
  void closeSocket();
  Socket currentSocket() const;
  bool openSocket();
  bool isWorkerThread() const;

  TcpSessionConfig config_;
  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;
  Socket socket_ = -1;
  bool connected_ = false;
  bool stopping_ = false;
  bool started_ = false;
  std::thread::id worker_thread_id_;

  // Serializes all socket operations with closeSocket(). Without this lock,
  // a reconnect can reuse a file descriptor while an in-flight send/recv is
  // still using the old descriptor.
  mutable std::mutex socket_io_mutex_;

  std::mutex request_mutex_;
  std::condition_variable request_cv_;
  bool request_pending_ = false;
  bool request_success_ = false;
  uint8_t request_expected_ = 0;
  std::vector<uint8_t> request_response_;

  mutable std::mutex callback_mutex_;
  RxPacketCallback rx_callback_;

  mutable std::mutex config_mutex_;
  bool has_radio_config_ = false;
  RadioConfig radio_config_;

  std::thread worker_;
  std::atomic<bool> stop_requested_{false};
};

using OpenHopTcpSession = TcpSession;

} // namespace openhop
