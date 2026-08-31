#pragma once

// Standalone openHop modem wire codec. This deliberately has no dependency
// on MeshService, RadioInterface, or the Portduino configuration layer so it
// can be tested and reused by the eventual OpenHopRadio implementation.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace openhop {

constexpr uint8_t PROTO_SYNC = 0xAA;
constexpr std::size_t MAX_LORA_PAYLOAD = 255;
constexpr std::size_t RX_PACKET_METADATA_SIZE = 6;
constexpr std::size_t MAX_FRAME_PAYLOAD =
    MAX_LORA_PAYLOAD + RX_PACKET_METADATA_SIZE;

// Host -> modem commands.
constexpr uint8_t CMD_TX_REQUEST = 0x01;
constexpr uint8_t CMD_SET_CONFIG = 0x10;
constexpr uint8_t CMD_GET_CONFIG = 0x11;
constexpr uint8_t CMD_STATUS_REQ = 0x20;
constexpr uint8_t CMD_NOISE_REQ = 0x22;
constexpr uint8_t CMD_CAD_REQUEST = 0x30;
constexpr uint8_t CMD_RX_START = 0x31;
constexpr uint8_t CMD_SET_CAD_PARAMS = 0x34;
constexpr uint8_t CMD_SET_WIFI = 0x41;
constexpr uint8_t CMD_AUTH = 0x50;
constexpr uint8_t CMD_WIFI_RESET = 0x60;
constexpr uint8_t CMD_GET_WIFI = 0x61;
constexpr uint8_t CMD_GET_VERSION = 0x70;
constexpr uint8_t CMD_PING = 0xFF;

// Modem -> host commands.
constexpr uint8_t CMD_TX_DONE = 0x02;
constexpr uint8_t CMD_TX_FAIL = 0x03;
constexpr uint8_t CMD_RX_PACKET = 0x04;
constexpr uint8_t CMD_CONFIG_RESP = 0x12;
constexpr uint8_t CMD_STATUS_RESP = 0x21;
constexpr uint8_t CMD_NOISE_RESP = 0x23;
constexpr uint8_t CMD_CAD_RESP = 0x32;
constexpr uint8_t CMD_RX_STARTED = 0x33;
constexpr uint8_t CMD_CAD_PARAMS_RESP = 0x35;
constexpr uint8_t CMD_AUTH_OK = 0x51;
constexpr uint8_t CMD_WIFI_STATUS = 0x62;
constexpr uint8_t CMD_VERSION_RESP = 0x71;
constexpr uint8_t CMD_ERROR = 0xFE;
constexpr uint8_t CMD_PONG = 0xFF;

// CMD_ERROR payload[0].
constexpr uint8_t ERR_CRC_MISMATCH = 0x01;
constexpr uint8_t ERR_INVALID_CMD = 0x02;
constexpr uint8_t ERR_RADIO_BUSY = 0x03;
constexpr uint8_t ERR_TX_TIMEOUT = 0x04;
constexpr uint8_t ERR_PAYLOAD_TOO_BIG = 0x05;
constexpr uint8_t ERR_INVALID_CONFIG = 0x06;
constexpr uint8_t ERR_CAD_FAILED = 0x07;
constexpr uint8_t ERR_RADIO_INIT = 0x08;
constexpr uint8_t ERR_UNAUTHORIZED = 0x09;

// CMD_WIFI_STATUS mode codes retained for wire-constant parity.
constexpr uint8_t WIFI_MODE_OFFLINE = 0;
constexpr uint8_t WIFI_MODE_STA_CONNECTING = 1;
constexpr uint8_t WIFI_MODE_STA_CONNECTED = 2;
constexpr uint8_t WIFI_MODE_AP_CONFIG = 3;

// Packed layouts from protocol_constants.py. Use explicit codec functions
// below rather than relying on host struct packing or host endianness.
struct RadioConfig {
  uint32_t freq_hz = 0;
  uint32_t bandwidth_hz = 0;
  uint8_t sf = 0;
  uint8_t cr = 0;
  int8_t power_dbm = 0;
  uint16_t syncword = 0;
  uint8_t preamble_len = 0;
};

bool validateRadioConfig(const RadioConfig &config);
bool makeRadioConfig(float frequency_mhz, float bandwidth_khz, uint8_t sf,
                     uint8_t cr, int8_t power_dbm, uint16_t syncword,
                     uint16_t preamble_len, RadioConfig &config);

inline bool isValidAirPacketLength(std::size_t length) {
  return length > 0 && length <= MAX_LORA_PAYLOAD;
}

struct StatusResp {
  uint32_t uptime_sec = 0;
  uint32_t rx_count = 0;
  uint32_t tx_count = 0;
  uint32_t crc_errors = 0;
  int16_t last_rssi = 0;
  int16_t last_snr_x10 = 0;
  int16_t noise_floor_x10 = 0;
  int8_t temp_c = 0;
  uint8_t radio_state = 0;
};

struct Frame {
  uint8_t command = 0;
  std::vector<uint8_t> payload;
};

uint16_t crc16Ccitt(const uint8_t *data, std::size_t length);
uint16_t crc16Ccitt(const std::vector<uint8_t> &data);

// Returns false when payload length is greater than the u16 wire field.
bool buildFrame(uint8_t command, const uint8_t *payload,
                std::size_t payload_length, std::vector<uint8_t> &frame);
bool buildFrame(uint8_t command, const std::vector<uint8_t> &payload,
                std::vector<uint8_t> &frame);

constexpr std::size_t RADIO_CONFIG_SIZE = 14;
constexpr std::size_t STATUS_RESP_SIZE = 24;
std::vector<uint8_t> encodeRadioConfig(const RadioConfig &config);
bool decodeRadioConfig(const uint8_t *payload, std::size_t length,
                       RadioConfig &config);
bool decodeStatusResp(const uint8_t *payload, std::size_t length,
                      StatusResp &status);

struct RxPacket {
  int16_t rssi = 0;
  int16_t snr_x10 = 0;
  int16_t signal_rssi = 0;
  std::vector<uint8_t> data;
};
bool decodeRxPacket(const std::vector<uint8_t> &payload, RxPacket &packet);

// Incremental parser for TCP/serial streams. It retains partial frames,
// emits every complete coalesced frame, bounds allocations, and drops one
// byte at a time after malformed input so an embedded sync byte can recover.
class FrameParser {
public:
  using FrameCallback = std::function<void(const Frame &)>;

  explicit FrameParser(std::size_t max_payload = MAX_FRAME_PAYLOAD);

  void reset();
  void feed(const uint8_t *data, std::size_t length,
            const FrameCallback &callback);
  void feed(const std::vector<uint8_t> &data, const FrameCallback &callback);
  std::size_t bufferedBytes() const { return buffer_.size(); }
  std::size_t rejectedFrames() const { return rejected_frames_; }

private:
  void parse(const FrameCallback &callback);

  std::vector<uint8_t> buffer_;
  std::size_t max_payload_;
  std::size_t rejected_frames_ = 0;
};

} // namespace openhop
