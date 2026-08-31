#include "OpenHopProtocol.h"

#include <algorithm>
#include <cmath>

namespace openhop {

namespace {

constexpr float SUPPORTED_BANDWIDTHS_KHZ[] = {
    7.8f, 10.4f, 15.6f, 20.8f, 31.25f, 41.7f, 62.5f,
    125.0f, 203.125f, 250.0f, 406.25f, 500.0f, 812.5f, 1625.0f,
};

bool closeEnough(float lhs, float rhs) {
  return std::fabs(lhs - rhs) <= std::max(0.01f, rhs * 0.0001f);
}

bool isSupportedBandwidth(uint32_t bandwidth_hz) {
  for (float bandwidth_khz : SUPPORTED_BANDWIDTHS_KHZ) {
    if (closeEnough(static_cast<float>(bandwidth_hz) / 1000.0f,
                    bandwidth_khz))
      return true;
  }
  return false;
}

uint16_t readU16(const uint8_t *data) {
  const uint16_t low = data[0];
  const uint16_t high = data[1];
  return static_cast<uint16_t>(low | static_cast<uint16_t>(high << 8));
}

uint32_t readU32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int16_t readI16(const uint8_t *data) {
  return static_cast<int16_t>(readU16(data));
}

void writeU16(std::vector<uint8_t> &out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>(value >> 8));
}

void writeU32(std::vector<uint8_t> &out, uint32_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>(value >> 24));
}

} // namespace

bool validateRadioConfig(const RadioConfig &config) {
  // openHop modem currently drives SX126x-class sub-GHz hardware. Keep
  // checks here stricter than generic wire-field ranges.
  return config.freq_hz >= 150000000u && config.freq_hz <= 960000000u &&
         isSupportedBandwidth(config.bandwidth_hz) && config.sf >= 5 &&
         config.sf <= 12 && config.cr >= 5 && config.cr <= 8 &&
         config.power_dbm >= -9 && config.power_dbm <= 22 &&
         config.syncword != 0 && config.preamble_len != 0;
}

bool makeRadioConfig(float frequency_mhz, float bandwidth_khz, uint8_t sf,
                     uint8_t cr, int8_t power_dbm, uint16_t syncword,
                     uint16_t preamble_len, RadioConfig &config) {
  if (!std::isfinite(frequency_mhz) || !std::isfinite(bandwidth_khz) ||
      frequency_mhz <= 0.0f || bandwidth_khz <= 0.0f || preamble_len > 255)
    return false;

  // Meshtastic stores MHz as float. Round at kHz precision first so values
  // such as 869.618f do not serialize as 869617981 Hz due to binary float
  // representation.
  const double frequency_hz =
      std::llround(static_cast<double>(frequency_mhz) * 1000.0) * 1000.0;
  const double bandwidth_hz = static_cast<double>(bandwidth_khz) * 1000.0;
  if (frequency_hz < 0.0 || frequency_hz > 4294967295.0 ||
      bandwidth_hz < 0.0 || bandwidth_hz > 4294967295.0)
    return false;

  RadioConfig candidate;
  candidate.freq_hz = static_cast<uint32_t>(std::llround(frequency_hz));
  candidate.bandwidth_hz = static_cast<uint32_t>(std::llround(bandwidth_hz));
  candidate.sf = sf;
  candidate.cr = cr;
  candidate.power_dbm = power_dbm;
  candidate.syncword = syncword;
  candidate.preamble_len = static_cast<uint8_t>(preamble_len);
  if (!validateRadioConfig(candidate))
    return false;
  config = candidate;
  return true;
}

uint16_t crc16Ccitt(const uint8_t *data, std::size_t length) {
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

uint16_t crc16Ccitt(const std::vector<uint8_t> &data) {
  return crc16Ccitt(data.data(), data.size());
}

bool buildFrame(uint8_t command, const uint8_t *payload,
                std::size_t payload_length, std::vector<uint8_t> &frame) {
  if (payload_length > 0xFFFF ||
      (payload_length != 0 && payload == nullptr))
    return false;

  frame.clear();
  frame.reserve(1 + 1 + 2 + payload_length + 2);
  frame.push_back(PROTO_SYNC);
  frame.push_back(command);
  frame.push_back(static_cast<uint8_t>(payload_length & 0xFF));
  frame.push_back(static_cast<uint8_t>(payload_length >> 8));
  if (payload_length != 0)
    frame.insert(frame.end(), payload, payload + payload_length);
  const uint16_t crc = crc16Ccitt(frame.data() + 1, frame.size() - 1);
  writeU16(frame, crc);
  return true;
}

bool buildFrame(uint8_t command, const std::vector<uint8_t> &payload,
                std::vector<uint8_t> &frame) {
  return buildFrame(command, payload.data(), payload.size(), frame);
}

std::vector<uint8_t> encodeRadioConfig(const RadioConfig &config) {
  std::vector<uint8_t> payload;
  payload.reserve(RADIO_CONFIG_SIZE);
  writeU32(payload, config.freq_hz);
  writeU32(payload, config.bandwidth_hz);
  payload.push_back(config.sf);
  payload.push_back(config.cr);
  payload.push_back(static_cast<uint8_t>(config.power_dbm));
  writeU16(payload, config.syncword);
  payload.push_back(config.preamble_len);
  return payload;
}

bool decodeRadioConfig(const uint8_t *payload, std::size_t length,
                       RadioConfig &config) {
  if (payload == nullptr || length < RADIO_CONFIG_SIZE)
    return false;
  config.freq_hz = readU32(payload);
  config.bandwidth_hz = readU32(payload + 4);
  config.sf = payload[8];
  config.cr = payload[9];
  config.power_dbm = static_cast<int8_t>(payload[10]);
  config.syncword = readU16(payload + 11);
  config.preamble_len = payload[13];
  return true;
}

bool decodeStatusResp(const uint8_t *payload, std::size_t length,
                      StatusResp &status) {
  if (payload == nullptr || length < STATUS_RESP_SIZE)
    return false;
  status.uptime_sec = readU32(payload);
  status.rx_count = readU32(payload + 4);
  status.tx_count = readU32(payload + 8);
  status.crc_errors = readU32(payload + 12);
  status.last_rssi = readI16(payload + 16);
  status.last_snr_x10 = readI16(payload + 18);
  status.noise_floor_x10 = readI16(payload + 20);
  status.temp_c = static_cast<int8_t>(payload[22]);
  status.radio_state = payload[23];
  return true;
}

bool decodeRxPacket(const std::vector<uint8_t> &payload, RxPacket &packet) {
  if (payload.size() < RX_PACKET_METADATA_SIZE)
    return false;
  packet.rssi = readI16(payload.data());
  packet.snr_x10 = readI16(payload.data() + 2);
  packet.signal_rssi = readI16(payload.data() + 4);
  packet.data.assign(payload.begin() + RX_PACKET_METADATA_SIZE, payload.end());
  return true;
}

FrameParser::FrameParser(std::size_t max_payload)
    : max_payload_(std::min<std::size_t>(max_payload, 0xFFFF)) {
  // A zero bound would otherwise make every frame invalid while still
  // allowing unbounded buffering. Keep the class bounded and useful.
  if (max_payload_ == 0)
    max_payload_ = MAX_FRAME_PAYLOAD;
  buffer_.reserve(1 + 1 + 2 + max_payload_ + 2);
}

void FrameParser::reset() {
  buffer_.clear();
  rejected_frames_ = 0;
}

void FrameParser::feed(const uint8_t *data, std::size_t length,
                       const FrameCallback &callback) {
  if (data == nullptr || length == 0)
    return;
  // Parse in bounded chunks. This preserves every frame in a large
  // coalesced recv() while preventing a peer from making the parser grow
  // without limit.
  const std::size_t max_buffer = 1 + 1 + 2 + max_payload_ + 2;
  while (length != 0) {
    if (buffer_.size() == max_buffer) {
      parse(callback);
      if (buffer_.size() == max_buffer) {
        ++rejected_frames_;
        buffer_.erase(buffer_.begin());
      }
    }
    const std::size_t room = max_buffer - buffer_.size();
    const std::size_t chunk_length = std::min(room, length);
    buffer_.insert(buffer_.end(), data, data + chunk_length);
    data += chunk_length;
    length -= chunk_length;
    parse(callback);
  }
}

void FrameParser::feed(const std::vector<uint8_t> &data,
                       const FrameCallback &callback) {
  feed(data.data(), data.size(), callback);
}

void FrameParser::parse(const FrameCallback &callback) {
  const std::size_t minimum_frame = 1 + 1 + 2 + 2;
  while (buffer_.size() >= minimum_frame) {
    auto sync = std::find(buffer_.begin(), buffer_.end(), PROTO_SYNC);
    if (sync == buffer_.end()) {
      buffer_.clear();
      return;
    }
    if (sync != buffer_.begin())
      buffer_.erase(buffer_.begin(), sync);
    if (buffer_.size() < minimum_frame)
      return;

    const uint16_t length = readU16(buffer_.data() + 2);
    if (length > max_payload_) {
      ++rejected_frames_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t frame_size = 1 + 1 + 2 + length + 2;
    if (buffer_.size() < frame_size)
      return;

    const uint16_t expected_crc = crc16Ccitt(buffer_.data() + 1, 3 + length);
    const uint16_t received_crc = readU16(buffer_.data() + 4 + length);
    if (expected_crc != received_crc) {
      ++rejected_frames_;
      // Drop only SYNC. A valid frame may begin inside corrupted input.
      buffer_.erase(buffer_.begin());
      continue;
    }

    Frame frame;
    frame.command = buffer_[1];
    frame.payload.assign(buffer_.begin() + 4, buffer_.begin() + 4 + length);
    buffer_.erase(
        buffer_.begin(),
        buffer_.begin() + static_cast<std::vector<uint8_t>::difference_type>(
                               frame_size));
    if (callback)
      callback(frame);
  }
}

} // namespace openhop
