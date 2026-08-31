#include "OpenHopProtocol.h"

#include <cstring>
#include <unity.h>

using namespace openhop;

void setUp(void) {}
void tearDown(void) {}

void test_crc_matches_ccitt_false_vector() {
  const uint8_t vector[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16Ccitt(vector, sizeof(vector)));
}

void test_codec_matches_wire_layout() {
  const std::vector<uint8_t> payload = {0x10, 0xAA, 0x00};
  std::vector<uint8_t> frame;
  TEST_ASSERT_TRUE(buildFrame(CMD_SET_CONFIG, payload, frame));
  TEST_ASSERT_EQUAL_UINT(1 + 1 + 2 + payload.size() + 2, frame.size());
  TEST_ASSERT_EQUAL_HEX8(PROTO_SYNC, frame[0]);
  TEST_ASSERT_EQUAL_HEX8(CMD_SET_CONFIG, frame[1]);
  TEST_ASSERT_EQUAL_HEX8(3, frame[2]);
  TEST_ASSERT_EQUAL_HEX8(0, frame[3]);
  TEST_ASSERT_EQUAL_HEX16(crc16Ccitt(frame.data() + 1, 6),
                          static_cast<uint16_t>(frame[7]) |
                              (static_cast<uint16_t>(frame[8]) << 8));
}

void test_build_frame_rejects_nonempty_null_payload() {
  std::vector<uint8_t> frame = {0xA5};
  TEST_ASSERT_FALSE(buildFrame(CMD_PING, nullptr, 1, frame));
  TEST_ASSERT_EQUAL_UINT(1, frame.size());
  TEST_ASSERT_EQUAL_HEX8(0xA5, frame[0]);
}

void test_parser_handles_fragmented_and_coalesced_frames() {
  std::vector<uint8_t> first;
  std::vector<uint8_t> second;
  TEST_ASSERT_TRUE(buildFrame(CMD_PONG, {}, first));
  TEST_ASSERT_TRUE(buildFrame(CMD_RX_STARTED, {0x01}, second));
  first.insert(first.end(), second.begin(), second.end());

  FrameParser parser;
  std::vector<Frame> received;
  const FrameParser::FrameCallback callback = [&received](const Frame &frame) {
    received.push_back(frame);
  };
  parser.feed(first.data(), 2, callback);
  TEST_ASSERT_EQUAL_UINT(0, received.size());
  parser.feed(first.data() + 2, first.size() - 2, callback);
  TEST_ASSERT_EQUAL_UINT(2, received.size());
  TEST_ASSERT_EQUAL_HEX8(CMD_PONG, received[0].command);
  TEST_ASSERT_EQUAL_HEX8(CMD_RX_STARTED, received[1].command);
  TEST_ASSERT_EQUAL_HEX8(0x01, received[1].payload[0]);
}

void test_parser_resynchronizes_after_garbage_and_bad_crc() {
  std::vector<uint8_t> bad;
  std::vector<uint8_t> good;
  TEST_ASSERT_TRUE(buildFrame(CMD_PING, {0x01}, bad));
  bad[bad.size() - 1] ^= 0xFF;
  TEST_ASSERT_TRUE(buildFrame(CMD_PONG, {}, good));

  std::vector<uint8_t> stream = {0x00, 0x7F, 0xAA};
  stream.insert(stream.end(), bad.begin() + 1, bad.end());
  stream.insert(stream.end(), good.begin(), good.end());
  FrameParser parser;
  std::vector<Frame> received;
  parser.feed(stream,
              [&received](const Frame &frame) { received.push_back(frame); });
  TEST_ASSERT_EQUAL_UINT(1, received.size());
  TEST_ASSERT_EQUAL_HEX8(CMD_PONG, received[0].command);
  TEST_ASSERT_GREATER_THAN_UINT(0, parser.rejectedFrames());
}

void test_radio_config_and_status_layouts_are_little_endian() {
  RadioConfig config{869618000, 62500, 8, 8, 22, 0x12, 16};
  const std::vector<uint8_t> encoded = encodeRadioConfig(config);
  TEST_ASSERT_EQUAL_UINT(RADIO_CONFIG_SIZE, encoded.size());
  TEST_ASSERT_EQUAL_HEX8(0x50, encoded[0]);
  TEST_ASSERT_EQUAL_HEX8(0x12, encoded[11]);

  const uint8_t status_bytes[STATUS_RESP_SIZE] = {
      1, 0, 0, 0, 2,    0,    0,  0, 3,    0,    0,  0,
      4, 0, 0, 0, 0xCE, 0xFF, 25, 0, 0x9C, 0xFF, 20, 2,
  };
  StatusResp status;
  TEST_ASSERT_TRUE(
      decodeStatusResp(status_bytes, sizeof(status_bytes), status));
  TEST_ASSERT_EQUAL_UINT32(1, status.uptime_sec);
  TEST_ASSERT_EQUAL_INT16(-50, status.last_rssi);
  TEST_ASSERT_EQUAL_INT16(25, status.last_snr_x10);
  TEST_ASSERT_EQUAL_HEX8(2, status.radio_state);
}

void test_meshtastic_mapping_accepts_supported_phy_values() {
  RadioConfig config;
  TEST_ASSERT_TRUE(makeRadioConfig(869.618f, 62.5f, 8, 8, 22, 0x2b, 16,
                                   config));
  TEST_ASSERT_EQUAL_UINT32(869618000, config.freq_hz);
  TEST_ASSERT_EQUAL_UINT32(62500, config.bandwidth_hz);
  TEST_ASSERT_EQUAL_UINT8(8, config.sf);
  TEST_ASSERT_EQUAL_UINT8(8, config.cr);
  TEST_ASSERT_EQUAL_INT8(22, config.power_dbm);
  TEST_ASSERT_EQUAL_HEX16(0x2b, config.syncword);
  TEST_ASSERT_EQUAL_UINT8(16, config.preamble_len);
}

void test_meshtastic_mapping_rejects_unsupported_values_and_payload_bounds() {
  RadioConfig config;
  TEST_ASSERT_FALSE(makeRadioConfig(869.618f, 100.0f, 8, 8, 22, 0x2b, 16,
                                    config));
  TEST_ASSERT_FALSE(makeRadioConfig(869.618f, 62.5f, 8, 4, 22, 0x2b, 16,
                                    config));
  TEST_ASSERT_FALSE(makeRadioConfig(869.618f, 62.5f, 8, 8, 22, 0x2b, 256,
                                    config));
  TEST_ASSERT_TRUE(isValidAirPacketLength(MAX_LORA_PAYLOAD));
  TEST_ASSERT_FALSE(isValidAirPacketLength(MAX_LORA_PAYLOAD + 1));
}

void test_rx_packet_metadata_and_air_bytes_decode() {
  const std::vector<uint8_t> payload = {0xCE, 0xFF, 25, 0, 0xD0, 0xFF,
                                        0x01, 0x02, 0x03};
  RxPacket packet;
  TEST_ASSERT_TRUE(decodeRxPacket(payload, packet));
  TEST_ASSERT_EQUAL_INT16(-50, packet.rssi);
  TEST_ASSERT_EQUAL_INT16(25, packet.snr_x10);
  TEST_ASSERT_EQUAL_INT16(-48, packet.signal_rssi);
  TEST_ASSERT_EQUAL_UINT(3, packet.data.size());
  TEST_ASSERT_EQUAL_HEX8(0x01, packet.data[0]);
  TEST_ASSERT_EQUAL_HEX8(0x03, packet.data[2]);
}

extern "C" void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_crc_matches_ccitt_false_vector);
  RUN_TEST(test_codec_matches_wire_layout);
  RUN_TEST(test_build_frame_rejects_nonempty_null_payload);
  RUN_TEST(test_parser_handles_fragmented_and_coalesced_frames);
  RUN_TEST(test_parser_resynchronizes_after_garbage_and_bad_crc);
  RUN_TEST(test_radio_config_and_status_layouts_are_little_endian);
  RUN_TEST(test_meshtastic_mapping_accepts_supported_phy_values);
  RUN_TEST(test_meshtastic_mapping_rejects_unsupported_values_and_payload_bounds);
  RUN_TEST(test_rx_packet_metadata_and_air_bytes_decode);
  exit(UNITY_END());
}

extern "C" void loop() {}
