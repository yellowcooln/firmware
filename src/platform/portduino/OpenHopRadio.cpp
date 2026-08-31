#include "OpenHopRadio.h"

#include "MeshTypes.h"
#include "MeshRadio.h"
#include "PortduinoGlue.h"
#include "Router.h"
#include "configuration.h"
#include "mesh/mesh-pb-constants.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

namespace openhop {

namespace {

constexpr uint16_t MESHTASTIC_SYNCWORD = 0x2b;

} // namespace

} // namespace openhop

OpenHopRadio::OpenHopRadio()
    : NotifiedWorkerThread("OpenHopRadio"),
      session_(new openhop::TcpSession({
          portduino_config.openhop_host,
          portduino_config.openhop_port,
          portduino_config.openhop_token,
          portduino_config.openhop_connect_timeout_ms,
          portduino_config.openhop_reconnect_initial_ms,
          portduino_config.openhop_reconnect_max_ms})) {
  session_->setRxPacketCallback(
      [this](const openhop::RxPacket &packet) { receivePacket(packet); });
}

OpenHopRadio::~OpenHopRadio() {
  // Stop/join the TCP reader before releasing pool-owned packets. The reader
  // can otherwise still invoke receivePacket() during destruction.
  if (session_)
    session_->stop();
  releaseQueuedPackets();
  std::lock_guard<std::mutex> lock(rx_mutex_);
  rx_queue_.clear();
  if (sendingPacket) {
    packetPool.release(sendingPacket);
    sendingPacket = nullptr;
  }
}

bool OpenHopRadio::init() {
  if (!RadioInterface::init())
    return false;
  if (!applyOpenHopConfig())
    return false;

  // TcpSession owns reconnect and intentionally starts asynchronously. A
  // missing or powered-off modem therefore does not make firmware boot fail.
  openhop::RadioConfig radio_config;
  if (!openhop::makeRadioConfig(getFreq(), bw, sf, cr, power,
                                openhop::MESHTASTIC_SYNCWORD, preambleLength,
                                radio_config) ||
      !session_->setRadioConfig(radio_config))
    return false;
  return session_->start();
}

bool OpenHopRadio::reconfigure() {
  if (!RadioInterface::reconfigure())
    return false;
  if (!applyOpenHopConfig())
    return false;

  openhop::RadioConfig radio_config;
  if (!openhop::makeRadioConfig(getFreq(), bw, sf, cr, power,
                                openhop::MESHTASTIC_SYNCWORD,
                                preambleLength, radio_config))
    return false;
  return session_->setRadioConfig(radio_config);
}

bool OpenHopRadio::applyOpenHopConfig() {
  // Base applyModemConfig() already resolves region, preset, channel hash,
  // frequency offset, power limit, SF and CR. Clamp final power to the
  // documented modem input range before constructing its wire config.
  limitPower(22);
  openhop::RadioConfig radio_config;
  if (myRegion && myRegion->wideLora) {
    LOG_ERROR("openHop modem does not support 2.4 GHz LoRa");
    return false;
  }
  if (!openhop::makeRadioConfig(getFreq(), bw, sf, cr, power,
                                openhop::MESHTASTIC_SYNCWORD,
                                preambleLength, radio_config)) {
    LOG_ERROR("Unsupported openHop radio configuration: freq=%.3f bw=%.3f sf=%u cr=%u power=%d preamble=%u",
              getFreq(), bw, sf, cr, power, preambleLength);
    return false;
  }
  return true;
}

ErrorCode OpenHopRadio::send(meshtastic_MeshPacket *p) {
  if (!p)
    return ERRNO_UNKNOWN;

  if (disabled || !config.lora.tx_enabled) {
    packetPool.release(p);
    return ERRNO_DISABLED;
  }
  if (p->to == NODENUM_BROADCAST_NO_LORA)
    return ERRNO_SHOULD_RELEASE;

  // beginSending() emits PacketHeader + encrypted bytes. Validate before
  // enqueue so an oversized packet cannot reach its assert or modem.
  if (p->which_payload_variant != meshtastic_MeshPacket_encrypted_tag ||
      p->encrypted.size > sizeof(radioBuffer.payload) ||
      p->encrypted.size + sizeof(PacketHeader) > openhop::MAX_LORA_PAYLOAD) {
    LOG_WARN("Drop oversized or unencoded openHop packet id=0x%08x", p->id);
    packetPool.release(p);
    return ERRNO_UNKNOWN;
  }

  bool dropped = false;
  const ErrorCode result =
      txQueue.enqueue(p, &dropped) ? ERRNO_OK : ERRNO_UNKNOWN;
  if (dropped)
    ++txDrop;
  if (result != ERRNO_OK) {
    packetPool.release(p);
    return result;
  }

  setTransmitDelay();
  return result;
}

meshtastic_QueueStatus OpenHopRadio::getQueueStatus() {
  meshtastic_QueueStatus status;
  status.res = status.mesh_packet_id = 0;
  status.free = txQueue.getFree();
  status.maxlen = txQueue.getMaxLen();
  return status;
}

bool OpenHopRadio::cancelSending(NodeNum from, PacketId id) {
  meshtastic_MeshPacket *packet = txQueue.remove(from, id);
  if (!packet)
    return false;
  packetPool.release(packet);
  return true;
}

bool OpenHopRadio::findInTxQueue(NodeNum from, PacketId id) {
  return txQueue.find(from, id);
}

bool OpenHopRadio::canSleep(bool deepSleep) {
  return txQueue.empty() && (!deepSleep || sendingPacket == nullptr);
}

bool OpenHopRadio::isChannelActive() {
  // LBT is an explicit modem request. There is no safe nonblocking status
  // probe to perform here, so worker code must call performCad().
  return false;
}

bool OpenHopRadio::isActivelyReceiving() {
  std::lock_guard<std::mutex> lock(rx_mutex_);
  return !rx_queue_.empty();
}

uint32_t OpenHopRadio::getPacketTime(uint32_t packet_length, bool) {
  if (packet_length == 0 || bw <= 0.0f || sf < 5 || cr < 5)
    return 0;

  const float bandwidth_hz = bw * 1000.0f;
  const float symbol_time = static_cast<float>(1u << sf) / bandwidth_hz;
  const bool low_data_rate_optimize = symbol_time >= 16e-3f;
  const float preamble_time = (preambleLength + 4.25f) * symbol_time;
  const float payload_symbols =
      8.0f + std::max(
                  std::ceil((8.0f * packet_length - 4.0f * sf + 28.0f +
                             16.0f - 20.0f) /
                            (4.0f * (sf - 2.0f * low_data_rate_optimize))) *
                      cr,
                  0.0f);
  const float airtime_ms =
      (preamble_time + payload_symbols * symbol_time) * 1000.0f;
  return airtime_ms <= 0.0f ? 0 : static_cast<uint32_t>(airtime_ms);
}

int16_t OpenHopRadio::getCurrentRSSI() {
  return current_rssi_.load();
}

void OpenHopRadio::setTransmitDelay() {
  meshtastic_MeshPacket *packet = txQueue.getFront();
  if (!packet || sendingPacket)
    return;

  uint32_t delay = 1;
  if (packet->tx_after) {
    const uint32_t now = millis();
    delay = packet->tx_after > now ? packet->tx_after - now : 1;
  } else if (airTime) {
    delay = (packet->rx_snr == 0 && packet->rx_rssi == 0)
                ? getTxDelayMsec()
                : getTxDelayMsecWeighted(packet);
  }
  notifyLater(delay, TRANSMIT_DELAY_COMPLETED, false);
}

bool OpenHopRadio::performCad() {
  if (!session_->isConnected())
    return false;
  std::vector<uint8_t> response;
  if (!session_->request(openhop::CMD_CAD_REQUEST, {}, openhop::CMD_CAD_RESP,
                         response, 1000)) {
    LOG_WARN("openHop CAD failed; retaining packet in TX queue");
    return false;
  }
  if (response.empty())
    return false;
  return response[0] == 0;
}

bool OpenHopRadio::restoreReceive() {
  std::vector<uint8_t> response;
  return session_->request(openhop::CMD_RX_START, {}, openhop::CMD_RX_STARTED,
                           response, 2000);
}

bool OpenHopRadio::startSend(meshtastic_MeshPacket *packet) {
  assert(packet && !sendingPacket);
  if (!packet || packet->encrypted.size + sizeof(PacketHeader) >
                     openhop::MAX_LORA_PAYLOAD) {
    if (packet)
      packetPool.release(packet);
    return false;
  }

  const size_t length = beginSending(packet);
  std::vector<uint8_t> air_packet(
      reinterpret_cast<const uint8_t *>(&radioBuffer),
      reinterpret_cast<const uint8_t *>(&radioBuffer) + length);
  std::vector<uint8_t> response;
  const bool sent = session_->request(openhop::CMD_TX_REQUEST, air_packet,
                                      openhop::CMD_TX_DONE, response, 10000);

  uint32_t airtime_ms = getPacketTime(static_cast<uint32_t>(length));
  if (sent && response.size() >= 4) {
    const uint32_t airtime_us = static_cast<uint32_t>(response[0]) |
                                (static_cast<uint32_t>(response[1]) << 8) |
                                (static_cast<uint32_t>(response[2]) << 16) |
                                (static_cast<uint32_t>(response[3]) << 24);
    airtime_ms = airtime_us / 1000;
  }

  // Modem sends TX_DONE/TX_FAIL asynchronously, then host explicitly puts
  // it back in continuous RX. Do this for both success and failure paths.
  if (!restoreReceive())
    LOG_WARN("openHop modem RX restore failed after TX");
  completeSending(sent, airtime_ms);
  return sent;
}

void OpenHopRadio::completeSending(bool success, uint32_t airtime_ms) {
  meshtastic_MeshPacket *packet = sendingPacket;
  sendingPacket = nullptr;
  if (!packet)
    return;

  if (success) {
    ++txGood;
    if (!isFromUs(packet))
      ++txRelay;
    if (airTime)
      airTime->logAirtime(TX_LOG, airtime_ms);
  } else {
    ++txDrop;
  }
  packetPool.release(packet);
}

void OpenHopRadio::receivePacket(const openhop::RxPacket &packet) {
  current_rssi_.store(packet.rssi);
  std::lock_guard<std::mutex> lock(rx_mutex_);
  rx_queue_.push_back(packet);
  notify(ISR_RX, true);
}

void OpenHopRadio::drainReceivedPackets() {
  std::deque<openhop::RxPacket> packets;
  {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    packets.swap(rx_queue_);
  }

  for (const openhop::RxPacket &received : packets) {
    const uint32_t airtime_ms =
        getPacketTime(static_cast<uint32_t>(received.data.size()), true);
    if (airTime)
      airTime->logAirtime(RX_ALL_LOG, airtime_ms);

    if (!openhop::isValidAirPacketLength(received.data.size()) ||
        received.data.size() < sizeof(PacketHeader)) {
      ++rxBad;
      continue;
    }

    const auto *header =
        reinterpret_cast<const PacketHeader *>(received.data.data());
    const size_t payload_length = received.data.size() - sizeof(PacketHeader);
    if (header->from == 0 || payload_length > sizeof(radioBuffer.payload)) {
      ++rxBad;
      continue;
    }

    meshtastic_MeshPacket *packet = packetPool.allocZeroed();
    if (!packet) {
      ++rxBad;
      continue;
    }

    packet->from = header->from;
    packet->to = header->to;
    packet->id = header->id;
    packet->channel = header->channel;
    packet->hop_limit = header->flags & PACKET_FLAGS_HOP_LIMIT_MASK;
    packet->hop_start =
        (header->flags & PACKET_FLAGS_HOP_START_MASK) >>
        PACKET_FLAGS_HOP_START_SHIFT;
    packet->want_ack = !!(header->flags & PACKET_FLAGS_WANT_ACK_MASK);
    packet->via_mqtt = !!(header->flags & PACKET_FLAGS_VIA_MQTT_MASK);
    packet->next_hop = packet->hop_start == 0 ? NO_NEXT_HOP_PREFERENCE
                                              : header->next_hop;
    packet->relay_node = packet->hop_start == 0 ? NO_RELAY_NODE
                                                 : header->relay_node;
    packet->rx_rssi = received.rssi;
    packet->rx_snr = received.snr_x10 / 10.0f;
    packet->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
    packet->encrypted.size = payload_length;
    std::memcpy(packet->encrypted.bytes,
                received.data.data() + sizeof(PacketHeader), payload_length);

    ++rxGood;
    if (airTime)
      airTime->logAirtime(RX_LOG, airtime_ms);
    deliverToReceiver(packet);
  }
}

void OpenHopRadio::onNotify(uint32_t notification) {
  switch (notification) {
  case ISR_RX:
    drainReceivedPackets();
    setTransmitDelay();
    break;
  case TRANSMIT_DELAY_COMPLETED:
    if (!txQueue.empty()) {
      if (!session_->isConnected() || !performCad()) {
        restoreReceive();
        setTransmitDelay();
        break;
      }

      meshtastic_MeshPacket *packet = txQueue.dequeue();
      if (packet)
        startSend(packet);
      setTransmitDelay();
    }
    break;
  default:
    break;
  }
}

void OpenHopRadio::releaseQueuedPackets() {
  while (meshtastic_MeshPacket *packet = txQueue.dequeue())
    packetPool.release(packet);
}
