#pragma once

#include "MeshPacketQueue.h"
#include "RadioInterface.h"
#include "concurrency/NotifiedWorkerThread.h"
#include "platform/portduino/OpenHopProtocol.h"
#include "platform/portduino/OpenHopTcpSession.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>

namespace openhop {
} // namespace openhop

/** RadioInterface adapter for an openHop TCP modem. */
class OpenHopRadio : public RadioInterface,
                     protected concurrency::NotifiedWorkerThread {
  public:
    OpenHopRadio();
    ~OpenHopRadio() override;

    OpenHopRadio(const OpenHopRadio &) = delete;
    OpenHopRadio &operator=(const OpenHopRadio &) = delete;

    ErrorCode send(meshtastic_MeshPacket *p) override;
    meshtastic_QueueStatus getQueueStatus() override;
    bool cancelSending(NodeNum from, PacketId id) override;
    bool findInTxQueue(NodeNum from, PacketId id) override;

    bool init() override;
    bool reconfigure() override;
    bool canSleep(bool deepSleep = false) override;
    bool wideLora() override { return false; }
    bool supportsSubGhz() override { return true; }

    // CAD is performed by the modem immediately before each TX. The radio is
    // otherwise continuously restored to RX by restoreReceive().
    bool isChannelActive();
    bool isActivelyReceiving();

    uint32_t getPacketTime(uint32_t totalPacketLen,
                           bool received = false) override;
    int16_t getCurrentRSSI() override;

    // Counters mirror RadioLibInterface/SimRadio and make adapter behaviour
    // observable without reaching into the worker implementation.
    uint32_t rxBad = 0;
    uint32_t rxGood = 0;
    uint32_t txGood = 0;
    uint32_t txRelay = 0;
    uint16_t txDrop = 0;

  protected:
    enum PendingNotification {
        TRANSMIT_DELAY_COMPLETED = 1,
        ISR_RX = 2,
    };

    void onNotify(uint32_t notification) override;

  private:
    void setTransmitDelay();
    void drainReceivedPackets();
    void receivePacket(const openhop::RxPacket &packet);
    bool performCad();
    bool restoreReceive();
    bool startSend(meshtastic_MeshPacket *packet);
    void completeSending(bool success, uint32_t airtime_ms);
    bool applyOpenHopConfig();
    void releaseQueuedPackets();

    MeshPacketQueue txQueue = MeshPacketQueue(MAX_TX_QUEUE);
    std::unique_ptr<openhop::TcpSession> session_;

    mutable std::mutex rx_mutex_;
    std::deque<openhop::RxPacket> rx_queue_;
    std::atomic<int16_t> current_rssi_{0};
};
