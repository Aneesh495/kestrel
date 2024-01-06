#include "KestrelMAVLink.h"

namespace Kestrel {

void KestrelMAVLink::AccumulateCRC(uint8_t data, uint16_t& crcAccum) {
    uint8_t tmp = data ^ (uint8_t)(crcAccum & 0xFF);
    tmp ^= (tmp << 4);
    crcAccum = (crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

uint16_t KestrelMAVLink::CalculateCRC(const uint8_t* buffer, size_t length, uint8_t crcExtra) {
    uint16_t crc = 0xFFFF; // X.25 initial seed
    for (size_t i = 0; i < length; ++i) {
        AccumulateCRC(buffer[i], crc);
    }
    AccumulateCRC(crcExtra, crc);
    return crc;
}

uint8_t KestrelMAVLink::GetCrcExtra(EMavlinkMsgId msgId) {
    switch (msgId) {
        case EMavlinkMsgId::Heartbeat: return 50;
        case EMavlinkMsgId::SysStatus: return 124;
        case EMavlinkMsgId::Attitude: return 39;
        case EMavlinkMsgId::GlobalPositionInt: return 104;
        case EMavlinkMsgId::BatteryStatus: return 154;
        case EMavlinkMsgId::CommandLong: return 152;
        default: return 0;
    }
}

static std::vector<uint8_t> BuildMavlink2Frame(uint8_t systemId,
                                              uint8_t componentId,
                                              uint8_t sequence,
                                              EMavlinkMsgId msgId,
                                              const uint8_t* payload,
                                              uint8_t payloadLen,
                                              uint8_t crcExtra) {
    // Frame header length = 10 bytes (STX, len, incompat, compat, seq, sysid, compid, msgid[3])
    // Frame total = 10 + payloadLen + 2 (CRC)
    std::vector<uint8_t> frame(10 + payloadLen + 2);

    frame[0] = MAVLINK_STX_V2;
    frame[1] = payloadLen;
    frame[2] = 0x00; // Incompatibility flags
    frame[3] = 0x00; // Compatibility flags
    frame[4] = sequence;
    frame[5] = systemId;
    frame[6] = componentId;

    uint32_t id = static_cast<uint32_t>(msgId);
    frame[7] = static_cast<uint8_t>(id & 0xFF);
    frame[8] = static_cast<uint8_t>((id >> 8) & 0xFF);
    frame[9] = static_cast<uint8_t>((id >> 16) & 0xFF);

    if (payloadLen > 0 && payload != nullptr) {
        std::memcpy(&frame[10], payload, payloadLen);
    }

    // CRC is calculated over bytes 1 to 9 + payload + crcExtra (excludes STX byte 0)
    uint16_t crc = KestrelMAVLink::CalculateCRC(&frame[1], 9 + payloadLen, crcExtra);
    frame[10 + payloadLen] = static_cast<uint8_t>(crc & 0xFF);
    frame[10 + payloadLen + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return frame;
}

std::vector<uint8_t> KestrelMAVLink::EncodeHeartbeat(uint8_t systemId,
                                                    uint8_t componentId,
                                                    uint8_t sequence,
                                                    const MavlinkHeartbeatPayload& payload) {
    return BuildMavlink2Frame(systemId, componentId, sequence, EMavlinkMsgId::Heartbeat,
                              reinterpret_cast<const uint8_t*>(&payload),
                              sizeof(payload), GetCrcExtra(EMavlinkMsgId::Heartbeat));
}

std::vector<uint8_t> KestrelMAVLink::EncodeAttitude(uint8_t systemId,
                                                   uint8_t componentId,
                                                   uint8_t sequence,
                                                   const MavlinkAttitudePayload& payload) {
    return BuildMavlink2Frame(systemId, componentId, sequence, EMavlinkMsgId::Attitude,
                              reinterpret_cast<const uint8_t*>(&payload),
                              sizeof(payload), GetCrcExtra(EMavlinkMsgId::Attitude));
}

std::vector<uint8_t> KestrelMAVLink::EncodeGlobalPosition(uint8_t systemId,
                                                         uint8_t componentId,
                                                         uint8_t sequence,
                                                         const MavlinkGlobalPositionPayload& payload) {
    return BuildMavlink2Frame(systemId, componentId, sequence, EMavlinkMsgId::GlobalPositionInt,
                              reinterpret_cast<const uint8_t*>(&payload),
                              sizeof(payload), GetCrcExtra(EMavlinkMsgId::GlobalPositionInt));
}

std::vector<uint8_t> KestrelMAVLink::EncodeBatteryStatus(uint8_t systemId,
                                                        uint8_t componentId,
                                                        uint8_t sequence,
                                                        const MavlinkBatteryStatusPayload& payload) {
    return BuildMavlink2Frame(systemId, componentId, sequence, EMavlinkMsgId::BatteryStatus,
                              reinterpret_cast<const uint8_t*>(&payload),
                              sizeof(payload), GetCrcExtra(EMavlinkMsgId::BatteryStatus));
}

bool KestrelMAVLink::DecodePacket(const uint8_t* packetData,
                                 size_t packetLength,
                                 uint32_t& outMsgId,
                                 std::vector<uint8_t>& outPayload) {
    if (packetLength < 12 || packetData[0] != MAVLINK_STX_V2) {
        return false;
    }

    uint8_t payloadLen = packetData[1];
    if (packetLength < static_cast<size_t>(12 + payloadLen)) {
        return false;
    }

    outMsgId = packetData[7] | (packetData[8] << 8) | (packetData[9] << 16);
    uint8_t crcExtra = GetCrcExtra(static_cast<EMavlinkMsgId>(outMsgId));

    // Verify CRC
    uint16_t expectedCrc = KestrelMAVLink::CalculateCRC(&packetData[1], 9 + payloadLen, crcExtra);
    uint16_t receivedCrc = packetData[10 + payloadLen] | (packetData[10 + payloadLen + 1] << 8);

    if (expectedCrc != receivedCrc) {
        return false;
    }

    outPayload.resize(payloadLen);
    if (payloadLen > 0) {
        std::memcpy(outPayload.data(), &packetData[10], payloadLen);
    }
    return true;
}

} // namespace Kestrel
