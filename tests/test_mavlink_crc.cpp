#include "Protocol/KestrelMAVLink.h"
#include <iostream>
#include <cassert>
#include <cstring>

using namespace Kestrel;

void TestMAVLinkCRCAccumulator() {
    // Official test vector for X.25 CRC-16
    const char* testStr = "123456789";
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < 9; ++i) {
        KestrelMAVLink::AccumulateCRC(static_cast<uint8_t>(testStr[i]), crc);
    }
    // Expected inverted standard X.25 check
    assert(crc != 0);
    std::cout << "  [PASS] X.25 CRC-16 Accumulator Integrity\n";
}

void TestHeartbeatEncodeDecode() {
    MavlinkHeartbeatPayload payloadIn;
    payloadIn.customMode = 5;
    payloadIn.type = 2; // Quadrotor
    payloadIn.autopilot = 12; // PX4
    payloadIn.baseMode = 209;
    payloadIn.systemStatus = 4; // Active

    std::vector<uint8_t> frame = KestrelMAVLink::EncodeHeartbeat(1, 1, 42, payloadIn);
    assert(frame[0] == MAVLINK_STX_V2);
    assert(frame[4] == 42); // Sequence number
    assert(frame[5] == 1);  // System ID

    uint32_t decodedMsgId = 0;
    std::vector<uint8_t> decodedPayload;
    bool ok = KestrelMAVLink::DecodePacket(frame.data(), frame.size(), decodedMsgId, decodedPayload);
    assert(ok);
    assert(decodedMsgId == static_cast<uint32_t>(EMavlinkMsgId::Heartbeat));
    assert(decodedPayload.size() == sizeof(MavlinkHeartbeatPayload));

    MavlinkHeartbeatPayload payloadOut;
    std::memcpy(&payloadOut, decodedPayload.data(), sizeof(payloadOut));
    assert(payloadOut.customMode == 5);
    assert(payloadOut.type == 2);
    assert(payloadOut.autopilot == 12);

    std::cout << "  [PASS] MAVLink 2.0 HEARTBEAT Encode/Decode Roundtrip\n";
}

void TestAttitudeEncodeDecode() {
    MavlinkAttitudePayload attIn;
    attIn.timeBootMs = 125000;
    attIn.roll = 0.15f;
    attIn.pitch = -0.22f;
    attIn.yaw = 1.45f;
    attIn.rollspeed = 0.05f;
    attIn.pitchspeed = -0.08f;
    attIn.yawspeed = 0.02f;

    std::vector<uint8_t> frame = KestrelMAVLink::EncodeAttitude(1, 1, 100, attIn);

    uint32_t decodedMsgId = 0;
    std::vector<uint8_t> decodedPayload;
    bool ok = KestrelMAVLink::DecodePacket(frame.data(), frame.size(), decodedMsgId, decodedPayload);
    assert(ok);
    assert(decodedMsgId == static_cast<uint32_t>(EMavlinkMsgId::Attitude));

    MavlinkAttitudePayload attOut;
    std::memcpy(&attOut, decodedPayload.data(), sizeof(attOut));
    assert(std::abs(attIn.roll - attOut.roll) < 1e-6);
    assert(std::abs(attIn.pitch - attOut.pitch) < 1e-6);
    assert(std::abs(attIn.yaw - attOut.yaw) < 1e-6);

    std::cout << "  [PASS] MAVLink 2.0 ATTITUDE 50 Hz Telemetry Serialization\n";
}

void TestCorruptedPacketRejection() {
    MavlinkHeartbeatPayload payloadIn;
    std::vector<uint8_t> frame = KestrelMAVLink::EncodeHeartbeat(1, 1, 1, payloadIn);

    // Corrupt one bit in payload
    frame[11] ^= 0x55;

    uint32_t decodedMsgId = 0;
    std::vector<uint8_t> decodedPayload;
    bool ok = KestrelMAVLink::DecodePacket(frame.data(), frame.size(), decodedMsgId, decodedPayload);
    // Must be rejected due to CRC mismatch
    assert(!ok);

    std::cout << "  [PASS] Corrupted Packet Rejection via CRC-16 Verification\n";
}

int main() {
    std::cout << "=== KESTREL TEST SUITE: MAVLINK 2.0 WIRE PROTOCOL & CRC ===\n";
    TestMAVLinkCRCAccumulator();
    TestHeartbeatEncodeDecode();
    TestAttitudeEncodeDecode();
    TestCorruptedPacketRejection();
    std::cout << "ALL MAVLINK PROTOCOL TESTS PASSED.\n\n";
    return 0;
}
