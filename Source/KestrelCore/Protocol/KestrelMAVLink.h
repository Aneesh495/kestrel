#pragma once

#include "Math/KestrelMathTypes.h"
#include <vector>
#include <cstdint>
#include <cstring>

namespace Kestrel {

constexpr uint8_t MAVLINK_STX_V2 = 0xFD; // MAVLink 2.0 packet start indicator

// MAVLink Standard Message IDs
enum class EMavlinkMsgId : uint32_t {
    Heartbeat = 0,
    SysStatus = 1,
    Attitude = 30,
    GlobalPositionInt = 33,
    BatteryStatus = 147,
    CommandLong = 76
};

#pragma pack(push, 1)

struct MavlinkHeartbeatPayload {
    uint32_t customMode{0};
    uint8_t type{2};            // MAV_TYPE_QUADROTOR
    uint8_t autopilot{12};      // MAV_AUTOPILOT_PX4
    uint8_t baseMode{209};      // Armed, Guided mode
    uint8_t systemStatus{4};    // MAV_STATE_ACTIVE
    uint8_t mavlinkVersion{3};
};

struct MavlinkAttitudePayload {
    uint32_t timeBootMs{0};
    float roll{0.0f};           // Radians
    float pitch{0.0f};          // Radians
    float yaw{0.0f};            // Radians
    float rollspeed{0.0f};      // Rad/s
    float pitchspeed{0.0f};     // Rad/s
    float yawspeed{0.0f};       // Rad/s
};

struct MavlinkGlobalPositionPayload {
    uint32_t timeBootMs{0};
    int32_t lat{0};             // Deg * 1e7
    int32_t lon{0};             // Deg * 1e7
    int32_t alt{0};             // Millimeters MSL
    int32_t relativeAlt{0};     // Millimeters above takeoff ground
    int16_t vx{0};              // cm/s North
    int16_t vy{0};              // cm/s East
    int16_t vz{0};              // cm/s Down
    uint16_t hdg{0};            // cdeg (0..35999)
};

struct MavlinkBatteryStatusPayload {
    int32_t currentConsumed{0}; // mAh
    int32_t energyConsumed{0};  // hJ
    int16_t temperature{2500};  // cdeg C (25.0 C)
    uint16_t voltages[10]{};    // mV per cell
    int16_t currentBattery{0};  // cA (10 mA steps)
    uint8_t id{0};
    uint8_t batteryFunction{0};
    uint8_t type{1};            // LiPo
    int8_t batteryRemaining{100}; // Percent [0..100]
};

#pragma pack(pop)

/**
 * MAVLink 2.0 Packet Encoder, Decoder, and X.25 CRC-16 Checksum Calculator.
 */
class KestrelMAVLink {
public:
    static uint16_t CalculateCRC(const uint8_t* buffer, size_t length, uint8_t crcExtra);
    static void AccumulateCRC(uint8_t data, uint16_t& crcAccum);

    static std::vector<uint8_t> EncodeHeartbeat(uint8_t systemId,
                                                uint8_t componentId,
                                                uint8_t sequence,
                                                const MavlinkHeartbeatPayload& payload);

    static std::vector<uint8_t> EncodeAttitude(uint8_t systemId,
                                               uint8_t componentId,
                                               uint8_t sequence,
                                               const MavlinkAttitudePayload& payload);

    static std::vector<uint8_t> EncodeGlobalPosition(uint8_t systemId,
                                                     uint8_t componentId,
                                                     uint8_t sequence,
                                                     const MavlinkGlobalPositionPayload& payload);

    static std::vector<uint8_t> EncodeBatteryStatus(uint8_t systemId,
                                                    uint8_t componentId,
                                                    uint8_t sequence,
                                                    const MavlinkBatteryStatusPayload& payload);

    static bool DecodePacket(const uint8_t* packetData,
                             size_t packetLength,
                             uint32_t& outMsgId,
                             std::vector<uint8_t>& outPayload);

private:
    static uint8_t GetCrcExtra(EMavlinkMsgId msgId);
};

} // namespace Kestrel
