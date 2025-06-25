//#define FLY_SKY
#ifdef FLY_SKY

#ifndef FLYSKY_REMOTE_HPP_
#define FLYSKY_REMOTE_HPP_

#include <cstdint>

#ifndef PLATFORM_HOSTED
#include "modm/platform.hpp"
#endif

#include "tap/util_macros.hpp"

namespace tap
{
class Drivers;
}

namespace SwitchState
{
constexpr double UP = 1.0;
constexpr double MIDDLE = 0.5;
constexpr double DOWN = 0.0;
}  // namespace SwitchState

namespace KeyValues
{
constexpr uint16_t W = 1;
constexpr uint16_t S = 2;
constexpr uint16_t A = 4;
constexpr uint16_t D = 8;
constexpr uint16_t SHIFT = 16;
constexpr uint16_t CTRL = 32;
constexpr uint16_t Q = 64;
constexpr uint16_t E = 128;
constexpr uint16_t R = 256;
constexpr uint16_t F = 512;
constexpr uint16_t G = 1024;
constexpr uint16_t Z = 2048;
constexpr uint16_t X = 4096;
constexpr uint16_t C = 8192;
constexpr uint16_t V = 16384;
constexpr uint16_t B = 32768;

}  // namespace KeyValues

namespace tap::communication::serial
{
class FlySky
{
public:
    static constexpr uint8_t IBUS_PACKET_SIZE = 32;
    static constexpr uint8_t MAX_CHANNELS = 14;
    static constexpr uint32_t FLYSKY_DISCONNECT_TIMEOUT = 100;  // ms
    static constexpr uint32_t FLYSKY_READ_TIMEOUT = 50;         // ms

    FlySky(Drivers* drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(FlySky)
    mockable ~FlySky() = default;

    enum Channel
    {
        RIGHT_HORIZONTAL,
        RIGHT_VERTICAL,
        LEFT_VERTICAL,
        LEFT_HORIZONTAL,
        SWITCH_A,
        SWITCH_B,
        SWITCH_C,
        SWITCH_D,
        WHEEL_A,
        WHEEL_B
    };

    void initialize();
    void read();

    bool isConnected() const;

    float getChannel(uint8_t ch) const;
    uint16_t getRawChannel(uint8_t ch) const;

    mockable uint32_t getUpdateCounter() const;

private:
    uint32_t updateCounter = 0;

    void parsePacket();

    Drivers* drivers;

    uint8_t ibusBuffer[IBUS_PACKET_SIZE] = {0};
    uint16_t channels[MAX_CHANNELS] = {0};

    bool connected = false;
    uint32_t lastRead = 0;
    uint8_t syncState = 0;
    uint8_t dataIndex = 0;
};

}  // namespace tap::communication::serial

#endif

#endif  // FLY_SKY