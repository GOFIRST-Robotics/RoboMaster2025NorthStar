//#define FLY_SKY
#ifdef FLY_SKY
/*
 * FlySky iBUS receiver driver for Taproot.
 * Compatible with FS-iA6B receiver and FS-i6X transmitter.
 *
 * Based on Remote.cpp style (object-based, Drivers passed in).
 */

#include "fly_sky.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

namespace tap::communication::serial
{
void FlySky::initialize()
{
    drivers->uart.init<Uart::UartPort::Uart1, 115200, Uart::Parity::Disabled>();
}

void FlySky::read()
{
    // Check disconnect timeout
    if (tap::arch::clock::getTimeMilliseconds() - lastRead > FLYSKY_DISCONNECT_TIMEOUT)
    {
        connected = false;
    }

    uint8_t byte;
    while (drivers->uart.read(Uart::UartPort::Uart1, &byte))
    {
        lastRead = tap::arch::clock::getTimeMilliseconds();

        if (syncState == 0)
        {
            if (byte == 0x20)
            {
                ibusBuffer[0] = byte;
                syncState = 1;
                dataIndex = 1;
            }
        }
        else if (syncState == 1)
        {
            if (byte == 0x40)
            {
                ibusBuffer[1] = byte;
                syncState = 2;
                dataIndex = 2;
            }
            else
            {
                syncState = 0;  // Resync
            }
        }
        else if (syncState == 2)
        {
            ibusBuffer[dataIndex++] = byte;

            if (dataIndex >= IBUS_PACKET_SIZE)
            {
                parsePacket();
                if (getChannel(Channel::SWITCH_D))
                {
                    connected = true;
                }
                else
                {
                    connected = false;
                }
                syncState = 0;
                dataIndex = 0;
            }
        }
    }

    // Check read timeout
    if (tap::arch::clock::getTimeMilliseconds() - lastRead > FLYSKY_READ_TIMEOUT)
    {
        syncState = 0;
        dataIndex = 0;
    }
}

bool FlySky::isConnected() const { return connected; }

float FlySky::getChannel(uint8_t ch) const
{
    if (ch >= MAX_CHANNELS) return 0.0f;

    // iBUS channels: typically ~1000 - 2000 → normalize to 0.0 - 1.0
    float value = (channels[ch] - 1000) / 1000.0f;
    if (ch < 4 && ch > -1)
    {
        value -= .5;
        value *= -2;
    }
    // else if (ch < 8 && ch > 3)
    // {
    //     value *= -1;
    // }
    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;
    switch (ch)
    {
        case Channel::SWITCH_A:
            return KeyValues::F * value;
            break;
        case Channel::SWITCH_B:
            return value;
    }

    return value;
}

uint16_t FlySky::getRawChannel(uint8_t ch) const
{
    if (ch >= MAX_CHANNELS) return 0;
    return channels[ch];
}

void FlySky::parsePacket()
{
    // Parse 14 channels (2 bytes per channel)
    for (uint8_t i = 0; i < MAX_CHANNELS; i++)
    {
        channels[i] = ibusBuffer[2 + i * 2] | (ibusBuffer[3 + i * 2] << 8);
    }
    drivers->uart.discardReceiveBuffer(Uart::UartPort::Uart1);

    drivers->commandMapper.handleKeyStateChange(
        getChannel(Channel::SWITCH_A),
        Remote::SwitchState::UNKNOWN,
        Remote::SwitchState::UNKNOWN,
        getChannel(Channel::SWITCH_B),
        false);

    // (Optional) You can add safety checks here if needed.
}

}  // namespace tap::communication::serial

#endif  // FLY_SKY