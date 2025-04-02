/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "rev_motor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#ifdef PLATFORM_HOSTED
#include <iostream>

#include "tap/communication/tcp-server/json_messages.hpp"
#include "tap/communication/tcp-server/tcp_server.hpp"

#include "modm/architecture/interface/can_message.hpp"
#endif

namespace tap
{
namespace motor
{
RevMotor::~RevMotor() { drivers->revMotorTxHandler.removeFromMotorManager(*this); }

RevMotor::RevMotor(
    Drivers* drivers,
    REVMotorId desMotorIdentifier,
    tap::can::CanBus motorCanBus,
    bool isInverted,
    const char* name
    )
    : CanRxListener(drivers, static_cast<uint32_t>(desMotorIdentifier), motorCanBus),
      motorName(name),
      drivers(drivers),
      motorIdentifier(desMotorIdentifier),
      motorCanBus(motorCanBus),
      desiredOutput(0),
      motorInverted(isInverted),
      targetVoltage(0)
{
    // motorDisconnectTimeout.stop();
}

void RevMotor::initialize()
{
    drivers->revMotorTxHandler.addMotorToManager(this);
}

// Add these implementations to rev_motor.cpp

void RevMotor::setControlMode(ControlMode mode)
{
    currentControlMode = mode;
}

RevMotor::ControlMode RevMotor::getControlMode() const
{
    return currentControlMode;
}

void RevMotor::setControlValue(float value)
{
    controlValue = value;
    
    // If you want backward compatibility with existing voltage control:
    if (currentControlMode == ControlMode::VOLTAGE) {
        setTargetVoltage(value);
    }
}

float RevMotor::getControlValue() const
{
    return controlValue;
}

// Modify serializeCanSendData to handle the different control modes
void RevMotor::serializeCanSendData(modm::can::Message* txMessage) const
{
    // Copy the control value to the message
    std::memcpy(&txMessage->data[0], &controlValue, sizeof(controlValue));
    
    // Zero out the remaining bytes
    for (int i = sizeof(controlValue); i < 8; i++) {
        txMessage->data[i] = 0;
    }
}

// void RevMotor::serializeCanSendData(modm::can::Message* txMessage) const
// {
//     std::memcpy(&txMessage->data[0], &targetVoltage, sizeof(targetVoltage));
//     int debug = txMessage->data[0];
// }

uint32_t RevMotor::getMotorIdentifier() const { return motorIdentifier; }

tap::can::CanBus RevMotor::getCanBus() const { return motorCanBus; }

const char* RevMotor::getName() const { return motorName; }

}  // namespace motor

}  // namespace tap
