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
    Drivers* drivers,    REVMotorId desMotorIdentifier,
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

void RevMotor::processMessage(const modm::can::Message& message)
{
    if (message.getIdentifier() != RevMotor::getMotorIdentifier())
    {
        return;
    }
    uint16_t encoderActual =
        static_cast<uint16_t>(message.data[0] << 8 | message.data[1]);        // encoder value
    // shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
    // shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
    // torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
    // torque = motorInverted ? -torque : torque;
    // temperature = static_cast<int8_t>(message.data[6]);  // temperature

    //TODO: Reimplement encoder functionality
}

// Add these implementations to rev_motor.cpp
APICommand RevMotor::controlModeToAPI(ControlMode mode){
    if(mode == ControlMode::DUTY_CYCLE){
        return APICommand::DutyCycle;
    } else if(mode == ControlMode::VELOCITY){
        return APICommand::Velocity;
    } else if(mode == ControlMode::POSITION){
        return APICommand::Position;
    } else if(mode == ControlMode::VOLTAGE){
        return APICommand::Voltage;
    } else if(mode == ControlMode::CURRENT){
        return APICommand::Current;
    }
  
}

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
        // setTargetVoltage(value);
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

void RevMotor::serializeRevMotorHeartBeat(modm::can::Message* message)
{
    for (int i = 0; i < 8; i++)
    {
        message->data[i] = 0xFF;
    }
}

/**
 * constructs a can message for the given REV motor by using the motor's id and the 
 * desired control mode id to tell the the motor what method of control you want to 
 * use. the control modes can be found in a REV SW Spark Max google sheet which can be
 * obtained by emailing REV Robotics.
 */
modm::can::Message RevMotor::createRevCanControlMessage(APICommand cmd, const RevMotor* motor) {
    uint32_t RevArbitrationId = CreateArbitrationControlId(cmd, motor);
    //the number of bytes in the message
    uint8_t canRevIdLength = 8;
    modm::can::Message canMessage(
        RevArbitrationId,
        canRevIdLength,
        0,
        true);
    return canMessage;
}

uint32_t RevMotor::CreateArbitrationControlId(APICommand cmd, const RevMotor* motor) const
{
  uint8_t apiClass = GetAPIClass(cmd);
  uint8_t apiIndex = GetAPIIndex(cmd);
  uint8_t deviceId = motor->getMotorIdentifier();

  

  return (static_cast<uint32_t>(0x02) << 24) | (static_cast<uint32_t>(0x05) << 16) |
         (static_cast<uint32_t>(apiClass) << 10) | (static_cast<uint32_t>(apiIndex) << 6) |
         static_cast<uint32_t>(deviceId);
}

modm::can::Message RevMotor::createRevCanParameterMessage(Parameter param, const RevMotor* motor) {
    uint32_t RevArbitrationId = CreateArbitrationParameterId(param, motor);
    //the number of bytes in the message
    uint8_t canRevIdLength = 8;
    modm::can::Message canMessage(
        RevArbitrationId,
        canRevIdLength,
        0,
        true);
    return canMessage;
}

uint32_t RevMotor::CreateArbitrationParameterId(Parameter param, const RevMotor* motor) const
{
  uint8_t deviceId = motor->getMotorIdentifier();

  

  return (static_cast<uint32_t>(0x02) << 24) | (static_cast<uint32_t>(0x05) << 16) |
         (static_cast<uint32_t>(48) <<
         10) | (static_cast<uint32_t>(param) << 6) | static_cast<uint32_t>(deviceId);
}

uint8_t RevMotor::GetAPIClass(APICommand cmd) const
{
  return static_cast<uint8_t>(static_cast<uint16_t>(cmd) >> 4);
}

uint8_t RevMotor::GetAPIIndex(APICommand cmd) const
{
  return static_cast<uint8_t>(static_cast<uint16_t>(cmd) & 0x0F);
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
