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
    // uint16_t encoderWrapped,
    // int64_t encoderRevolutions
    )
    : CanRxListener(drivers, static_cast<uint32_t>(desMotorIdentifier), motorCanBus),
      motorName(name),
      drivers(drivers),
      motorIdentifier(desMotorIdentifier),
      motorCanBus(motorCanBus),
      desiredOutput(0),
    //   shaftRPM(0),
    //   temperature(0),
    //   torque(0),
      motorInverted(isInverted),
      targetVoltage(0)
    //   encoderWrapped(encoderWrapped),
    //   encoderRevolutions(encoderRevolutions)
{
    // motorDisconnectTimeout.stop();
}

void RevMotor::initialize()
{
    drivers->revMotorTxHandler.addMotorToManager(this);
    // attachSelfToRxHandler();
}

// void RevMotor::processMessage(const modm::can::Message& message)
// {
//     if (message.getIdentifier() != RevMotor::getMotorIdentifier())
//     {
//         return;
//     }
//     uint16_t encoderActual =
//         static_cast<uint16_t>(message.data[0] << 8 | message.data[1]);        // encoder value
//     shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
//     shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
//     torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
//     torque = motorInverted ? -torque : torque;
//     temperature = static_cast<int8_t>(message.data[6]);  // temperature

//     // restart disconnect timer, since you just received a message from the motor
//     motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

//     // invert motor if necessary
//     encoderActual = motorInverted ? ENC_RESOLUTION - 1 - encoderActual : encoderActual;
//     updateEncoderValue(encoderActual);
// }

void RevMotor::setDesiredOutput(int32_t desiredOutput)
{
    int16_t desOutputNotInverted =
        static_cast<int16_t>(tap::algorithms::limitVal<int32_t>(desiredOutput, SHRT_MIN, SHRT_MAX));
    this->desiredOutput = motorInverted ? -desOutputNotInverted : desOutputNotInverted;
}

void RevMotor::setTargetVoltage(float targetVoltage)
{
    this->targetVoltage = targetVoltage;
}




// bool RevMotor::isMotorOnline() const
// {
//     /*
//      * motor online if the disconnect timout has not expired (if it received message but
//      * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout
//      * is stopped)
//      */
//     return !motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped();
// }

void RevMotor::serializeCanSendData(modm::can::Message* txMessage) const
{
    //target speed 8000
    // std::memcpy(txMessage->data, &targetVoltage, sizeof(targetVoltage));
    //TODO: is this little endian or big endian?
    uint32_t floatAsInt;
    std::memcpy(&floatAsInt, &targetVoltage, sizeof(float));
    txMessage->data[3] = static_cast<uint8_t>(floatAsInt & 0xFF);
    txMessage->data[2] = static_cast<uint8_t>((floatAsInt >> 8) & 0xFF);
    txMessage->data[1] = static_cast<uint8_t>((floatAsInt >> 16) & 0xFF);
    txMessage->data[0] = static_cast<uint8_t>((floatAsInt >> 24) & 0xFF);
    txMessage->data[4] = 0;
    txMessage->data[5] = 0;
    txMessage->data[6] = 0;
    txMessage->data[7] = 0;



    // txMessage->data[4] = static_cast<uint8_t>(floatAsInt & 0xFF);
    // txMessage->data[5] = static_cast<uint8_t>((floatAsInt >> 8) & 0xFF);
    // txMessage->data[6] = static_cast<uint8_t>((floatAsInt >> 16) & 0xFF);
    // txMessage->data[7] = static_cast<uint8_t>((floatAsInt >> 24) & 0xFF);
    // txMessage->data[0] = 0;
    // txMessage->data[0] = 0;
    // txMessage->data[0] = 0;
    // txMessage->data[0] = 0;

    // txMessage->data[0] = static_cast<uint8_t>(floatAsInt & 0xFF);
    // txMessage->data[1] = static_cast<uint8_t>((floatAsInt >> 8) & 0xFF);
    // txMessage->data[2] = static_cast<uint8_t>((floatAsInt >> 16) & 0xFF);
    // txMessage->data[3] = static_cast<uint8_t>((floatAsInt >> 24) & 0xFF);
    // txMessage->data[4] = 0;
    // txMessage->data[5] = 0;
    // txMessage->data[6] = 0;
    // txMessage->data[7] = 0;



    // txMessage->data[0] = 0;
    // txMessage->data[1] = 0; 
    // txMessage->data[2] = 0;
    // txMessage->data[3] = 0;
    // txMessage->data[4] = static_cast<uint8_t>(floatAsInt & 0xFF);
    // txMessage->data[5] = static_cast<uint8_t>((floatAsInt >> 8) & 0xFF);
    // txMessage->data[6] = static_cast<uint8_t>((floatAsInt >> 16) & 0xFF);
    // txMessage->data[7] = static_cast<uint8_t>((floatAsInt >> 24) & 0xFF);



    // txMessage->data[0] = 0;
    // txMessage->data[1] = 0; 
    // txMessage->data[2] = 0;
    // txMessage->data[3] = 0;
    // txMessage->data[4] = 0;
    // txMessage->data[5] = 0;
    // txMessage->data[6] = 0;
    // txMessage->data[7] = 0;
    int debug = 0;
}

// getter functions
int16_t RevMotor::getOutputDesired() const { return desiredOutput; }

uint32_t RevMotor::getMotorIdentifier() const { return motorIdentifier; }

// int8_t RevMotor::getTemperature() const { return temperature; }

// int16_t RevMotor::getTorque() const { return torque; }

// int16_t RevMotor::getShaftRPM() const { return shaftRPM; }

// bool RevMotor::isMotorInverted() const { return fas; }

tap::can::CanBus RevMotor::getCanBus() const { return motorCanBus; }

const char* RevMotor::getName() const { return motorName; }

// int64_t RevMotor::getEncoderUnwrapped() const
// {
//     return static_cast<int64_t>(encoderWrapped) +
//            static_cast<int64_t>(ENC_RESOLUTION) * encoderRevolutions;
// }

// uint16_t RevMotor::getEncoderWrapped() const { return encoderWrapped; }

// void RevMotor::updateEncoderValue(uint16_t newEncWrapped)
// {
//     int16_t enc_dif = newEncWrapped - encoderWrapped;
//     if (enc_dif < -ENC_RESOLUTION / 2)
//     {
//         encoderRevolutions++;
//     }
//     else if (enc_dif > ENC_RESOLUTION / 2)
//     {
//         encoderRevolutions--;
//     }
//     encoderWrapped = newEncWrapped;
// }
}  // namespace motor

}  // namespace tap
