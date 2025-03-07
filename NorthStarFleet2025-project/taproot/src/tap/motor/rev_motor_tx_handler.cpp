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

#include "rev_motor_tx_handler.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/assert.h"
#include "modm/architecture/interface/can_message.hpp"

namespace tap::motor
{
void RevMotorTxHandler::addMotorToManager(RevMotor** canMotorStore, RevMotor* const motor)
{
    assert(motor != nullptr);
    uint32_t idIndex = motor->getMotorIdentifier();
    bool motorOverloaded = canMotorStore[idIndex] != nullptr;
    bool motorOutOfBounds = idIndex >= REV_MOTORS_PER_CAN;
    modm_assert(!motorOverloaded && !motorOutOfBounds, "RevMotorTxHandler", "overloading");
    canMotorStore[idIndex] = motor;
}

void RevMotorTxHandler::addMotorToManager(RevMotor* motor)
{
    // add new motor to either the can1 or can2 motor store
    // because we checked to see if the motor is overloaded, we will
    // never have to worry about overfilling the CanxMotorStore array
    if (motor->getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        addMotorToManager(can1MotorStore, motor);
    }
    else
    {
        addMotorToManager(can2MotorStore, motor);
    }
}

void RevMotorTxHandler::encodeAndSendCanData()
{
    // set up new can messages to be sent via CAN bus 1 and 2

    bool can1ValidMotorMessage = true;
    bool can2ValidMotorMessage = true;

    modm::can::Message can1Message = createRevCanMessage(0x2051080, can1MotorStore[1]);


    serializeMotorStoreSendData(
        can1MotorStore,
        &can1Message);


    modm::can::Message can2Message = createRevCanMessage(0x2051080, can1MotorStore[1]);

    serializeMotorStoreSendData(
        can2MotorStore,
        &can2Message);


    bool messageSuccess = true;

    if (drivers->can.isReadyToSend(can::CanBus::CAN_BUS1))
    {
        if (can1ValidMotorMessage)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS1, can1Message);
        }
    }
    if (drivers->can.isReadyToSend(can::CanBus::CAN_BUS2))
    {
        if (can2ValidMotorMessage)
        {
            messageSuccess &= drivers->can.sendMessage(can::CanBus::CAN_BUS2, can2Message);
        }
    }

    if (!messageSuccess)
    {
        RAISE_ERROR(drivers, "sendMessage failure");
    }
}

void RevMotorTxHandler::serializeMotorStoreSendData(
    RevMotor** canMotorStore, modm::can::Message* message)
{

    for (int i = 0; i < REV_MOTORS_PER_CAN; i++)
    {
        const RevMotor* const motor = canMotorStore[i];
        motor->serializeCanSendData(message);
    }
}

void RevMotorTxHandler::removeFromMotorManager(const RevMotor& motor)
{
    if (motor.getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        removeFromMotorManager(motor, can1MotorStore);
    }
    else
    {
        removeFromMotorManager(motor, can2MotorStore);
    }
}

void RevMotorTxHandler::removeFromMotorManager(const RevMotor& motor, RevMotor** motorStore)
{
    uint32_t id = motor.getMotorIdentifier();
    if (id > tap::motor::REV_MOTOR8 || motorStore[id] == nullptr)
    {
        RAISE_ERROR(drivers, "invalid motor id");
        return;
    }
    motorStore[id] = nullptr;
}

RevMotor const* RevMotorTxHandler::getCan1Motor(REVMotorId motorId)
{
    uint32_t index = motorId;
    return index > tap::motor::REV_MOTOR8 ? nullptr : can1MotorStore[index];
}

RevMotor const* RevMotorTxHandler::getCan2Motor(REVMotorId motorId)
{
    uint32_t index = motorId;
    return index > tap::motor::MOTOR8 ? nullptr : can2MotorStore[index];
}









/**
 * constructs a can message for the given REV motor by using the motor's id and the 
 * desired control mode id to tell the the motor what method of control you want to 
 * use. the control modes can be found in a REV SW Spark Max google sheet which can be
 * obtained by emailing REV Robotics.
 */
modm::can::Message RevMotorTxHandler::createRevCanMessage(u_int32_t controlModeID, const RevMotor* motor) {
    uint32_t canVoltageArbitrationID = controlModeID | motor->getMotorIdentifier();
    //the number of bytes in the message
    uint8_t canRevIdLength = 8;
    modm::can::Message canMessage(
        canVoltageArbitrationID,
        canRevIdLength,
        0,
        true);
    return canMessage;
}

















}  // namespace tap::motor
