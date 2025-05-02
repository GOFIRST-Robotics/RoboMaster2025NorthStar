/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-turret-mcb.
 *
 * aruw-turret-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-turret-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-turret-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "chassis_mcb_can_comm.hpp"

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/drivers.hpp"

#include "modm/architecture/interface/can_message.hpp"

ChassisMcbCanComm::ChassisMcbCanComm(tap::Drivers* drivers)
    : drivers(drivers),
      commandMsgBitmask(),
      chassisCommandHandler(
          drivers,
          CHASSIS_MCB_COMMAND_RX_CAN_ID,
          CHASSIS_IMU_CAN_BUS,
          this,
          &ChassisMcbCanComm::handleChassisCommandMessage),
      timeSyncronizationHandler(
          drivers,
          SYNC_RX_CAN_ID,
          CHASSIS_IMU_CAN_BUS,
          this,
          &ChassisMcbCanComm::handleTimeSynchronizationMessage)
{
}

void ChassisMcbCanComm::init()
{
    chassisCommandHandler.attachSelfToRxHandler();
    timeSyncronizationHandler.attachSelfToRxHandler();
}

void ChassisMcbCanComm::handleChassisCommandMessage(const modm::can::Message& message)
{
    commandMsgBitmask.value = message.data[0];
    chassisMcbConnectedTimeout.restart(DISCONNECT_TIMEOUT_PERIOD);
}

void ChassisMcbCanComm::handleTimeSynchronizationMessage(const modm::can::Message& message)
{
    uint32_t masterSentTime = *reinterpret_cast<const uint32_t*>(message.data);
    currProcessingSyncData.followerReceiveResponseTimeUs = tap::arch::clock::getTimeMicroseconds();
    currProcessingSyncData.leaderReceiveReqTimeUs = masterSentTime;
    currProcessingSyncData.leaderResponseTimeUs = masterSentTime;

    // short circuit if too much time between request and response or if the request time was
    // somehow larger than the response time (i.e. clock wrapping)
    if (currProcessingSyncData.followerReceiveResponseTimeUs <
            currProcessingSyncData.followerReqTimeUs ||
        currProcessingSyncData.followerReceiveResponseTimeUs -
                currProcessingSyncData.followerReqTimeUs >
            1000 * SYNC_REQUEST_PERIOD / 2)
    {
        return;
    }

    lastestSyncData = currProcessingSyncData;
}

void ChassisMcbCanComm::sendSynchronizationRequest()
{
    if (timeSyncLoopTimeout.execute() && drivers->can.isReadyToSend(CHASSIS_IMU_CAN_BUS))
    {
        modm::can::Message message(SYNC_TX_CAN_ID, 0);
        message.setExtended(false);
        currProcessingSyncData.followerReqTimeUs = tap::arch::clock::getTimeMicroseconds();
        drivers->can.sendMessage(CHASSIS_IMU_CAN_BUS, message);
    }
}

void ChassisMcbCanComm::sendTurretStatusData(tap::gpio::Digital::InputPin limitSwitchPin)
{
    if (drivers->can.isReadyToSend(CHASSIS_IMU_CAN_BUS))
    {
        bool limitSwitchDepressed = drivers->digital.read(limitSwitchPin);

        modm::can::Message msg(TURRET_STATUS_TX_CAN_ID, 1);
        msg.setExtended(false);

        msg.data[0] = static_cast<uint8_t>(limitSwitchDepressed) & 0b1;

        drivers->can.sendMessage(CHASSIS_IMU_CAN_BUS, msg);
    }
}

void ChassisMcbCanComm::sendAxisData(
    CanIDs messageID,
    float angle,
    float angularVelocity,
    float linearAcceleration)
{
    // Maps a full rotation to the size of a uint16 (min angle measured ~= 6E-3 deg)
    static constexpr float ANGLE_FIXED_POINT_PRECISION = 360.0f / UINT16_MAX;

    // Convert acceleration to cm/s^2 (max acceleration magnitude ~ 320 m/s^2 ~ 32.5*g)
    static constexpr float MPS2_TO_CMPS2 = 100.0;

    static constexpr uint8_t AXIS_MESSAGE_LENGTH = 7;  // (bytes)

    modm::can::Message axisMessage(messageID, AXIS_MESSAGE_LENGTH);
    axisMessage.setExtended(false);

    AxisMessageData* axisData = reinterpret_cast<AxisMessageData*>(axisMessage.data);
    axisData->angleFixedPoint =
        tap::algorithms::ContiguousFloat(angle, -180.0f, 180.0f).getValue() /
        ANGLE_FIXED_POINT_PRECISION;
    axisData->angleAngularVelocityRaw =
        angularVelocity / tap::communication::sensors::imu::bmi088::Bmi088::GYRO_DS_PER_GYRO_COUNT;
    axisData->linearAcceleration = linearAcceleration * MPS2_TO_CMPS2;
    axisData->seq = imuDataSeq;

    drivers->can.sendMessage(CHASSIS_IMU_CAN_BUS, axisMessage);
}

void ChassisMcbCanComm::sendIMUData()
{
    using namespace tap::communication::sensors::imu::bmi088;
    const Bmi088::ImuState imuState = drivers->bmi088.getImuState();

    if (getImuRecalibrationRequested())
    {
        drivers->bmi088.requestRecalibration();
    }

    if (imuState == Bmi088::ImuState::IMU_CALIBRATING)
    {
        clearImuRecalibration();
    }

    if ((imuState == Bmi088::ImuState::IMU_CALIBRATED ||
         imuState == Bmi088::ImuState::IMU_NOT_CALIBRATED) &&
        drivers->can.isReadyToSend(CHASSIS_IMU_CAN_BUS))
    {
        drivers->leds.set(tap::gpio::Leds::Green, blinkCounter < 50);
        blinkCounter = (blinkCounter + 1) % 100;

        sendAxisData(
            X_AXIS_TX_CAN_ID,
            drivers->bmi088.getRoll(),
            drivers->bmi088.getGx(),
            drivers->bmi088.getAx());
        sendAxisData(
            Y_AXIS_TX_CAN_ID,
            drivers->bmi088.getPitch(),
            drivers->bmi088.getGy(),
            drivers->bmi088.getAy());
        sendAxisData(
            Z_AXIS_TX_CAN_ID,
            drivers->bmi088.getYaw(),
            drivers->bmi088.getGz(),
            drivers->bmi088.getAz());

        imuDataSeq++;
    }
    else
    {
        std::cout << "opopopo";
    }
}

ChassisMcbCanComm::MainMcbRxHandler::MainMcbRxHandler(
    tap::Drivers* drivers,
    uint32_t id,
    tap::can::CanBus cB,
    ChassisMcbCanComm* msgHandler,
    CanCommListenerFunc funcToCall)
    : CanRxListener(drivers, id, cB),
      msgHandler(msgHandler),
      funcToCall(funcToCall)
{
}

void ChassisMcbCanComm::MainMcbRxHandler::processMessage(const modm::can::Message& message)
{
    (msgHandler->*funcToCall)(message);
}
