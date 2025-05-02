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

 #ifndef CHASSIS_MCB_CAN_COMM_HPP_
 #define CHASSIS_MCB_CAN_COMM_HPP_
 
 #include "tap/architecture/periodic_timer.hpp"
 #include "tap/architecture/timeout.hpp"
 #include "tap/communication/can/can_rx_listener.hpp"
 #include "tap/communication/gpio/digital.hpp"
 
 #include "modm/architecture/interface/register.hpp"
 
 namespace src {
    class Drivers;
}
 
 class ChassisMcbCanComm
 {
 public:
     enum class RxCommandMsgBitmask : uint8_t
     {
         OPEN_HOPPER = modm::Bit0,
         RECALIBRATE_IMU = modm::Bit1,
         TURN_LASER_ON = modm::Bit2,
     };
     MODM_FLAGS8(RxCommandMsgBitmask);
 
     ChassisMcbCanComm(tap::Drivers* drivers);
     DISALLOW_COPY_AND_ASSIGN(ChassisMcbCanComm);
 
     void init();
 
     inline bool getOpenHopperRequested() const
     {
         return commandMsgBitmask.any(RxCommandMsgBitmask::OPEN_HOPPER);
     }
 
     inline bool getImuRecalibrationRequested() const
     {
         return commandMsgBitmask.any(RxCommandMsgBitmask::RECALIBRATE_IMU);
     }
 
     inline bool getLaserOnRequested() const
     {
         return commandMsgBitmask.any(RxCommandMsgBitmask::TURN_LASER_ON);
     }
 
     inline void clearImuRecalibration()
     {
         commandMsgBitmask.reset(RxCommandMsgBitmask::RECALIBRATE_IMU);
     }
 
     inline bool isConnected() const
     {
         return !chassisMcbConnectedTimeout.isExpired() && !chassisMcbConnectedTimeout.isStopped();
     }
 
     inline uint32_t getLeaderTimeMicroseconds()
     {
         return getTimeRelativeToLeaderMicroseconds(tap::arch::clock::getTimeMicroseconds());
     }
 
     inline uint32_t getTimeRelativeToLeaderMicroseconds(uint32_t followerTime)
     {
         return followerTime + lastestSyncData.getCorrelationTimeOffsetMicroseconds();
     }
 
     void sendIMUData();
 
     void sendSynchronizationRequest();
 
     void sendTurretStatusData(tap::gpio::Digital::InputPin limitSwitchPin);
 
 private:
     enum CanIDs
     {
         SYNC_TX_CAN_ID = 0x1f8,
         SYNC_RX_CAN_ID = 0x1f9,
         TURRET_STATUS_TX_CAN_ID = 0x1fa,
         X_AXIS_TX_CAN_ID = 0x1fb,
         Y_AXIS_TX_CAN_ID = 0x1fc,
         Z_AXIS_TX_CAN_ID = 0x1fd,
         CHASSIS_MCB_COMMAND_RX_CAN_ID = 0x1fe,
     };
 
     struct SyncData
     {
         int64_t followerReqTimeUs;
         int64_t leaderReceiveReqTimeUs;
         int64_t leaderResponseTimeUs;
         int64_t followerReceiveResponseTimeUs;
         int64_t getCorrelationTimeOffsetMicroseconds()
         {
             return ((leaderReceiveReqTimeUs - followerReqTimeUs) +
                     (leaderResponseTimeUs - followerReceiveResponseTimeUs)) /
                    2;
         }
     };
 
     struct AxisMessageData
     {
         int16_t angleFixedPoint;
         int16_t angleAngularVelocityRaw;
         int16_t linearAcceleration;
         uint8_t seq;
     } modm_packed;
 
     static constexpr tap::can::CanBus CHASSIS_IMU_CAN_BUS = tap::can::CanBus::CAN_BUS2;
     static constexpr uint32_t DISCONNECT_TIMEOUT_PERIOD = 1000;
     static constexpr uint32_t SYNC_REQUEST_PERIOD = 100;
 
     using CanCommListenerFunc = void (ChassisMcbCanComm::*)(const modm::can::Message& message);
 
     class MainMcbRxHandler : public tap::can::CanRxListener
     {
     public:
         MainMcbRxHandler(
             tap::Drivers* drivers,
             uint32_t id,
             tap::can::CanBus cB,
             ChassisMcbCanComm* msgHandler,
             CanCommListenerFunc funcToCall);
         void processMessage(const modm::can::Message& message) override;
 
     private:
         ChassisMcbCanComm* msgHandler;
         CanCommListenerFunc funcToCall;
     };
 
     tap::Drivers* drivers;
 
     RxCommandMsgBitmask_t commandMsgBitmask;
 
     tap::arch::MilliTimeout chassisMcbConnectedTimeout;
 
     tap::arch::PeriodicMilliTimer timeSyncLoopTimeout{SYNC_REQUEST_PERIOD};
 
     SyncData currProcessingSyncData;
     SyncData lastestSyncData;
 
     MainMcbRxHandler chassisCommandHandler;
     MainMcbRxHandler timeSyncronizationHandler;
 
     uint32_t imuDataSeq = 0;
     uint32_t blinkCounter = 0;
 
     void sendAxisData(
         CanIDs messageID,
         float angle,
         float angularVelocity,
         float linearAcceleration);
 
     void handleChassisCommandMessage(const modm::can::Message& message);
     void handleTimeSynchronizationMessage(const modm::can::Message& message);
 };
 
 #endif  // CHASSIS_MCB_CAN_COMM_HPP_
 