/*
 * Copyright (c) 2020-2021 NorthStart
 *
 * This file is part of NorthStarFleet2025.
 *
 * NorthStarFleet2025 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NorthStarFleet2025 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NorthStarFleet2025.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "robot/robot_control.hpp"

/* robot includes ---------------------------------------------------------*/

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);

#ifdef TARGET_STANDARD
using namespace src::standard;
#elif TARGET_SENTRY
using namespace src::sentry;
#elif TARGET_HERO
using namespace src::hero;
#elif TURRET
#include "communication/can/chassis/chassis_mcb_can_comm.hpp"
using namespace src::gyro;
ChassisMcbCanComm chassisMcbCanComm(DoNotUse_getDrivers());
#endif

// using namespace std::chrono_literals;

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(Drivers *drivers);

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    Drivers *drivers = DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    initSubsystemCommands(drivers);
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->bmi088.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
#ifdef TURRET
            PROFILE(drivers->profiler, chassisMcbCanComm.sendIMUData, ());
            PROFILE(drivers->profiler, chassisMcbCanComm.sendSynchronizationRequest, ());
#else
            PROFILE(drivers->profiler, drivers->turretMCBCanCommBus2.sendData, ());
#if defined(TARGET_STANDARD) || defined(TARGET_SENTRY)
            PROFILE(drivers->profiler, drivers->revMotorTxHandler.encodeAndSendCanData, ());
#endif

            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
#endif
        }
        // if(!drivers->turretMCBCanCommBus2.isConnected()){
        //     std::cout<<"poop";
        // }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(Drivers *drivers)
{
    drivers->can.initialize();
    drivers->leds.init();
    drivers->digital.init();
    drivers->pwm.init();
    drivers->bmi088.initialize(500, 0.1, 0);
    drivers->errorController.init();
    drivers->terminalSerial.initialize();
#if defined(TARGET_STANDARD) || defined(TARGET_HERO)
    drivers->turretMCBCanCommBus2.init();
#endif
#ifdef TURRET
    chassisMcbCanComm.init();
#else
    drivers->analog.init();
    drivers->remote.initialize();
    drivers->refSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
#endif
}

static void updateIo(Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->bmi088.read();
#ifndef TURRET
    drivers->refSerial.updateSerial();
    drivers->remote.read();
#endif
}