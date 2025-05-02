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

#ifndef DRIVERS_SINGLETON_HPP_
#define DRIVERS_SINGLETON_HPP_

#ifndef ENV_UNIT_TESTS

#include "tap/util_macros.hpp"

#ifdef TARGET_STANDARD
#include "robot/standard/standard_drivers.hpp"
namespace src::standard
#elif TARGET_SENTRY
#include "robot/sentry/sentry_drivers.hpp"
namespace src::sentry
#elif TARGET_HERO
#include "robot/hero/hero_drivers.hpp"
namespace src::hero
#elif TURRET
#include "robot/turret/turret_drivers.hpp"
namespace src::gyro
#endif
{
/**
 * @return The singleton instance of the Drivers class. This is the only instance of the
 *      Drivers class that should be created anywhere in the non-unit test framework.
 * @note It is likely that you will never have to use this. There are only two files you
 *      should be calling this function from -- `main.cpp` and `*_control.cpp`, either to
 *      run I/O stuff and to add a Drivers pointer to an instance of a Subsystem or Command.
 */
Drivers *DoNotUse_getDrivers();
using driversFunc = Drivers *(*)();
}  // namespace src

#endif  // DRIVERS_SINGLETON_HPP_

#endif
