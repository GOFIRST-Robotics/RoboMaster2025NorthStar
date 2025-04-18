/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ROBOT_TURRET_SUBSYSTEM_HPP_
#define ROBOT_TURRET_SUBSYSTEM_HPP_

#include "control/turret/turret_orientation_interface.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace src::control::turret
{
/**
 * Subsystem that must be extended. Extends both the TurretSubsystem and TurretOrientationInterface.
 */
class RobotTurretSubsystem : public src::control::turret::TurretSubsystem,
                             public src::control::turret::TurretOrientationInterface
{
    using TurretSubsystem::TurretSubsystem;
};
}  // namespace src::control::turret

#endif  // ROBOT_TURRET_SUBSYSTEM_HPP_
