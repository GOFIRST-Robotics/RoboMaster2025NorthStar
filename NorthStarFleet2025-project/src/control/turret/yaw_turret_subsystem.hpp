/*
 * Copyright (c) 2021-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef YAW_TURRET_SUBSYSTEM_HPP_
#define YAW_TURRET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "turret_motor.hpp"

namespace src::control::turret
{
/**
 *
 * A turret subystem that is only capable of yawing.
 *
 * Stores software necessary for interacting with a gimbal that control the
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 *
 * All angles computed using a right hand coordinate system. In other words, yaw is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from above.
 */
class YawTurretSubsystem final : public tap::control::Subsystem
{
public:
    YawTurretSubsystem(
        tap::Drivers& drivers,
        tap::motor::MotorInterface& yawMotor,
        const src::control::turret::TurretMotorConfig& yawMotorConfig);

    void refresh();

    void initialize();

    float getChassisYaw() const;

    // @note: these are here to differentiate between a client wants to control the motor vs. just
    // read values from it. If we decide that we're fine using const / non-const references to the
    // motor, these can be removed
    inline src::control::turret::TurretMotor& getMutableMotor() { return yawMotor; }
    inline const src::control::turret::TurretMotor& getReadOnlyMotor() const
    {
        return yawMotor;
    }

    void refreshSafeDisconnect() override { yawMotor.setMotorOutput(0); }

private:
    src::control::turret::TurretMotor yawMotor;
};  // class YawTurretSubsystem

}  // namespace src::control::turret
#endif  // YAW_TURRET_SUBSYSTEM_HPP_
