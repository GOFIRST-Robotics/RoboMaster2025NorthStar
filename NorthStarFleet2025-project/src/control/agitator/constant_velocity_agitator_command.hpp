/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CONSTANT_VELOCITY_AGITATOR_COMMAND_HPP_
#define CONSTANT_VELOCITY_AGITATOR_COMMAND_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"

using namespace tap::algorithms;

namespace src::control::agitator
{
/**
 * A command that aims to keep the agitator at a constant velocity. At the end of this command,
 * the agitator will move to a specified setpoint, as to give a consistent starting point for each
 * shot.
 *
 * Ends if the agitator is offline or jammed.
 */

class ConstantVelocityAgitatorCommand : public tap::control::setpoint::MoveIntegralCommand
{
public:
    ConstantVelocityAgitatorCommand(
        tap::control::setpoint::IntegrableSetpointSubsystem& integrableSetpointSubsystem,
        const Config& config)
        : tap::control::setpoint::MoveIntegralCommand(integrableSetpointSubsystem, config)
    {
    }

    const char* getName() const override { return "Constant velocity agitator command"; }

    bool isFinished() const
    {
        // The subsystem is jammed or offline or it is within the setpoint tolerance
        return integrableSetpointSubsystem.isJammed() || !integrableSetpointSubsystem.isOnline() ||
               (useSingleShotMode && targetIntegralReached());
    };

    void enableConstantRotation(bool constantRotation)
    {
        bool previousModeWasConstantRotation = !useSingleShotMode;
        useSingleShotMode = !constantRotation;
        if (useSingleShotMode && previousModeWasConstantRotation)
        {
            if (getSign(config.targetIntegralChange) == -1)
            {
                while (finalTargetIntegralSetpoint >
                       integrableSetpointSubsystem.getCurrentValueIntegral())
                {
                    finalTargetIntegralSetpoint -= config.targetIntegralChange;
                }
            }
            else
            {
                while (finalTargetIntegralSetpoint <
                       integrableSetpointSubsystem.getCurrentValueIntegral())
                {
                    finalTargetIntegralSetpoint += config.targetIntegralChange;
                }
            }
        }
    }

protected:
    bool useSingleShotMode = true;
};

}  // namespace src::control::agitator
#endif  // CONSTANT_VELOCITY_AGITATOR_COMMAND_HPP_
