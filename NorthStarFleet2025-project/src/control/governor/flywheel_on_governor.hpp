/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FLYWHEEL_ON_GOVERNOR_HPP_
#define FLYWHEEL_ON_GOVERNOR_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

#include "control/agitator/velocity_agitator_subsystem.hpp"
#include "control/flywheel/flywheel_subsystem.hpp"

namespace src::control::governor
{
/**
 * Governor that allows one to gate a command from running when the actual, average friction wheel
 * speed isn't above a certain threshold.
 *
 * Useful for disallowing the agitator from rotating while friction wheels are not on.
 */
class FlywheelOnGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] flywheel Reference to the friction wheel subsystem being used in the
     * governor's behavior.
     */
    FlywheelOnGovernor(src::control::flywheel::FlywheelSubsystem &flywheel) : flywheel(flywheel) {}

    bool isReady() final
    {
        return
            // left
            (!tap::algorithms::compareFloatClose(flywheel.getDesiredFlywheelSpeedLeft(), .0f, 1) &&
             flywheel.getCurrentLeftFlywheelMotorRPM() >=
                 flywheel.getDesiredFlywheelSpeedLeft() * MINIMUM_SPEED_THRESHOLD_FRACTION &&
             flywheel.getCurrentLeftFlywheelMotorRPM() <=
                 flywheel.getDesiredFlywheelSpeedLeft() * MAXIMUM_SPEED_THRESHOLD_FRACTION) &&
            // right
            (!tap::algorithms::compareFloatClose(flywheel.getDesiredFlywheelSpeedRight(), .0f, 1) &&
             flywheel.getCurrentRightFlywheelMotorRPM() >=
                 flywheel.getDesiredFlywheelSpeedRight() * MINIMUM_SPEED_THRESHOLD_FRACTION &&
             flywheel.getCurrentRightFlywheelMotorRPM() <=
                 flywheel.getDesiredFlywheelSpeedRight() * MAXIMUM_SPEED_THRESHOLD_FRACTION) &&
            // up
            (!tap::algorithms::compareFloatClose(flywheel.getDesiredFlywheelSpeedUp(), .0f, 1) &&
             flywheel.getCurrentUpFlywheelMotorRPM() >=
                 flywheel.getDesiredFlywheelSpeedUp() * MINIMUM_SPEED_THRESHOLD_FRACTION &&
             flywheel.getCurrentUpFlywheelMotorRPM() <=
                 flywheel.getDesiredFlywheelSpeedUp() * MAXIMUM_SPEED_THRESHOLD_FRACTION);
    }

    bool isFinished() final { return !isReady(); }

private:
    src::control::flywheel::FlywheelSubsystem &flywheel;

    static constexpr float MINIMUM_SPEED_THRESHOLD_FRACTION = 0.9;
    static constexpr float MAXIMUM_SPEED_THRESHOLD_FRACTION = 1.02;
};
}  // namespace src::control::governor

#endif  // FLYWHEEL_ON_GOVERNOR_HPP_
