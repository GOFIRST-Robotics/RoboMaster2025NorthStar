#ifndef HERO_FLYWHEEL_ON_GOVERNOR_HPP_
#define HERO_FLYWHEEL_ON_GOVERNOR_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

#include "control/agitator/velocity_agitator_subsystem.hpp"
#include "robot/hero/hero_flywheel_subsystem.hpp"

namespace src::control::governor
{
class HeroFlywheelOnGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    HeroFlywheelOnGovernor(src::control::flywheel::HeroFlywheelSubsystem &flywheel)
        : flywheel(flywheel)
    {
    }

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
            // Down
            (!tap::algorithms::compareFloatClose(flywheel.getDesiredFlywheelSpeedDown(), .0f, 1) &&
             flywheel.getCurrentDownFlywheelMotorRPM() >=
                 flywheel.getDesiredFlywheelSpeedDown() * MINIMUM_SPEED_THRESHOLD_FRACTION &&
             flywheel.getCurrentDownFlywheelMotorRPM() <=
                 flywheel.getDesiredFlywheelSpeedDown() * MAXIMUM_SPEED_THRESHOLD_FRACTION);
    }

    bool isFinished() final { return !isReady(); }

private:
    src::control::flywheel::HeroFlywheelSubsystem &flywheel;

    static constexpr float MINIMUM_SPEED_THRESHOLD_FRACTION = 0.9;
    static constexpr float MAXIMUM_SPEED_THRESHOLD_FRACTION = 1.02;
};
}  // namespace src::control::governor

#endif  // HERO_FLYWHEEL_ON_GOVERNOR_HPP_
