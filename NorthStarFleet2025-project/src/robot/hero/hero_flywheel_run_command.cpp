#ifdef TARGET_HERO

#include "hero_flywheel_run_command.hpp"

namespace src::control::flywheel
{
HeroFlywheelRunCommand::HeroFlywheelRunCommand(HeroFlywheelSubsystem *flywheel) : flywheel(flywheel)
{
    addSubsystemRequirement(flywheel);
}

void HeroFlywheelRunCommand::initialize()
{
    flywheel->setDesiredLaunchSpeedLeft(2000.0f);
    flywheel->setDesiredLaunchSpeedRight(2000.0f);
    flywheel->setDesiredLaunchSpeedDown(2000.0f);
}

void HeroFlywheelRunCommand::end(bool interrupted)
{
    flywheel->setDesiredLaunchSpeedLeft(0);
    flywheel->setDesiredLaunchSpeedRight(0);
    flywheel->setDesiredLaunchSpeedDown(0);
}

}  // namespace src::control::flywheel

#endif  // TARGET_HERO