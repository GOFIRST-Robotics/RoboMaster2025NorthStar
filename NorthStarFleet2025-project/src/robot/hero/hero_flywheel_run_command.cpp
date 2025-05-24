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
    flywheel->setDesiredSpin(110);
    flywheel->setDesiredLaunchSpeed(15.0f);
}

void HeroFlywheelRunCommand::end(bool interrupted) { flywheel->setDesiredLaunchSpeed(0.0f); }

}  // namespace src::control::flywheel

#endif  // TARGET_HERO