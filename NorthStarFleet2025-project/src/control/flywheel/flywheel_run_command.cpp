#ifndef TARGET_HERO

#include "flywheel_run_command.hpp"

namespace src::control::flywheel
{
FlywheelRunCommand::FlywheelRunCommand(FlywheelSubsystem *flywheel) : flywheel(flywheel)
{
    addSubsystemRequirement(flywheel);
}

void FlywheelRunCommand::initialize()
{
    flywheel->setDesiredSpin(90);
    flywheel->setDesiredLaunchSpeed(18);
}

void FlywheelRunCommand::end(bool interrupted) { flywheel->setDesiredLaunchSpeed(0); }

}  // namespace src::control::flywheel

#endif  // TARGET_HERO