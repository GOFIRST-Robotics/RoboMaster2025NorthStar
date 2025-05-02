#ifdef TARGET_HERO

#ifndef FLYWHEEL_RUN_COMMAND
#define FLYWHEEL_RUN_COMMAND

#include "tap/control/command.hpp"

#include "robot/hero/hero_flywheel_subsystem.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::control::flywheel
{
class HeroFlywheelRunCommand : public tap::control::Command
{
public:
    HeroFlywheelRunCommand(HeroFlywheelSubsystem *flywheel);

    const char *getName() const override { return "Hero Flywheel Run Command"; }

    void initialize() override;

    void execute() override {}

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    HeroFlywheelSubsystem *flywheel;
};
}  // namespace src::control::flywheel
#endif  // FLYWHEEL_RUN_COMMAND

#endif  // TARGET_HERO