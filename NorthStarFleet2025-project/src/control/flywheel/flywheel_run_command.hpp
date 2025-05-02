#ifndef FLYWHEEL_RUN_COMMAND
#define FLYWHEEL_RUN_COMMAND

#include "tap/control/command.hpp"

#include "control/flywheel/flywheel_subsystem.hpp"

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
class FlywheelRunCommand : public tap::control::Command
{
public:
    FlywheelRunCommand(FlywheelSubsystem *flywheel);

    const char *getName() const override { return "Flywheel Run Command"; }

    void initialize() override;

    void execute() override {}

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    FlywheelSubsystem *flywheel;
};
}  // namespace src::control::flywheel
#endif