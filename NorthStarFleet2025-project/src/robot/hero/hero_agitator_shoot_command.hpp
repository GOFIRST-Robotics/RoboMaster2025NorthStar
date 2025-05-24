#ifndef HERO_AGITATOR_SHOOT_COMMAND_HPP_
#define HERO_AGITATOR_SHOOT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "robot/hero/hero_agitator_subsystem.hpp"

namespace src::control::agitator
{
class HeroAgitatorShootCommand : public tap::control::Command
{
public:
    HeroAgitatorShootCommand(src::agitator::HeroAgitatorSubsystem* agitator);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "shoot"; }

private:
    src::agitator::HeroAgitatorSubsystem* agitator;

    uint32_t startTime = 0;
};

}  // namespace src::control::agitator

#endif  // HERO_AGITATOR_SHOOT_COMMAND_HPP_
