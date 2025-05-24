#include "hero_agitator_shoot_command.hpp"

namespace src::control::agitator
{
HeroAgitatorShootCommand::HeroAgitatorShootCommand(src::agitator::HeroAgitatorSubsystem* agitator)
    : tap::control::Command(),
      agitator(agitator),
      startTime(0)
{
    addSubsystemRequirement(agitator);
}

void HeroAgitatorShootCommand::initialize()
{
    startTime = tap::arch::clock::getTimeMilliseconds();
    agitator->shoot();
}

// set the hopper servo to the open position
void HeroAgitatorShootCommand::execute() {}

// set the hopper servo to the close position
void HeroAgitatorShootCommand::end(bool) { agitator->reload(); }

bool HeroAgitatorShootCommand::isFinished() const
{
    return tap::arch::clock::getTimeMilliseconds() - startTime >= 500;
}

}  // namespace src::control::agitator
