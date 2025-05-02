#include "chassis_beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisBeybladeCommand::ChassisBeybladeCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface,
    float distScaleFactor,
    short direction,
    float spinVel,
    bool isVariable)
    : chassis(chassis),
      operatorInterface(operatorInterface),
      distScaleFactor(distScaleFactor),
      direction(direction),
      spinVel(spinVel),
      isVariable(isVariable)
{
    addSubsystemRequirement(chassis);
}

void ChassisBeybladeCommand::initialize() { prevTime = tap::arch::clock::getTimeMilliseconds(); }

void ChassisBeybladeCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    auto scale = [](float raw) -> float {
        return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
    };
    chassis->setVelocityFieldDrive(
        scale(operatorInterface->getDrivetrainVerticalTranslation()),
        -scale(operatorInterface->getDrivetrainHorizontalTranslation()),
        calculateBeyBladeRotationSpeed(1, dt));
}

void ChassisBeybladeCommand::end(bool interrupted) { chassis->setVelocityFieldDrive(0, 0, 0); }

float ChassisBeybladeCommand::calculateBeyBladeRotationSpeed(float distance, uint32_t dt)
{
    accumTime += dt;
    if (distance < 0.1f)
    {
        return 0.0f;
    }
    if (!isVariable)
    {
        return limitVal<float>(distScaleFactor / distance, 0.0f, 1.0f) * direction * spinVel;
    }
    RandomNumberGenerator::enable();
    if (accumTime > 500)
    {
        float calcSpeed = limitVal<float>(distScaleFactor / distance, 0.0f, 0.9f) * direction;
        if (RandomNumberGenerator::isReady())
        {
            calcSpeed = limitVal<float>(
                0.1 * sin(RandomNumberGenerator::getValue()) + calcSpeed,
                -1.0f,
                1.0f);
        }
        accumTime = 0;
        return calcSpeed * spinVel;
    }
}
};  // namespace src::chassis