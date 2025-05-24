#include "hero_agitator_subsystem.hpp"

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/drivers.hpp"

using namespace src::control::agitator::constants;

namespace src::agitator
{
HeroAgitatorSubsystem::HeroAgitatorSubsystem(
    tap::Drivers* drivers,
    const HeroAgitatorSubsystemConfig& config,
    const tap::algorithms::SmoothPidConfig& pidConfig)
    : tap::control::Subsystem(drivers),
      config(config),
      agitatorServo(
          drivers,
          config.agitatorServoId,
          config.maximumPwm,
          config.minimumPwm,
          config.pwmRampSpeed),
      agitatorMotor(
          drivers,
          config.agitatorMotorId,
          config.canBus,
          config.agitatorMotorInverted,
          "Agitator"),
      pid(pidConfig)

{
    agitatorServo.setTargetPwm(agitatorServo.getMinPWM());
}

void HeroAgitatorSubsystem::initialize() { agitatorMotor.initialize(); }

void HeroAgitatorSubsystem::shoot() { agitatorServo.setTargetPwm(agitatorServo.getMaxPWM()); }

void HeroAgitatorSubsystem::reload()
{
    init = true;
    if (getUncalibratedAgitatorAngle() > M_TWOPI)
    {
        agitatorServo.setTargetPwm(agitatorServo.getMinPWM());
        agitatorMotor.resetEncoderValue();
    }
}

void HeroAgitatorSubsystem::refresh()
{
    agitatorServo.updateSendPwmRamp();
    if (getUncalibratedAgitatorAngle() < M_TWOPI)
    {
        velocitySetpoint = init * M_TWOPI * 1.5;
    }
    else
    {
        velocitySetpoint = 0.0f;
    }
    runVelocityPidControl();
}
float HeroAgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    return (2.0f * M_PI / tap::motor::DjiMotor::ENC_RESOLUTION) *
           agitatorMotor.getEncoderUnwrapped() / config.agitatorGearRatio;
}

void HeroAgitatorSubsystem::runVelocityPidControl()
{
    const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    const uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    const float velocityError = velocitySetpoint - getVelocity();

    pid.runControllerDerivateError(velocityError, dt);

    agitatorMotor.setDesiredOutput(pid.getOutput() + velocitySetpoint);
}

}  // namespace src::agitator