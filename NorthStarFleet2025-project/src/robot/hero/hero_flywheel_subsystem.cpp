#ifdef TARGET_HERO
#include "hero_flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace src::control::flywheel
{
HeroFlywheelSubsystem::HeroFlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId,
    tap::motor::MotorId downMotorId,
    tap::can::CanBus canBus)
    : tap::control::Subsystem(drivers),
      velocityPidLeftWheel(
          FLYWHEEL_PID_KP,
          FLYWHEEL_PID_KI,
          FLYWHEEL_PID_KD,
          FLYWHEEL_PID_MAX_ERROR_SUM,
          FLYWHEEL_PID_MAX_OUTPUT),
      velocityPidRightWheel(
          FLYWHEEL_PID_KP,
          FLYWHEEL_PID_KI,
          FLYWHEEL_PID_KD,
          FLYWHEEL_PID_MAX_ERROR_SUM,
          FLYWHEEL_PID_MAX_OUTPUT),
      velocityPidDownWheel(
          FLYWHEEL_PID_KP,
          FLYWHEEL_PID_KI,
          FLYWHEEL_PID_KD,
          FLYWHEEL_PID_MAX_ERROR_SUM,
          FLYWHEEL_PID_MAX_OUTPUT),
      leftWheel(drivers, leftMotorId, canBus, false, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, false, "Right Flywheel"),
      downWheel(drivers, downMotorId, canBus, true, "Down Flywheel"),
      desiredLaunchSpeedLeft(0),
      desiredLaunchSpeedRight(0),
      desiredLaunchSpeedDown(0),
      desiredRpmRampLeft(0),
      desiredRpmRampRight(0),
      desiredRpmRampDown(0){};

void HeroFlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    downWheel.initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void HeroFlywheelSubsystem::setDesiredLaunchSpeedLeft(float speed)
{
    desiredLaunchSpeedLeft = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredRpmRampLeft.setTarget(desiredLaunchSpeedLeft);
}

void HeroFlywheelSubsystem::setDesiredLaunchSpeedRight(float speed)
{
    desiredLaunchSpeedRight = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredRpmRampRight.setTarget(desiredLaunchSpeedRight);
}

void HeroFlywheelSubsystem::setDesiredLaunchSpeedDown(float speed)
{
    desiredLaunchSpeedDown = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredRpmRampDown.setTarget(desiredLaunchSpeedDown);
}

float HeroFlywheelSubsystem::getCurrentFlyWheelMotorRPM(tap::motor::DjiMotor motor) const
{
    return motor.getShaftRPM();
}

void HeroFlywheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime == prevTime)
    {
        return;
    }
    desiredRpmRampLeft.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampRight.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampDown.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;

    velocityPidLeftWheel.update(desiredRpmRampLeft.getValue() - leftWheel.getShaftRPM());
    leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
    velocityPidRightWheel.update(desiredRpmRampRight.getValue() - rightWheel.getShaftRPM());
    rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
    velocityPidDownWheel.update(desiredRpmRampDown.getValue() - downWheel.getShaftRPM());
    downWheel.setDesiredOutput(static_cast<int32_t>(velocityPidDownWheel.getValue()));
}
}  // namespace src::control::flywheel

#endif  // TARGET_HERO