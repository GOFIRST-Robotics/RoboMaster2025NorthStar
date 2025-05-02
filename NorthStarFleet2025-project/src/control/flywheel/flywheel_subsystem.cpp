#include "flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace src::flywheel;

namespace src::control::flywheel
{
FlywheelSubsystem::FlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::REVMotorId leftMotorId,
    tap::motor::REVMotorId rightMotorId,
    tap::motor::REVMotorId upMotorId,
    tap::can::CanBus canBus)
    : tap::control::Subsystem(drivers),
      leftWheel(drivers, leftMotorId, canBus, false, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, true, "Right Flywheel"),
      upWheel(drivers, upMotorId, canBus, false, "Up Flywheel"),
      desiredLaunchSpeedLeft(0),
      desiredLaunchSpeedRight(0),
      desiredLaunchSpeedUp(0),
      desiredRpmRampLeft(0),
      desiredRpmRampRight(0),
      desiredRpmRampUp(0){};

void FlywheelSubsystem::initialize()
{
    leftWheel.initialize();
    rightWheel.initialize();
    upWheel.initialize();
}

void FlywheelSubsystem::setDesiredLaunchSpeedLeft(float speed)
{
    desiredLaunchSpeedLeft = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredRpmRampLeft.setTarget(desiredLaunchSpeedLeft);
}
void FlywheelSubsystem::setDesiredLaunchSpeedRight(float speed)
{
    desiredLaunchSpeedRight = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredRpmRampRight.setTarget(desiredLaunchSpeedRight);
}
void FlywheelSubsystem::setDesiredLaunchSpeedUp(float speed)
{
    desiredLaunchSpeedUp = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredRpmRampUp.setTarget(desiredLaunchSpeedUp);
}

float FlywheelSubsystem::getCurrentFlyWheelMotorRPM(tap::motor::RevMotor motor) const
{
    // return motor.getShaftRPM(); // TODO
    return 0.0f;
}

void FlywheelSubsystem::refresh()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (currTime == prevTime)
    {
        return;
    }
    desiredRpmRampLeft.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampRight.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    desiredRpmRampUp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;
    leftWheel.setControlValue(desiredRpmRampLeft.getValue());
    rightWheel.setControlValue(desiredRpmRampRight.getValue());
    upWheel.setControlValue(desiredRpmRampUp.getValue());
    // just use spark pid
    // uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    // if (currTime == prevTime)
    // {
    //     return;
    // }
    // desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    // prevTime = currTime;
    // velocityPidLeftWheel.update(desiredRpmRamp.getValue() - leftWheel.getShaftRPM());
    // leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
    // velocityPidRightWheel.update(desiredRpmRamp.getValue() - rightWheel.getShaftRPM());
    // rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
    // velocityPidUpWheel.update(desiredRpmRamp.getValue() - upWheel.getShaftRPM());
    // upWheel.setDesiredOutput(static_cast<int32_t>(velocityPidUpWheel.getValue()));
}
}  // namespace src::control::flywheel