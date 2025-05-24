#ifndef TARGET_HERO

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
      spinToRPMMap(SPIN_TO_INTERPOLATABLE_MPS_TO_RPM),
      leftWheel(drivers, leftMotorId, canBus, false, "Left Flywheel"),
      rightWheel(drivers, rightMotorId, canBus, false, "Right Flywheel"),
      upWheel(drivers, upMotorId, canBus, true, "Up Flywheel"),
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

void FlywheelSubsystem::setDesiredSpin(u_int16_t spin)
{
    if (auto spinSet = toSpinPreset(spin))
    {
        desiredSpin = spinSet.value();
        desiredSpinValue = spin;
    }
}

/**
 * using the set spin sets a desired rpm for the flywheels with the up wheel scaled by the spin
 * @param[in] speed in meters per second
 */
void FlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeedLeft = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredLaunchSpeedRight = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredLaunchSpeedUp =
        limitVal(speed * (desiredSpinValue / 100.0f), 0.0f, MAX_DESIRED_LAUNCH_SPEED);  // uses spin

    desiredRpmRampLeft.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft));
    desiredRpmRampRight.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedRight));
    desiredRpmRampUp.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedUp));
}
// void FlywheelSubsystem::setDesiredLaunchSpeedRight(float speed)
// {
//     desiredLaunchSpeedRight = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
//     desiredRpmRampRight.setTarget(launchSpeedToFlywheelRpm(speed));
// }
// void FlywheelSubsystem::setDesiredLaunchSpeedUp(float speed)
// {
//     desiredLaunchSpeedUp = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
//     desiredRpmRampUp.setTarget(launchSpeedToFlywheelRpm(speed));
// }

float FlywheelSubsystem::launchSpeedToFlywheelRpm(float launchSpeed) const
{
    modm::interpolation::Linear<modm::Pair<float, float>> MPSToRPMInterpolator = {
        spinToRPMMap.at(desiredSpin).data(),
        spinToRPMMap.at(desiredSpin).size()};
    return MPSToRPMInterpolator.interpolate(launchSpeed);
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
}
}  // namespace src::control::flywheel

#endif  // TARGET_HERO