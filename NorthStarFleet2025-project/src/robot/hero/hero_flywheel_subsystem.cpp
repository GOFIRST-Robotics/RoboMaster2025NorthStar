#ifdef TARGET_HERO
#include "hero_flywheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/flywheel/flywheel_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace src::flywheel;

namespace src::control::flywheel
{
HeroFlywheelSubsystem::HeroFlywheelSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId,
    tap::motor::MotorId downMotorId,
    tap::can::CanBus canBus)
    : tap::control::Subsystem(drivers),
      spinToRPMMap(SPIN_TO_INTERPOLATABLE_MPS_TO_RPM),
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
      rightWheel(drivers, rightMotorId, canBus, true, "Right Flywheel"),
      downWheel(drivers, downMotorId, canBus, false, "Down Flywheel"),
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

void HeroFlywheelSubsystem::setDesiredSpin(u_int16_t spin)
{
    if (auto spinSet = toSpinPreset(spin))
    {
        desiredSpin = spinSet.value();
        desiredSpinValue = spin;
    }
}

void HeroFlywheelSubsystem::setDesiredLaunchSpeed(float speed)
{
    desiredLaunchSpeedLeft = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredLaunchSpeedRight = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
    desiredLaunchSpeedDown =
        limitVal(speed * (desiredSpinValue / 100.0f), 0.0f, MAX_DESIRED_LAUNCH_SPEED);  // uses spin

    desiredRpmRampLeft.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft));
    desiredRpmRampRight.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedRight));
    desiredRpmRampDown.setTarget(launchSpeedToFlywheelRpm(desiredLaunchSpeedDown));
}

float HeroFlywheelSubsystem::launchSpeedToFlywheelRpm(float launchSpeed) const
{
    modm::interpolation::Linear<modm::Pair<float, float>> MPSToRPMInterpolator = {
        spinToRPMMap.at(desiredSpin).data(),
        spinToRPMMap.at(desiredSpin).size()};
    return MPSToRPMInterpolator.interpolate(launchSpeed);
}
float debugWheelLeft = 0;
float debugWheelRight = 0;
float debugWheelDown = 0;
float debugDesiredLeft = 0;
float debugDesiredRight = 0;
float debugDesiredDown = 0;
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
    debugDesiredLeft = desiredRpmRampLeft.getValue();
    debugDesiredRight = desiredRpmRampRight.getValue();
    debugDesiredDown = desiredRpmRampDown.getValue();
    debugWheelLeft = leftWheel.getShaftRPM();
    debugWheelRight = rightWheel.getShaftRPM();
    debugWheelDown = downWheel.getShaftRPM();
}
}  // namespace src::control::flywheel

#endif  // TARGET_HERO