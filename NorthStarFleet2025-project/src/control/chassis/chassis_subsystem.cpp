#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

//#define MECANUM

#include <cmath>

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers,
    const ChassisConfig& config,
    src::can::TurretMCBCanComm* turretMcbCanComm)
    : Subsystem(drivers),
      desiredOutput{},
      pidControllers{
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          modm::Pid<float>(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT)},
      motors{
          Motor(drivers, config.leftFrontId, config.canBus, false, "LF"),
          Motor(drivers, config.leftBackId, config.canBus, false, "LB"),
          Motor(drivers, config.rightFrontId, config.canBus, false, "RF"),
          Motor(drivers, config.rightBackId, config.canBus, false, "RB"),
      },
      rateLimiters{
          src::chassis::algorithms::SlewRateLimiter(1000, 10),
          src::chassis::algorithms::SlewRateLimiter(1000, 10),
          src::chassis::algorithms::SlewRateLimiter(1000, 10),
          src::chassis::algorithms::SlewRateLimiter(1000, 10),
      },
      turretMcbCanComm(turretMcbCanComm)
{
}

void ChassisSubsystem::initialize()
{
    for (auto& i : motors)
    {
        i.initialize();
    }
}

void ChassisSubsystem::setVelocityDrive(
    float forward,
    float sideways,
    float rotational,
    float turretRot = 0.0f)
{
#ifdef FIELD
    float robotHeading = modm::toRadian(turretMcbCanComm->getYaw());
#else
    float robotHeading = -(turretRot);  // Signs subject to change, just want the difference
    robotHeading = fmod(robotHeading, 2 * M_PI);
#endif
// For robotCentric only just + M_PI_4
#ifdef MECANUM
    // Mecanum
    float forwardAdjusted = forward * cos(robotHeading);
    float sidewaysAdjusted = sideways * sin(robotHeading);
    LFSpeed = mpsToRpm(
        forwardAdjusted - sidewaysAdjusted - (2 * DIST_TO_CENTER * rotational * M_PI / 180));
    LBSpeed = mpsToRpm(
        forwardAdjusted + sidewaysAdjusted - (2 * DIST_TO_CENTER * rotational * M_PI / 180));
    RFSpeed = mpsToRpm(
        forwardAdjusted + sidewaysAdjusted + (2 * DIST_TO_CENTER * rotational * M_PI / 180));
    RBSpeed = mpsToRpm(
        forwardAdjusted - sidewaysAdjusted + (2 * DIST_TO_CENTER * rotational * M_PI / 180));
#else
    // Omni
    turretRot = -turretMcbCanComm->getYaw() + modm::toRadian(drivers->bmi088.getYaw());
    double cos_theta = cos(turretRot);
    double sin_theta = sin(turretRot);
    double vx_local = forward * cos_theta + sideways * sin_theta;
    double vy_local = -forward * sin_theta + sideways * cos_theta;
    double sqrt2 = sqrt(2.0);
    rotational = modm::toRadian(rotational);
    float LFSpeed = mpsToRpm(
        (vx_local - vy_local) / sqrt2 + rotational * DIST_TO_CENTER * sqrt2);  // Front-left wheel
    float RFSpeed = mpsToRpm(
        (-vx_local - vy_local) / sqrt2 + rotational * DIST_TO_CENTER * sqrt2);  // Front-right wheel
    float RBSpeed = mpsToRpm(
        (-vx_local + vy_local) / sqrt2 + rotational * DIST_TO_CENTER * sqrt2);  // Rear-right wheel
    float LBSpeed = mpsToRpm(
        (vx_local + vy_local) / sqrt2 + rotational * DIST_TO_CENTER * sqrt2);  // Rear-left wheel
#endif
    int LF = static_cast<int>(MotorId::LF);
    int LB = static_cast<int>(MotorId::LB);
    int RF = static_cast<int>(MotorId::RF);
    int RB = static_cast<int>(MotorId::RB);
    desiredOutput[LF] = limitVal<float>(LFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    desiredOutput[LB] = limitVal<float>(LBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    desiredOutput[RF] = limitVal<float>(RFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    desiredOutput[RB] = limitVal<float>(RBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
}

void ChassisSubsystem::refresh()
{
    auto runPid = [](Pid& pid, Motor& motor, float desiredOutput) {
        pid.update(desiredOutput - motor.getShaftRPM());
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
    }
}
}  // namespace src::chassis
