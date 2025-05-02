#include "chassis_subsystem.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

using tap::algorithms::limitVal;

namespace src::chassis
{
ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers,
    const ChassisConfig& config,
    src::can::TurretMCBCanComm* turretMcbCanComm,
    tap::motor::DjiMotor* yawMotor)
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
      turretMcbCanComm(turretMcbCanComm),
      yawMotor(yawMotor)
{
}

void ChassisSubsystem::initialize()
{
    for (auto& i : motors)
    {
        i.initialize();
    }
}
float LFSpeed;
float LBSpeed;
float RFSpeed;
float RBSpeed;

inline float ChassisSubsystem::getTurretYaw()
{
    return (fmod(yawMotor->getPositionWrapped() + M_PI, M_PI * 2)) - M_PI;
}

float ChassisSubsystem::getChassisTurretOffset()
{
    // In future may change with turretMcbCanComm->getYaw()
    return fmod(modm::toRadian(drivers->bmi088.getYaw()) - getTurretYaw() + M_PI_4, M_PI_2) -
           M_PI_4;
}

void ChassisSubsystem::setVelocityTurretDrive(float forward, float sideways, float rotational)
{
    float turretRot = -getTurretYaw() + modm::toRadian(drivers->bmi088.getYaw());
    driveBasedOnHeading(forward, sideways, rotational, turretRot);
}

void ChassisSubsystem::setVelocityFieldDrive(float forward, float sideways, float rotational)
{
    float robotHeading = modm::toRadian(drivers->bmi088.getYaw());
    driveBasedOnHeading(forward, sideways, rotational, robotHeading);
}

void ChassisSubsystem::driveBasedOnHeading(
    float forward,
    float sideways,
    float rotational,
    float heading)
{
    float distToCenter = 0.3048f;
    double cos_theta = cos(heading);
    double sin_theta = sin(heading);
    double vx_local = forward * cos_theta + sideways * sin_theta;
    double vy_local = -forward * sin_theta + sideways * cos_theta;
    double sqrt2 = sqrt(2.0);
    LFSpeed = mpsToRpm(
        (vx_local - vy_local) / sqrt2 + (rotational)*distToCenter * sqrt2);  // Front-left wheel
    RFSpeed = mpsToRpm(
        (-vx_local - vy_local) / sqrt2 + (rotational)*distToCenter * sqrt2);  // Front-right wheel
    RBSpeed = mpsToRpm(
        (-vx_local + vy_local) / sqrt2 + (rotational)*distToCenter * sqrt2);  // Rear-right wheel
    LBSpeed = mpsToRpm(
        (vx_local + vy_local) / sqrt2 + (rotational)*distToCenter * sqrt2);  // Rear-left wheel
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
