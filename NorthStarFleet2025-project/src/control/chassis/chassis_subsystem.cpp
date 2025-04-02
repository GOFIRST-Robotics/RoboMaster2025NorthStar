#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "tap/drivers.hpp"

//#define MECANUM

#include <cmath>

using tap::algorithms::limitVal;

namespace control::chassis
{
// STEP 1 (Tank Drive): create constructor

    float stupidHead;
    control::chassis::ChassisSubsystem::Pid firstPid;

    ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers, const ChassisConfig& config, src::can::TurretMCBCanComm* turretMcbCanComm) :
    Subsystem(drivers), 
    desiredOutput{},
    pidControllers{
        Pid(),
        Pid(),
        Pid(),
        Pid()
    },
    motors{
        Motor(drivers, config.leftFrontId, config.canBus, false, "LF"),
        Motor(drivers, config.leftBackId, config.canBus, false, "LB"),
        Motor(drivers, config.rightFrontId, config.canBus, true, "RF"),
        Motor(drivers, config.rightBackId, config.canBus, true, "RB"),
    },
    turretMcbCanComm(turretMcbCanComm)
    {
        for (auto &controller : pidControllers) {
            controller.setParameter(config.wheelVelocityPidConfig);
        }
        firstPid = pidControllers[0];
    }
// STEP 2 (Tank Drive): initialize function
    void ChassisSubsystem::initialize() {
        for (auto &i : motors) {
            i.initialize();
        }
    }
    float LFSpeed;
    float LBSpeed;
    float RFSpeed;
    float RBSpeed;

    void ChassisSubsystem::setVelocityTurretDrive(float forward, float sideways, float rotational) {
        float turretRot=-turretMcbCanComm->getYaw()+modm::toRadian(drivers->bmi088.getYaw());
        driveBasedOnHeading(forward, sideways, rotational, turretRot);
        
    }

    void ChassisSubsystem::setVelocityFieldDrive(float forward, float sideways, float rotational) {
        float robotHeading = modm::toRadian(drivers->bmi088.getYaw());
        driveBasedOnHeading(forward, sideways, rotational, robotHeading);
    }

    void ChassisSubsystem::driveBasedOnHeading(float forward, float sideways, float rotational, float heading) {
        MotorId::LF;
        float distToCenter = 30.48f;
        float rotationVal;
        if (abs(beyBladeRotationSpeed) > 0.0f) {
            rotationVal = beyBladeRotationSpeed;
        } else if (abs(rotational) > 0) {
            rotationVal = rotational;
        } else {
            rotationVal = fmod(modm::toDegree(turretMcbCanComm->getYaw()+modm::toRadian(drivers->bmi088.getYaw())), 360.0f) / (360.0f * 2);
        }
        double cos_theta = cos(heading);
        double sin_theta = sin(heading);
        double vx_local = forward * cos_theta + sideways * sin_theta;
        double vy_local = -forward * sin_theta + sideways * cos_theta;
        double sqrt2 = sqrt(2.0);
        rotational=modm::toRadian(rotational);
        LFSpeed = mpsToRpm((vx_local - vy_local) / sqrt2 + (rotationVal) * distToCenter * sqrt2);  // Front-left wheel
        RFSpeed = -mpsToRpm((-vx_local - vy_local) / sqrt2 + (rotationVal) * distToCenter * sqrt2); // Front-right wheel
        RBSpeed = -mpsToRpm((-vx_local + vy_local) / sqrt2 + (rotationVal) * distToCenter * sqrt2); // Rear-right wheel
        LBSpeed = mpsToRpm((vx_local + vy_local) / sqrt2 + (rotationVal) * distToCenter * sqrt2);  // Rear-left wheel
        int LF = static_cast<int>(MotorId::LF);
        int LB = static_cast<int>(MotorId::LB);
        int RF = static_cast<int>(MotorId::RF);
        int RB = static_cast<int>(MotorId::RB);
        desiredOutput[LF] = limitVal<float>(LFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        desiredOutput[LB] = limitVal<float>(LBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        desiredOutput[RF] = limitVal<float>(RFSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
        desiredOutput[RB] = limitVal<float>(RBSpeed, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    }

    void ChassisSubsystem::updateBeyBladeRotationSpeed(float distance, float dt) {
        float scaleFactor = 1;
        short direction = -1;
        RandomNumberGenerator::enable();
        if (dt > 50) {
            float calcSpeed = limitVal<float>(scaleFactor / distance, 0.0f, 0.9f) * direction;
            if (RandomNumberGenerator::isReady()) {
                calcSpeed = limitVal<float>(0.1 * sin(RandomNumberGenerator::getValue() + calcSpeed), 0.0f, 1.0f);
            }
            beyBladeRotationSpeed = calcSpeed;
        }
        else if (distance == 0) {
            beyBladeRotationSpeed = 0.1f * direction;
        }
    }

    void ChassisSubsystem::refresh() {
        auto runPid = [](Pid &pid, Motor &motor, float desiredOutput) {
            pid.update(desiredOutput - motor.getShaftRPM());
            motor.setDesiredOutput(pid.getValue());
        };

        for (size_t ii = 0; ii < motors.size(); ii++)
        {
            runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
        }
    }
}  // namespace control::chassis
