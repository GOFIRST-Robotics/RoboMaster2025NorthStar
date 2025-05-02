/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "world_frame_chassis_imu_turret_controller.hpp"

#include "tap/drivers.hpp"

#include "control/turret/constants/turret_constants.hpp"

#include "turret_gravity_compensation.hpp"

namespace src::control::turret::algorithms
{
/**
 * Transforms the passed in turret yaw angle in the chassis frame to the world frame (units
 * radians).
 *
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 *      PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU.
 * @param[in] angleToTransform The angle, in radians, to transform. Measured as a turret yaw angle
 *      in the chassis frame.
 * @return A turret yaw angle in radians. `angleToTransform` transformed into the world frame.
 */
static inline WrappedFloat transformChassisFrameToWorldFrame(
    const WrappedFloat initChassisFrameImuAngle,
    const WrappedFloat currChassisFrameImuAngle,
    const WrappedFloat angleToTransform)
{
    return angleToTransform + currChassisFrameImuAngle - initChassisFrameImuAngle;
}

/**
 * Transforms the passed in turret yaw angle in the world frame to the chassis frame (units
 * radians).
 *
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 * PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU.
 * @param[in] angleToTransform The angle, in radians to transform. Measured as a turret yaw angle in
 *      the world frame.
 * @return A turret yaw angle in radians. `angleToTransform` transformed into the chassis frame.
 */
static inline WrappedFloat transformWorldFrameToChassisFrame(
    const WrappedFloat initChassisFrameImuAngle,
    const WrappedFloat currChassisFrameImuAngle,
    const WrappedFloat angleToTransform)
{
    return angleToTransform - currChassisFrameImuAngle + initChassisFrameImuAngle;
}

/**
 * A helper function for the `run*PidYawWorldFrameController` functions below. Updates the passed in
 * `yawMotor`'s desired chassis frame setpoint and the passed in `worldFrameYawSetpoint`'.
 * Performs necessary limiting of the `worldFrameYawSetpoint` based on the `yawMotor`'s
 * min/max yaw setpoints.
 *
 * @param[in] desiredSetpoint The new user-specified world frame turret yaw angle setpoint, in
 *      radians.
 * @param[in] initChassisFrameImuAngle The initial chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU that is captured upon initialization of the chassis IMU world relative
 *      PID controller.
 * @param[in] currChassisFrameImuAngle The current chassis IMU angle, in radians, measured from the
 *      chassis mounted IMU.
 * @param[out] worldFrameYawSetpoint The limited and wrapped world frame turret yaw setpoint, in
 *      radians. Set to `desiredSetpoint` and then wrapped/limited as necessary.
 * @param[out] yawMotor The turret subsystem whose chassis relative turret yaw angle is
 *      updated by this function.
 */
static inline void updateWorldFrameSetpoint(
    const WrappedFloat desiredSetpoint,
    const WrappedFloat chassisFrameInitImuYawAngle,
    const WrappedFloat chassisFrameImuYawAngle,
    WrappedFloat &worldFrameYawSetpoint,
    TurretMotor &yawMotor)
{
    worldFrameYawSetpoint = desiredSetpoint;

    // project target angle in world relative to chassis relative to limit the value
    yawMotor.setChassisFrameSetpoint(transformWorldFrameToChassisFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        worldFrameYawSetpoint));

    if (yawMotor.getConfig().limitMotorAngles)
    {
        // project angle that is limited by the subsystem to world relative again to run the
        // controller. Otherwise use worldFrameYawSetpoint directly.
        worldFrameYawSetpoint = transformChassisFrameToWorldFrame(
            chassisFrameInitImuYawAngle,
            chassisFrameImuYawAngle,
            yawMotor.getChassisFrameSetpoint());
    }
}

WorldFrameYawChassisImuTurretController::WorldFrameYawChassisImuTurretController(
    tap::Drivers &drivers,
    TurretMotor &yawMotor,
    const tap::algorithms::SmoothPidConfig &pidConfig)
    : TurretYawControllerInterface(yawMotor),
      drivers(drivers),
      pid(pidConfig),
      worldFrameSetpoint(Angle(0)),
      chassisFrameInitImuYawAngle(Angle(0))
{
}

void WorldFrameYawChassisImuTurretController::initialize()
{
    if (turretMotor.getTurretController() != this)
    {
        pid.reset();

        chassisFrameInitImuYawAngle = getBmi088Yaw();
        worldFrameSetpoint = turretMotor.getChassisFrameSetpoint();

        turretMotor.attachTurretController(this);
    }
}

void WorldFrameYawChassisImuTurretController::runController(
    const uint32_t dt,
    const WrappedFloat desiredSetpoint)
{
    const WrappedFloat chassisFrameImuYawAngle = getBmi088Yaw();

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        worldFrameSetpoint,
        turretMotor);
    const WrappedFloat worldFrameYawAngle = transformChassisFrameToWorldFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        turretMotor.getChassisFrameMeasuredAngle());

    // position controller based on imu and yaw gimbal angle
    const float positionControllerError =
        turretMotor.getValidMinError(worldFrameSetpoint, worldFrameYawAngle);
    const float pidOutput = pid.runController(
        positionControllerError,
        turretMotor.getChassisFrameVelocity() + modm::toRadian(drivers.bmi088.getGz()),
        dt);

    turretMotor.setMotorOutput(pidOutput);
}

void WorldFrameYawChassisImuTurretController::setSetpoint(WrappedFloat desiredSetpoint)
{
    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameInitImuYawAngle,
        chassisFrameInitImuYawAngle,
        worldFrameSetpoint,
        turretMotor);
}

WrappedFloat WorldFrameYawChassisImuTurretController::getMeasurement() const
{
    const WrappedFloat chassisFrameImuYawAngle =
        getBmi088Yaw();  // NOTE THIS WAS NOT PREVIOUSLY IN RADIANS

    return transformChassisFrameToWorldFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        turretMotor.getChassisFrameMeasuredAngle());
}

bool WorldFrameYawChassisImuTurretController::isOnline() const
{
    return turretMotor.isOnline() &&
           (drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED ||
            drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::
                    IMU_NOT_CALIBRATED);  // TODO not shure if this is valid, was
                                          // drivers.mpu6500.isRunning(); NOTE this is working now
}

WrappedFloat WorldFrameYawChassisImuTurretController::convertControllerAngleToChassisFrame(
    WrappedFloat controllerFrameAngle) const
{
    const WrappedFloat chassisFrameImuYawAngle = getBmi088Yaw();

    return transformWorldFrameToChassisFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        controllerFrameAngle);
}

WrappedFloat WorldFrameYawChassisImuTurretController::convertChassisAngleToControllerFrame(
    WrappedFloat chassisFrameAngle) const
{
    const WrappedFloat chassisFrameImuYawAngle = getBmi088Yaw();

    return transformChassisFrameToWorldFrame(
        chassisFrameInitImuYawAngle,
        chassisFrameImuYawAngle,
        chassisFrameAngle);
}

WorldFramePitchChassisImuTurretController::WorldFramePitchChassisImuTurretController(
    tap::Drivers &drivers,
    TurretMotor &PitchMotor,
    const tap::algorithms::SmoothPidConfig &pidConfig)
    : TurretPitchControllerInterface(PitchMotor),
      drivers(drivers),
      pid(pidConfig),
      worldFrameSetpoint(Angle(0)),
      chassisFrameInitImuPitchAngle(Angle(0))
{
}

void WorldFramePitchChassisImuTurretController::initialize()
{
    if (turretMotor.getTurretController() != this)
    {
        pid.reset();

        chassisFrameInitImuPitchAngle = getBmi088Pitch();
        worldFrameSetpoint = turretMotor.getChassisFrameSetpoint();

        turretMotor.attachTurretController(this);
    }
}

void WorldFramePitchChassisImuTurretController::runController(
    const uint32_t dt,
    const WrappedFloat desiredSetpoint)
{
    const WrappedFloat chassisFrameImuPitchAngle = getBmi088Pitch();

    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameInitImuPitchAngle,
        chassisFrameImuPitchAngle,
        worldFrameSetpoint,
        turretMotor);

    const WrappedFloat worldFramePitchAngle = transformChassisFrameToWorldFrame(
        chassisFrameInitImuPitchAngle,
        chassisFrameImuPitchAngle,
        turretMotor.getChassisFrameMeasuredAngle());

    // position controller based on imu and Pitch gimbal angle
    const float positionControllerError =
        turretMotor.getValidMinError(worldFrameSetpoint, worldFramePitchAngle);
    float pidOutput = pid.runController(
        positionControllerError,
        turretMotor.getChassisFrameVelocity() + modm::toRadian(drivers.bmi088.getGz()),
        dt);
    pidOutput += -computeGravitationalForceOffset(
        TURRET_CG_X,
        TURRET_CG_Z,
        turretMotor.getChassisFrameMeasuredAngle().getWrappedValue() - M_PI / 2,
        GRAVITY_COMPENSATION_SCALAR);
    turretMotor.setMotorOutput(pidOutput);
}

void WorldFramePitchChassisImuTurretController::setSetpoint(WrappedFloat desiredSetpoint)
{
    updateWorldFrameSetpoint(
        desiredSetpoint,
        chassisFrameInitImuPitchAngle,
        chassisFrameInitImuPitchAngle,
        worldFrameSetpoint,
        turretMotor);
}

WrappedFloat WorldFramePitchChassisImuTurretController::getMeasurement() const
{
    const WrappedFloat chassisFrameImuPitchAngle =
        getBmi088Pitch();  // NOTE THIS WAS NOT PREVIOUSLY IN RADIANS

    return transformChassisFrameToWorldFrame(
        chassisFrameInitImuPitchAngle,
        chassisFrameImuPitchAngle,
        turretMotor.getChassisFrameMeasuredAngle());
}

bool WorldFramePitchChassisImuTurretController::isOnline() const
{
    return turretMotor.isOnline() &&
           (drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED ||
            drivers.bmi088.getImuState() ==
                tap::communication::sensors::imu::ImuInterface::ImuState::
                    IMU_NOT_CALIBRATED);  // TODO not shure if this is valid, was
                                          // drivers.mpu6500.isRunning();
}

WrappedFloat WorldFramePitchChassisImuTurretController::convertControllerAngleToChassisFrame(
    WrappedFloat controllerFrameAngle) const
{
    const WrappedFloat chassisFrameImuPitchAngle = getBmi088Pitch();

    return transformWorldFrameToChassisFrame(
        chassisFrameInitImuPitchAngle,
        chassisFrameImuPitchAngle,
        controllerFrameAngle);
}

WrappedFloat WorldFramePitchChassisImuTurretController::convertChassisAngleToControllerFrame(
    WrappedFloat chassisFrameAngle) const
{
    const WrappedFloat chassisFrameImuPitchAngle = getBmi088Pitch();

    return transformChassisFrameToWorldFrame(
        chassisFrameInitImuPitchAngle,
        chassisFrameImuPitchAngle,
        chassisFrameAngle);
}

}  // namespace src::control::turret::algorithms
