/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_MOTOR_HPP_
#define TURRET_MOTOR_HPP_

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

#include "algorithms/turret_controller_interface.hpp"
#include "modm/math/geometry/angle.hpp"

#include "turret_motor_config.hpp"

namespace src::control::turret
{
/**
 * Logic encapsulating the control of a single axis of a turret gimbal motor. Contains logic for
 * storing chassis relative position measurements and setpoints and logic for limiting the angle
 * setpoint.
 *
 * Currently, there are GM6020-specific motor parameters in this object such that it is expected
 * that the gimbal motor used is a 6020, but in general with some taproot-side MRs, this class can
 * be generalized to work with any motor interface.
 */
class TurretMotor
{
public:
    /// Maximum output, voltage control between [-24, 24] volts scaled up to [-30,000, 30,000] units
    static constexpr float MAX_OUT_6020 = 30'000;

    /**
     * Construct a turret motor with some particular hardware motor interface and a motor
     * configuration struct.
     */
    TurretMotor(tap::motor::MotorInterface *motor, const TurretMotorConfig &motorConfig);

    mockable inline void initialize() { motor->initialize(); }

    /// Updates the measured motor angle
    mockable void updateMotorAngle();

    /**
     * Set the motor's desired output when the motor is online. The output is expected to be in the
     * motor's unitless form. For the GM6020, the motor output is limited between [-MAX_OUT_6020,
     * MAX_OUT_6020].
     *
     * @param[in] out The desired motor output.
     */
    mockable void setMotorOutput(float out);

    /**
     * Attaches the specified turretController to this turret motor. This does not give ownership
     * of the controller to this object. Instead it allows commands to know which turret controller
     * is currently being run (since turret controllers are shared by commands but persist across
     * different commands).
     */
    mockable inline void attachTurretController(
        const algorithms::TurretControllerInterface *turretController)
    {
        this->turretController = turretController;
    }

    /**
     * Sets (and limits!) the chassis frame turret measurement.
     *
     * The setpoint is limited between the min and max config angles as specified in the
     * constructor.
     */
    mockable void setChassisFrameSetpoint(WrappedFloat setpoint);

    /// @return `true` if the hardware motor is connected and powered on
    mockable inline bool isOnline() const { return motor->isMotorOnline(); }

    /**
     * @return turret motor angle setpoint relative to the chassis, in radians
     */
    mockable inline WrappedFloat getChassisFrameSetpoint() const { return chassisFrameSetpoint; }

    /// @return turret motor angle measurement relative to the chassis, in radians, wrapped between
    /// [0, 2 PI)
    mockable inline const WrappedFloat &getChassisFrameMeasuredAngle() const
    {
        return chassisFrameMeasuredAngle;
    }

    /**
     * @return angular velocity of the turret, in rad/sec, positive rotation is defined by the
     * motor.
     */
    mockable inline float getChassisFrameVelocity() const
    {
        return motor->getEncoder()->getVelocity() / static_cast<float>(M_TWOPI) * 60.f;
    }

    /// @return turret controller controlling this motor (as specified by `attachTurretController`)
    mockable const algorithms::TurretControllerInterface *getTurretController() const
    {
        return turretController;
    }

    /// @return The turret motor config struct associated with this motor
    mockable const TurretMotorConfig &getConfig() const { return config; }

    /**
     * @return Valid minimum error between the chassis relative setpoint and measurement, in
     * radians.
     *
     * @note A valid measurement error is either:
     * - The shortest wrapped distance between the chassis frame measurement and setpoint
     *   if the turret motor is not limited to some min/max values.
     * - The absolute difference between the chassis frame measurement and setpoint if the
     *   turret motor is limited to some min/max values.
     */
    mockable float getValidChassisMeasurementError() const;

    /**
     * @param[in] measurement A turret measurement in the chassis frame, an angle in radians. This
     * can be encoder based (via getChassisFrameMeasuredAngle) or can be measured by some other
     * means (for example, an IMU on the turret that is than transformed to the chassis frame).
     *
     * @return The minimum error between the chassis frame setpoint and the specified measurement.
     * If the turret motor is not limited, the error is wrapped between [0, 2*PI), otherwise the
     * error is absolute.
     *
     * @note Call getValidChassisMeasurementError if you want the error between the chassis-frame
     * setpoint and measurement
     *
     * @note The measurement does not need to be normalized to [0, 2*PI]. In fact, if the turret
     * motor is limited, an unwrapped measurement should be used in order to avoid unexpected
     * wrapping errors.
     *
     * @note Before calling this function, you **must** first set the chassis frame setpoint before
     * calling this function (i.e. call `setChassisFrameSetpoint`).
     */
    mockable float getValidMinError(const WrappedFloat setpoint, const WrappedFloat measurement)
        const;

    int16_t getMotorOutput() const { return motor->getOutputDesired(); }

private:
    const TurretMotorConfig config;

    /// Low-level motor object that this object interacts with
    tap::motor::MotorInterface *motor;

    /// Associated turret controller interface that is being used by a command to control this motor
    const algorithms::TurretControllerInterface *turretController = nullptr;

    /// ratio of motor rotations per rotation of controled pivot
    float ratio;

    /// Unwrapped chassis frame setpoint specified by the user and limited to `[config.minAngle,
    /// config.maxAngle]`. Units radians.
    WrappedFloat chassisFrameSetpoint;

    /// Wrapped chassis frame measured angle between [0, 2*PI). Units radians.
    WrappedFloat chassisFrameMeasuredAngle;

    int64_t lastUpdatedEncoderValue;
};
}  // namespace src::control::turret

#endif  // TURRET_MOTOR_HPP_
