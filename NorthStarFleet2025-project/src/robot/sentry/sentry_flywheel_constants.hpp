#ifndef SENTRY_FLYWHEEL_CONSTANTS_HPP_
#define SENTRY_FLYWHEEL_CONSTANTS_HPP_

#include "tap/motor/rev_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace src::flywheel
{
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 1.0f;

static constexpr tap::motor::REVMotorId LEFT_MOTOR_ID = tap::motor::REV_MOTOR1;
static constexpr tap::motor::REVMotorId RIGHT_MOTOR_ID = tap::motor::REV_MOTOR2;
static constexpr tap::motor::REVMotorId UP_MOTOR_ID = tap::motor::REV_MOTOR3;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS2;

static constexpr float FLYWHEEL_PID_KP = 10.0f;
static constexpr float FLYWHEEL_PID_KI = 0.0f;
static constexpr float FLYWHEEL_PID_KD = 0.0f;
static constexpr float FLYWHEEL_PID_MAX_ERROR_SUM = 5'000.0f;
static constexpr float FLYWHEEL_PID_MAX_OUTPUT = 16'000.0f;

}  // namespace src::flywheel

#endif