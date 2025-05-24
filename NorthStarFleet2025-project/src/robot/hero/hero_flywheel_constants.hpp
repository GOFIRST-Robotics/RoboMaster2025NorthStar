#ifndef HERO_FLYWHEEL_CONSTANTS_HPP_
#define HERO_FLYWHEEL_CONSTANTS_HPP_

#include <optional>

#include <modm/container/pair.hpp>

#include "tap/motor/dji_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace src::control::flywheel
{
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 4.0f;

static constexpr tap::motor::MotorId LEFT_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId DOWN_MOTOR_ID = tap::motor::MOTOR1;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;

static constexpr float FLYWHEEL_PID_KP = 30.0f;
static constexpr float FLYWHEEL_PID_KI = 0.0f;
static constexpr float FLYWHEEL_PID_KD = 0.0f;
static constexpr float FLYWHEEL_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float FLYWHEEL_PID_MAX_OUTPUT = 25'000.0f;

enum Spin : u_int8_t
{
    SPIN_90 = 0,
    SPIN_100 = 1,
    SPIN_110 = 2,

    SPIN_COUNT
};

static std::array<std::array<modm::Pair<float, float>, 5>, SPIN_COUNT>
    SPIN_TO_INTERPOLATABLE_MPS_TO_RPM = {
        {{{{0.0f, 0.0f},  // SPIN_90
           {5.0f, 2'600.0f},
           {10.0f, 5'300.0f},
           {15.0f, 8'000.0f},
           {20.0l, 10'600.0f}}},
         {{{0.0f, 0.0f},  // SPIN_100
           {5.0f, 2'600.0f},
           {10.0f, 5'300.0f},
           {15.0f, 8'000.0f},
           {20.0l, 10'600.0f}}},
         {{{0.0f, 0.0f},  // SPIN_110
           {5.0f, 2'600.0f},
           {10.0f, 5'300.0f},
           {15.0f, 8'000.0f},
           {20.0l, 10'600.0f}}}}};

inline std::optional<Spin> toSpinPreset(int value)
{
    switch (value)
    {
        case 90:
            return SPIN_90;
        case 100:
            return SPIN_100;
        case 110:
            return SPIN_110;
        default:
            return SPIN_100;  // invalid input
    }
}

};  // namespace src::control::flywheel

#endif  // HERO_FLYWHEEL_CONSTANTS_HPP_