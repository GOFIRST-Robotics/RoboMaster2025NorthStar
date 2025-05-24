#ifndef STANDARD_FLYWHEEL_CONSTANTS_HPP_
#define STANDARD_FLYWHEEL_CONSTANTS_HPP_
#include <modm/container/pair.hpp>

#include "tap/motor/rev_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace src::control::flywheel
{
static constexpr float FRICTION_WHEEL_RAMP_SPEED = .0001f;

static constexpr tap::motor::REVMotorId LEFT_MOTOR_ID = tap::motor::REV_MOTOR4;
static constexpr tap::motor::REVMotorId RIGHT_MOTOR_ID = tap::motor::REV_MOTOR5;
static constexpr tap::motor::REVMotorId UP_MOTOR_ID = tap::motor::REV_MOTOR6;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;

static constexpr float FLYWHEEL_PID_KP = 10.0f;
static constexpr float FLYWHEEL_PID_KI = 0.0f;
static constexpr float FLYWHEEL_PID_KD = 0.0f;
static constexpr float FLYWHEEL_PID_MAX_ERROR_SUM = 5'000.0f;
static constexpr float FLYWHEEL_PID_MAX_OUTPUT = 16'000.0f;

// TODO make these correct
enum Spin : u_int8_t
{
    SPIN_90 = 0,
    SPIN_100 = 1,
    SPIN_110 = 2,

    SPIN_COUNT
};

static std::array<std::array<modm::Pair<float, float>, 5>, SPIN_COUNT>
    SPIN_TO_INTERPOLATABLE_MPS_TO_RPM = {
        {{{{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}},    // SPIN_90
         {{{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}},    // SPIN_100
         {{{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}}}};  // SPIN_110

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

// static std::unordered_map<u_int16_t, std::vector<modm::Pair<float, float>>>
//     SPIN_TO_INTERPOLATABLE_MPS_TO_RPM = {
//         {90, {{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}},
//         {100, {{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}},
//         {110, {{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}}};
}  // namespace src::control::flywheel

#endif