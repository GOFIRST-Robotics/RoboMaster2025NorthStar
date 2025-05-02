#ifndef HERO_CHASSIS_CONSTANTS_HPP_
#define HERO_CHASSIS_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#include "control/chassis/chassis_subsystem.hpp"

#ifndef CHASSIS_CONSTANTS_HPP_
#error "Do not include this file directly! Use chassis_constants.hpp instead."
#endif

using tap::can::CanBus;
using tap::motor::DjiMotor;

namespace src::chassis
{
static constexpr float VELOCITY_PID_KP = 1.0f;                  // 10.0f;
static constexpr float VELOCITY_PID_KI = 0.0f;                  // 0.0f;
static constexpr float VELOCITY_PID_KD = 0.0f;                  // 1.25f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 16'000.0f;  // 0.0f;
static constexpr float VELOCITY_PID_KV = 0.0f;                  // 0.057f;
static constexpr float VELOCITY_PID_KS = 0.0f;                  // 350.0f;
static constexpr float VELOCITY_PID_MAX_OUTPUT = DjiMotor::MAX_OUTPUT_C620;

static const float DIST_TO_CENTER = 30.48f;  // from wheel to center
}  // namespace src::chassis

#endif  // HERO_CHASSIS_CONSTANTS_HPP_
