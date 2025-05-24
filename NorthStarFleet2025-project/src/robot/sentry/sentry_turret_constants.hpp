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

#ifndef STANDARD_TURRET_CONSTANTS_HPP_
#define STANDARD_TURRET_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/turret/turret_motor_config.hpp"
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_controller_constants.hpp instead."
#endif

using tap::motor::DjiMotor;

// adding a missing definition in this version of taproot
namespace DjiMotorConstants
{
// Output is in mV
static constexpr uint16_t MAX_OUTPUT_GM6020 = 25000;
}  // namespace DjiMotorConstants

namespace src::control::turret
{
static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = -0.02f;

static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::motor::MotorId PITCH_MOTOR_BOTTOM_ID = tap::motor::MOTOR5;  // 1
static constexpr tap::motor::MotorId YAW_MOTOR_BOTTOM_ID = tap::motor::MOTOR6;    // 2
static constexpr tap::motor::MotorId PITCH_MOTOR_TOP_ID = tap::motor::MOTOR7;     // 3
static constexpr tap::motor::MotorId YAW_MOTOR_TOP_ID = tap::motor::MOTOR8;       // 4

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG_BOTTOM = {
    .startAngle = 0,
    .startEncoderValue = 5000,  // enc res 8191
    .minAngle = 0,
    .maxAngle = M_PI / 4,
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG_TOP = {
    .startAngle = modm::toRadian(90),
    .startEncoderValue = 2700,  // enc res 8191
    .minAngle = 0,
    .maxAngle = modm::toRadian(180),
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG_BOTTOM = {
    .startAngle = modm::toRadian(90),
    .startEncoderValue = 7461,
    .minAngle = modm::toRadian(20),
    .maxAngle = modm::toRadian(120),
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG_TOP = {
    .startAngle = modm::toRadian(90),
    .startEncoderValue = 7461,
    .minAngle = modm::toRadian(20),
    .maxAngle = modm::toRadian(120),
    .limitMotorAngles = true,
};

static constexpr float TURRET_CG_X = 0.0f;                  // 30.17;
static constexpr float TURRET_CG_Z = 0.0f;                  // 34.02;
static constexpr float GRAVITY_COMPENSATION_SCALAR = 0.0f;  // 7'000;

namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    .kp = 40.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    // .kp = 22.0f,
    // .ki = 0.0f,
    // .kd = 0.3f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 50.0f,
    .ki = 0.0f,
    .kd = 0.3f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    .kp = 700.0f,
    .ki = 20.0f,
    .kd = 0.0f,
    .maxICumulative = 1'000.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    // .kp = 20'000.0f,
    // .ki = 100.0f,
    // .kd = 0.0f,
    // .maxICumulative = 2'000.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.5f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 20.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.5f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    // .kp = 22.0f,
    // .ki = 0.0f,
    // .kd = 0.0f,
    // .maxICumulative = 0.5f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 45.0f,
    .ki = 0.6f,
    .kd = 1.0f,
    .maxICumulative = 0.5f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 1'000.0f,
    .ki = 0.0f,
    .kd = 10.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    // .kp = 10'000.0f,
    // .ki = 0.0f,
    // .kd = 100.0f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.5f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 75000.0f,
    .ki = 0.0f,
    .kd = 8000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,

    // .kp = 100'000.0f,
    // .ki = 0.0f,
    // .kd = 2000.0f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 0.1f,
    // .tRDerivativeKalman = 70.0f,
    // .tQProportionalKalman = 0.1f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,

    // .kp = 3'000.1f,
    // .ki = 0.0f,
    // .kd = 500.2f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 0.0001f,
    // .tRDerivativeKalman = 100000.0f,
    // .tQProportionalKalman = 0.00001f,
    // .tRProportionalKalman =10000.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    // .kp = 100'183.1f,
    // .ki = 0.0f,
    // .kd = 3'448.5f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 0.1f,
    // .tRDerivativeKalman = 10.0f,
    // .tQProportionalKalman = 0.1f,
    // .tRProportionalKalman = 2.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,

    .kp = 70'000.0f,
    .ki = 0.0f,
    .kd = 500.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace world_rel_chassis_imu

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 75000.0f,
    .ki = 0.0f,
    .kd = 8000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,

    // .kp = 100'000.0f,
    // .ki = 0.0f,
    // .kd = 2000.0f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 0.1f,
    // .tRDerivativeKalman = 70.0f,
    // .tQProportionalKalman = 0.1f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,

    // .kp = 3'000.1f,
    // .ki = 0.0f,
    // .kd = 500.2f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 0.0001f,
    // .tRDerivativeKalman = 100000.0f,
    // .tQProportionalKalman = 0.00001f,
    // .tRProportionalKalman =10000.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    // .kp = 100'183.1f,
    // .ki = 0.0f,
    // .kd = 3'448.5f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 0.1f,
    // .tRDerivativeKalman = 10.0f,
    // .tQProportionalKalman = 0.1f,
    // .tRProportionalKalman = 2.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,

    .kp = 70'000.0f,
    .ki = 0.0f,
    .kd = 500.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

}  // namespace chassis_rel

}  // namespace src::control::turret
#endif