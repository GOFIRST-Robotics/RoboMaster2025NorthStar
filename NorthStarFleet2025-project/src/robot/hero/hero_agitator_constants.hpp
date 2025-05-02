#ifndef HERO_AGITATOR_CONSTANTS_HPP_
#define HERO_AGITATOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem_config.hpp"

// Do not include this file directly: use agitator_constants.hpp instead.
#ifndef AGITATOR_CONSTANTS_HPP_
#error "Do not include this file directly! Use agitator_constants.hpp instead."
#endif

using tap::motor::DjiMotor;

namespace src::control::agitator::constants
{
// position PID terms
// PID terms for standard
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 2'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_C610,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr int AGITATOR_NUM_POCKETS = 8;    // number of balls in one rotation
static constexpr float AGITATOR_MAX_ROF = 20.0f;  // balls per second

static constexpr src::agitator::VelocityAgitatorSubsystemConfig AGITATOR_CONFIG = {
    .gearRatio = 36.0f,
    .agitatorMotorId = tap::motor::MOTOR7,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
    .isAgitatorInverted = false,
    /**
     * The jamming constants. Agitator is considered jammed if difference between the velocity
     * setpoint and actual velocity is > jammingVelocityDifference for > jammingTime.
     */
    .jammingVelocityDifference = static_cast<float>(M_TWOPI),
    .jammingTime = 100,
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 500.0f / static_cast<float>(M_TWOPI),
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config AGITATOR_ROTATE_CONFIG = {
    .targetIntegralChange = 1.1f * (static_cast<float>(M_TWOPI) / AGITATOR_NUM_POCKETS),
    .desiredSetpoint = AGITATOR_MAX_ROF * (static_cast<float>(M_TWOPI) / AGITATOR_NUM_POCKETS),
    .integralSetpointTolerance = (static_cast<float>(M_TWOPI) / AGITATOR_NUM_POCKETS) * 0.25f,
};

static constexpr src::control::agitator::UnjamSpokeAgitatorCommand::Config AGITATOR_UNJAM_CONFIG = {
    .targetUnjamIntegralChange = (static_cast<float>(M_TWOPI) / AGITATOR_NUM_POCKETS),
    .unjamSetpoint =
        0.25f * AGITATOR_MAX_ROF * (static_cast<float>(M_TWOPI) / AGITATOR_NUM_POCKETS),
    /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
    /// seconds.Convert to ms, Add 100 ms extra tolerance.
    .maxWaitTime = static_cast<uint32_t>(
                       1000.0f * (static_cast<float>(M_TWOPI) / AGITATOR_NUM_POCKETS) / 0.25f *
                       AGITATOR_MAX_ROF * (static_cast<float>(M_TWOPI) / AGITATOR_NUM_POCKETS)) +
                   100,
    .targetCycleCount = 3,
};
}  // namespace src::control::agitator::constants

#endif  // HERO_AGITATOR_CONSTANTS_HPP_