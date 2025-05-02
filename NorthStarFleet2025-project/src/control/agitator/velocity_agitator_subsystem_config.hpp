#ifndef VELOCITY_AGITATOR_SUBSYSTEM_CONFIG_HPP_
#define VELOCITY_AGITATOR_SUBSYSTEM_CONFIG_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

namespace src::agitator
{
/**
 * @param[in] jammingDistance jamming timer counts down when distance between
 *      setpoint and current angle is > `jammingDistance` and resets timer when
 *      distance is <= `jammingDistance`.
 * @param[in] jammingTime how long the jamming timer is. Once this timer finishes
 *      the subsystem is considered jammed
 * @param[in] jamLogicEnabled whether or not to enable jam detection
 *
 * Configuration struct for the velocity agitator subsystem
 */
struct VelocityAgitatorSubsystemConfig
{
    /// Motor gear ratio, so we use shaft angle rather than encoder angle.
    float gearRatio;
    /// The motor ID for this motor.
    tap::motor::MotorId agitatorMotorId;
    /// The motor CAN bus for this motor.
    tap::can::CanBus agitatorCanBusId;
    /// If `true` positive rotation is clockwise when looking at the motor shaft opposite the motor.
    /// Counterclockwise if false.
    bool isAgitatorInverted;
    /// Difference between target and setpoint velocity.
    float jammingVelocityDifference;
    /// How long the jamming timer is. If the target/setpoint difference is > the
    /// jammingVelocityDifference for jammingTime the agitator is considered to be jammed.
    uint32_t jammingTime;
    /// A flag which determines whether or not jamming detection is enabled. `true` means enabled,
    /// `false` means disabled.
    bool jamLogicEnabled;
    /// Velocity PID feed forward term. Scaling factor that converts desired velocity to desired
    /// motor output. When using the M308 or the M2006, motor velocity -> motor current is mostly
    /// linear since these motors take a desired current as a command. When using a motor that is
    /// controlled by sending voltage commands, this term should be 0.
    float velocityPIDFeedForwardGain;
};
}  // namespace src::agitator

#endif  // VELOCITY_AGITATOR_SUBSYSTEM_CONFIG_HPP_