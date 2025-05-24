#ifndef HERO_AGITATOR_SUBSYSTEM_CONFIG_HPP_
#define HERO_AGITATOR_SUBSYSTEM_CONFIG_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/servo.hpp"

namespace src::agitator
{
/**
 * Configuration struct for the hero agitator subsystem
 */
struct HeroAgitatorSubsystemConfig
{
    /// The servo ID for this servo.
    tap::gpio::Pwm::Pin agitatorServoId;

    /// The maximum allowable PWM output. This is limited between 0 and 1.
    float maximumPwm;
    /// The minimum allowable PWM output. This is limited between 0 and 1.
    float minimumPwm;
    /// The speed in PWM percent per millisecond.
    float pwmRampSpeed;
    /// If `true` positive rotation is clockwise when looking at the servo shaft opposite the
    /// servo. Counterclockwise if false.
    bool isAgitatorServoInverted;

    tap::motor::MotorId agitatorMotorId;

    tap::can::CanBus canBus;

    bool agitatorMotorInverted;

    float agitatorGearRatio;
};

struct HeroAgitatorMoveCongig
{
    int targetDisplacement;

    uint32_t moveTime;

    uint32_t pauseAfterMoveTime;

    bool setToTargetOnEnd;

    float setpointTolerance;
};
}  // namespace src::agitator

#endif  // HERO_AGITATOR_SUBSYSTEM_CONFIG_HPP_