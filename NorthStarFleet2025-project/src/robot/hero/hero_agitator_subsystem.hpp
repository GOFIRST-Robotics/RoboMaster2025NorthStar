#ifndef HERO_AGITATOR_SUBSYSTEM_HPP_
#define HERO_AGITATOR_SUBSYSTEM_HPP_

#include <modm/math/geometry/angle.hpp>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/conditional_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/setpoint/interfaces/integrable_setpoint_subsystem.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/servo.hpp"

#include "control/agitator/constants/agitator_constants.hpp"
#include "robot/hero/hero_agitator_config.hpp"

namespace src::agitator
{
class HeroAgitatorSubsystem : public tap::control::Subsystem
{
public:
    HeroAgitatorSubsystem(
        tap::Drivers* drivers,
        const HeroAgitatorSubsystemConfig& config,
        const tap::algorithms::SmoothPidConfig& pidConfig);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override { agitatorMotor.setDesiredOutput(0); }

    const char* getName() const override { return "hero agitator"; }

    /**
     * Sets the velocity setpoint to the specified velocity
     *
     * @param[in] velocity The desired velocity in radians / second.
     */
    void setVelocity(float velocity) { velocitySetpoint = velocity * config.agitatorGearRatio; }

    /// @return The agitator velocity in radians / second.
    inline float getVelocity() const
    {
        return (agitatorMotor.getShaftRPM() / config.agitatorGearRatio) * (M_TWOPI / 60.0f);
    }

    void shoot();

    void reload();

private:
    tap::algorithms::SmoothPid pid;

    HeroAgitatorSubsystemConfig config;

    tap::motor::Servo agitatorServo;

    tap::motor::DjiMotor agitatorMotor;

    uint32_t prevTime = 0;

    float velocitySetpoint = 0;

    /// Get the raw angle of the shaft from the motor, in radians
    float getUncalibratedAgitatorAngle() const;

    /// Runes the velocity PID controller
    void runVelocityPidControl();

    bool init = false;
};

}  // namespace src::agitator

#endif  // HERO_AGITATOR_SUBSYSTEM_HPP_