#ifndef SENTRY_TURRET_SUBSYSTEM_HPP_
#define SENTRY_TURRET_SUBSYSTEM_HPP_

#include "control/turret/robot_turret_subsystem.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace src::control::turret
{
class TurretSubsystem;
}

namespace src::control::turret
{
/**
 * Turret subsystem for the Sentry.
 */
class SentryTurretSubsystem final : public tap::control::Subsystem
{
public:
    SentryTurretSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorInterface *pitchMotorBottom,
        tap::motor::MotorInterface *yawMotorBottom,
        tap::motor::MotorInterface *pitchMotorTop,
        tap::motor::MotorInterface *yawMotorTop,
        const TurretMotorConfig &pitchMotorBottomConfig,
        const TurretMotorConfig &yawMotorBottomConfig,
        const TurretMotorConfig &pitchMotorTopConfig,
        const TurretMotorConfig &yawMotorTopConfig,
        const src::can::TurretMCBCanComm *turretMCB);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        yawMotorBottom.setMotorOutput(0);
        pitchMotorBottom.setMotorOutput(0);
        yawMotorTop.setMotorOutput(0);
        pitchMotorTop.setMotorOutput(0);
    }

    const char *getName() const override { return "Sentry Turrets"; }

    mockable inline bool isOnline() const
    {
        return pitchMotorBottom.isOnline() && yawMotorBottom.isOnline() &&
               pitchMotorTop.isOnline() && yawMotorTop.isOnline();
    }

    /// Associated with and contains logic for controlling the turret's pitch motor
    TurretMotor pitchMotorBottom;
    /// Associated with and contains logic for controlling the turret's yaw motor
    TurretMotor yawMotorBottom;
    /// Associated with and contains logic for controlling the turret's pitch motor
    TurretMotor pitchMotorTop;
    /// Associated with and contains logic for controlling the turret's yaw motor
    TurretMotor yawMotorTop;

protected:
    const src::can::TurretMCBCanComm *turretMCB;
};  // class SentryTurretSubsystem

}  // namespace src::control::turret

#endif  // SENTRY_TURRET_SUBSYSTEM_HPP_
