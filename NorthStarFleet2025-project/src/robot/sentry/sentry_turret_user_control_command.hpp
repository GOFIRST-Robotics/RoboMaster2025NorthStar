#ifndef SENTRY_TURRET_USER_CONTROL_COMMAND_HPP_
#define SENTRY_TURRET_USER_CONTROL_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "control/turret/algorithms/turret_controller_interface.hpp"
#include "robot/sentry/sentry_turret_subsystem.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::control::turret::user
{
/**
 * Command that takes user input from the `ControlOperatorInterface` to control the pitch and yaw
 * axis of some turret using some passed in yaw and pitch controller upon construction.
 */
class SentryTurretUserControlCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the sentry turret to control.
     * @param[in] yawController Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchController Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     * @param[in] userYawInputScalar Value to scale the user input from `ControlOperatorInterface`
     * by. Basically mouse sensitivity.
     * @param[in] userPitchInputScalar See userYawInputScalar.
     */
    SentryTurretUserControlCommand(
        tap::Drivers *drivers,
        ControlOperatorInterface &controlOperatorInterface,
        SentryTurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawControllerBottom,
        algorithms::TurretPitchControllerInterface *pitchControllerBottom,
        algorithms::TurretYawControllerInterface *yawControllerTop,
        algorithms::TurretPitchControllerInterface *pitchControllerTop,
        float userYawInputScalar,
        float userPitchInputScalar);

    bool isReady() override;

    const char *getName() const override { return "Sentry user turret control"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

private:
    tap::Drivers *drivers;
    ControlOperatorInterface &controlOperatorInterface;
    SentryTurretSubsystem *turretSubsystem;

    uint32_t prevTime = 0;

    algorithms::TurretYawControllerInterface *yawControllerBottom;
    algorithms::TurretPitchControllerInterface *pitchControllerBottom;
    algorithms::TurretYawControllerInterface *yawControllerTop;
    algorithms::TurretPitchControllerInterface *pitchControllerTop;

    const float userYawInputScalar;
    const float userPitchInputScalar;

    WrappedFloat userYawTopInput;
};
}  // namespace src::control::turret::user

#endif  // SENTRY_TURRET_USER_CONTROL_COMMAND_HPP_
