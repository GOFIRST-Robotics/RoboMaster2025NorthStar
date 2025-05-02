#ifndef SENTRY_TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_
#define SENTRY_TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "control/turret/algorithms/turret_controller_interface.hpp"
#include "robot/sentry/sentry_turret_subsystem.hpp"
#include "robot/sentry/sentry_turret_user_control_command.hpp"

namespace src
{
class Drivers;
}

namespace src::control
{
class ControlOperatorInterface;
}

namespace src::control::turret
{
class TurretSubsystem;
}

namespace src::control::turret::user
{
/**
 * Turret control, with the yaw and pitch gimbals using the world relative frame,
 * such that the desired turret angle is independent of the direction that the chassis
 * is facing or rotating. Assumes the board running this subsystem is a RoboMaster type A //NOTE
 * changed for type C board with an Mpu6500 and that this board is mounted statically on the
 * chassis. Also assumes that there is an IMU mounted on the turret that interfaces with the
 * `TurretMCBCanComm`. If there is no such IMU, the chassis IMU will be used to run the turret
 * controller and the pitch axis will run a controller in the chassis frame.
 *
 * Takes in user input from the `ControlOperatorInterface` to control the pitch and yaw
 * axis of some turret.
 */
class SentryTurretUserWorldRelativeCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * This command requires the turret subsystem from a command/subsystem framework perspective.
     *
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the turret to control.
     * @param[in] chassisImuYawController World frame turret controller that uses the chassis IMU.
     * @param[in] chassisImuPitchController Turret controller that is used when the chassis IMU is
     * in use.
     * @param[in] turretImuYawController World frame turret controller that uses the turret IMU.
     * @param[in] turretImuPitchController Turret controller that is used when the turret IMU is in
     * use. Doesn't strictly have to be world relative.
     */
    SentryTurretUserWorldRelativeCommand(
        tap::Drivers *drivers,
        ControlOperatorInterface &controlOperatorInterface,
        src::control::turret::SentryTurretSubsystem *sentryTurretSubsystem,
        algorithms::TurretYawControllerInterface *chassisImuYawControllerBottom,
        algorithms::TurretPitchControllerInterface *chassisImuPitchControllerBottom,
        algorithms::TurretYawControllerInterface *turretImuYawControllerBottom,
        algorithms::TurretPitchControllerInterface *turretImuPitchControllerBottom,
        algorithms::TurretYawControllerInterface *chassisFrameYawTurretControllerTop,
        algorithms::TurretPitchControllerInterface *chassisImuPitchControllerTop,
        algorithms::TurretPitchControllerInterface *turretImuPitchControllerTop,
        float userYawInputScalar,
        float userPitchInputScalar);

    bool isReady() override;

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupted) override;

    const char *getName() const override { return "turret sentry WR"; }

private:
    SentryTurretUserControlCommand turretWRChassisImuCommand;
    SentryTurretUserControlCommand turretWRTurretImuCommand;
};  // class SentryTurretUserWorldRelativeCommand

}  // namespace src::control::turret::user

#endif  // SENTRY_TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_
