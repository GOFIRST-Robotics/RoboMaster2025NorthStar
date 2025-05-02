#include "sentry_turret_user_world_relative_command.hpp"

#include "tap/drivers.hpp"

namespace src::control::turret::user
{
SentryTurretUserWorldRelativeCommand::SentryTurretUserWorldRelativeCommand(
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
    float userPitchInputScalar)
    : tap::control::ComprisedCommand(drivers),
      turretWRChassisImuCommand(
          drivers,
          controlOperatorInterface,
          sentryTurretSubsystem,
          chassisImuYawControllerBottom,
          chassisImuPitchControllerBottom,
          chassisFrameYawTurretControllerTop,  // controler for top turret
          chassisImuPitchControllerTop,
          userYawInputScalar,
          userPitchInputScalar),
      turretWRTurretImuCommand(
          drivers,
          controlOperatorInterface,
          sentryTurretSubsystem,
          turretImuYawControllerBottom,
          chassisImuPitchControllerBottom,     // turretImuPitchControllerBottom, //TODO change to
                                               // pitch
          chassisFrameYawTurretControllerTop,  // controler for top turret
          chassisImuPitchControllerTop,        // turretImuPitchControllerTop,
          userYawInputScalar,
          userPitchInputScalar)

{
    comprisedCommandScheduler.registerSubsystem(sentryTurretSubsystem);
    addSubsystemRequirement(sentryTurretSubsystem);
}

bool SentryTurretUserWorldRelativeCommand::isReady()
{
    return turretWRChassisImuCommand.isReady() || turretWRTurretImuCommand.isReady();
}

void SentryTurretUserWorldRelativeCommand::initialize()
{
    // Try and use turret IMU, otherwise default to chassis IMU.
    if (turretWRTurretImuCommand.isReady())
    {
        comprisedCommandScheduler.addCommand(&turretWRTurretImuCommand);
    }
    else
    {
        comprisedCommandScheduler.addCommand(&turretWRChassisImuCommand);
    }
}
void SentryTurretUserWorldRelativeCommand::execute()
{
    // Re-initialize if no commands scheduled or if the turret WR Turret IMU command
    // is ready and isn't scheduled
    if (!comprisedCommandScheduler.isCommandScheduled(&turretWRChassisImuCommand) &&
        !turretWRTurretImuCommand.isReady() && turretWRChassisImuCommand.isReady())
    {
        initialize();
    }
    else
    {
        if (comprisedCommandScheduler.getAddedCommandBitmap() == 0 ||
            (!comprisedCommandScheduler.isCommandScheduled(&turretWRTurretImuCommand) &&
             turretWRTurretImuCommand.isReady()))
        {
            initialize();
        }
    }

    comprisedCommandScheduler.run();
}

bool SentryTurretUserWorldRelativeCommand::isFinished() const
{
    return turretWRChassisImuCommand.isFinished() && turretWRTurretImuCommand.isFinished();
}

void SentryTurretUserWorldRelativeCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&turretWRTurretImuCommand, interrupted);
    comprisedCommandScheduler.removeCommand(&turretWRChassisImuCommand, interrupted);
}

}  // namespace src::control::turret::user
