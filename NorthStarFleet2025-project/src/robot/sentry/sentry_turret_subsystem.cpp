#include "sentry_turret_subsystem.hpp"

#include "tap/drivers.hpp"

#include "communication/can/turret/turret_mcb_can_comm.hpp"

namespace src::control::turret
{
SentryTurretSubsystem::SentryTurretSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorInterface *pitchMotorBottom,
    tap::motor::MotorInterface *yawMotorBottom,
    tap::motor::MotorInterface *pitchMotorTop,
    tap::motor::MotorInterface *yawMotorTop,
    const TurretMotorConfig &pitchMotorBottomConfig,
    const TurretMotorConfig &yawMotorBottomConfig,
    const TurretMotorConfig &pitchMotorTopConfig,
    const TurretMotorConfig &yawMotorTopConfig,
    const src::can::TurretMCBCanComm *turretMCB)
    : tap::control::Subsystem(drivers),
      pitchMotorBottom(pitchMotorBottom, pitchMotorBottomConfig),
      yawMotorBottom(yawMotorBottom, yawMotorBottomConfig),
      pitchMotorTop(pitchMotorTop, pitchMotorTopConfig),
      yawMotorTop(yawMotorTop, yawMotorTopConfig),
      turretMCB(turretMCB)
{
}
void SentryTurretSubsystem::initialize()
{
    pitchMotorBottom.initialize();
    yawMotorBottom.initialize();
    pitchMotorTop.initialize();
    yawMotorTop.initialize();
}

void SentryTurretSubsystem::refresh()
{
    pitchMotorBottom.updateMotorAngle();
    yawMotorBottom.updateMotorAngle();
    pitchMotorTop.updateMotorAngle();
    yawMotorTop.updateMotorAngle();
}

}  // namespace src::control::turret
