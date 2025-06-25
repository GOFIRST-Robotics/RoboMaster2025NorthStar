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

#include "sentry_turret_user_control_command.hpp"

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/drivers.hpp"

#include "control/turret/turret_subsystem.hpp"
#include "robot/control_operator_interface.hpp"

using tap::algorithms::WrappedFloat;

namespace src::control::turret::user
{
SentryTurretUserControlCommand::SentryTurretUserControlCommand(
    tap::Drivers *drivers,
    ControlOperatorInterface &controlOperatorInterface,
    SentryTurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawControllerBottom,
    algorithms::TurretPitchControllerInterface *pitchControllerBottom,
    algorithms::TurretYawControllerInterface *yawControllerTop,
    algorithms::TurretPitchControllerInterface *pitchControllerTop,
    float userYawInputScalar,
    float userPitchInputScalar,
    float DELTA_MAX)
    : drivers(drivers),
      controlOperatorInterface(controlOperatorInterface),
      turretSubsystem(turretSubsystem),
      yawControllerBottom(yawControllerBottom),
      pitchControllerBottom(pitchControllerBottom),
      yawControllerTop(yawControllerTop),
      pitchControllerTop(pitchControllerTop),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      DELTA_MAX(DELTA_MAX)

{
    addSubsystemRequirement(turretSubsystem);
}
bool SentryTurretUserControlCommand::isReady() { return !isFinished(); }

void SentryTurretUserControlCommand::initialize()
{
    yawControllerBottom->initialize();
    pitchControllerBottom->initialize();
    yawControllerTop->initialize();
    pitchControllerTop->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
    yawSetpointTop = yawControllerTop->getSetpoint().getUnwrappedValue();
    if (!turretSubsystem->offsets)
    {
        turretSubsystem->setOffsets(
            yawControllerBottom->getSetpoint().getUnwrappedValue(),
            yawControllerBottom->getMeasurement().getUnwrappedValue(),
            yawControllerTop->getMeasurement().getUnwrappedValue());
    }
    comp = yawControllerTop->getSetpoint().getUnwrappedValue() +
           yawControllerBottom->getMeasurement().getUnwrappedValue() -
           turretSubsystem->bottomMeasurementOffset;
}

void SentryTurretUserControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    WrappedFloat pitchSetpointBottom = pitchControllerBottom->getSetpoint();

    WrappedFloat yawSetpointBottom = yawControllerBottom->getSetpoint();

    if (controlOperatorInterface.isRightSwitchDown())
    {
        pitchSetpointBottom +=
            userPitchInputScalar * controlOperatorInterface.getTurretPitchInput(0);

        yawSetpointBottom += userYawInputScalar * controlOperatorInterface.getTurretYawInput(0);
    }

    WrappedFloat pitchSetpointTop = pitchControllerTop->getSetpoint();

    float bottomMeasurement = yawControllerBottom->getMeasurement().getUnwrappedValue() -
                              turretSubsystem->bottomMeasurementOffset;

    if (controlOperatorInterface.isRightSwitchUp())
    {
        pitchSetpointTop = pitchControllerTop->getSetpoint() +
                           userPitchInputScalar * controlOperatorInterface.getTurretPitchInput(0);

        float delta =
            -(yawControllerTop->getMeasurement().getUnwrappedValue() -
              turretSubsystem->topMeasurementOffset);

        float input = userPitchInputScalar * controlOperatorInterface.getTurretYawInput(0);

        if (delta <= -DELTA_MAX)
        {
            comp = bottomMeasurement + DELTA_MAX;
        }
        else if (delta >= DELTA_MAX)
        {
            comp = bottomMeasurement - DELTA_MAX;
        }

        if (yawSetpointTop + input < DELTA_MAX && yawSetpointTop + input > -DELTA_MAX)
        {
            comp += input;
        }
        else if (input != 0)
        {
            comp = getSign(input) * DELTA_MAX + bottomMeasurement;
        }

        yawSetpointTop = limitVal(-(bottomMeasurement) + comp, -DELTA_MAX, DELTA_MAX);

        if (abs(yawSetpointTop) == DELTA_MAX && input != 0 && yawSetpointTop + input < DELTA_MAX &&
            yawSetpointTop + input > -DELTA_MAX)
        {
            yawSetpointTop += input;
        }
    }
    yawControllerBottom->runController(dt, yawSetpointBottom);

    pitchControllerBottom->runController(dt, pitchSetpointBottom);

    yawControllerTop->runController(dt, Angle(yawSetpointTop));

    pitchControllerTop->runController(dt, pitchSetpointTop);
}

bool SentryTurretUserControlCommand::isFinished() const
{
    return !pitchControllerBottom->isOnline() && !yawControllerBottom->isOnline() &&
           !pitchControllerTop->isOnline() && !yawControllerTop->isOnline();
}

void SentryTurretUserControlCommand::end(bool)
{
    turretSubsystem->yawMotorBottom.setMotorOutput(0);
    turretSubsystem->pitchMotorBottom.setMotorOutput(0);
    turretSubsystem->yawMotorTop.setMotorOutput(0);
    turretSubsystem->pitchMotorTop.setMotorOutput(0);
}

}  // namespace src::control::turret::user
