#include "chassis_orient_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "robot/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

float orientPidval;

namespace src::chassis
{
ChassisOrientDriveCommand::ChassisOrientDriveCommand(
    ChassisSubsystem* chassis,
    src::control::ControlOperatorInterface* operatorInterface)
    : chassis(chassis),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(chassis);
    orientPid = modm::Pid<float>(1.0, 0, 0, 0, 1.0);
}

void ChassisOrientDriveCommand::execute()
{
    auto scale = [](float raw) -> float {
        return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
    };
    float updateVal = chassis->getChassisTurretOffset();
    short sign = 1;
    if (updateVal < 0)
    {
        sign = -1;
    }

    orientPid.update(abs(updateVal));
    orientPidval = orientPid.getValue();
    chassis->setVelocityTurretDrive(
        scale(operatorInterface->getDrivetrainVerticalTranslation()),
        -scale(operatorInterface->getDrivetrainHorizontalTranslation()),
        scale(orientPid.getValue() * sign));
}

void ChassisOrientDriveCommand::end(bool interrupted) { chassis->setVelocityTurretDrive(0, 0, 0); }
};  // namespace src::chassis