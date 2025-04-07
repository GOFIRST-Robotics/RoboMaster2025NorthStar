#ifdef TARGET_STANDARD

#include "tap/drivers.hpp"
#include "drivers_singleton.hpp"
#include "../../robot-type/robot_type.hpp"

#include "tap/util_macros.hpp"
#include "robot/standard/standard_drivers.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
//chasis
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_drive_command.hpp"
//agitator
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"
//turret
#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "control/turret/user/turret_quick_turn_command.hpp"
#include "control/turret/user/turret_user_world_relative_command.hpp"
#include "control/turret/user/turret_user_control_command.hpp"

//flywheel
#include "communication/RevMotorTester.hpp"

#include "robot/standard/standard_turret_subsystem.hpp"

#include "control/chassis/constants/chassis_constants.hpp"

#include "control/imu/imu_calibrate_command.hpp"

using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::motor::MotorId;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::standard;
using namespace src::control::turret;
using namespace src::control;
using namespace src::agitator;
using namespace src::control::agitator;
using namespace Communications::Rev;

driversFunc drivers = DoNotUse_getDrivers;

namespace standard_control
{

    inline src::can::TurretMCBCanComm &getTurretMCBCanComm()
    {
        return drivers()->turretMCBCanCommBus1;
    }
    //chassis
    src::chassis::ChassisSubsystem chassisSubsystem(
                  drivers(),
                  src::chassis::ChassisConfig{
                    .leftFrontId = src::chassis::LEFT_FRONT_MOTOR_ID,
                    .leftBackId = src::chassis::LEFT_BACK_MOTOR_ID,
                    .rightBackId = src::chassis::RIGHT_BACK_MOTOR_ID,
                    .rightFrontId = src::chassis::RIGHT_FRONT_MOTOR_ID,
                    .canBus = CanBus::CAN_BUS1,
                    .wheelVelocityPidConfig = modm::Pid<float>::Parameter(src::chassis::VELOCITY_PID_KP,
                                                                          src::chassis::VELOCITY_PID_KI,
                                                                          src::chassis::VELOCITY_PID_KD,
                                                                          src::chassis::VELOCITY_PID_MAX_ERROR_SUM),
                  },
                  &drivers()->turretMCBCanCommBus1);

    src::chassis::ChassisDriveCommand chassisDriveCommand(
        &chassisSubsystem,
        &drivers()->controlOperatorInterface);

    //agitator
    VelocityAgitatorSubsystem agitator(
        drivers(),
        constants::AGITATOR_PID_CONFIG,
        constants::AGITATOR_CONFIG);

    ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

    UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);
    
    MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
        *drivers(),
        agitator,
        rotateAgitator,
        unjamAgitator);

    HoldRepeatCommandMapping leftMousePressed(
        drivers(),
        {&rotateAndUnjamAgitator},
        RemoteMapState(RemoteMapState::MouseButton::LEFT),
        false);

    //turret
    tap::motor::DjiMotor pitchMotor(
        drivers(), 
        PITCH_MOTOR_ID, 
        CAN_BUS_MOTORS, 
        true, "PitchMotor");
    
    tap::motor::DjiMotor yawMotor(
        drivers(), 
        YAW_MOTOR_ID, 
        CAN_BUS_MOTORS, 
        false, "YawMotor");
        
    StandardTurretSubsystem turret(
        drivers(),
        &pitchMotor,
        &yawMotor,
        PITCH_MOTOR_CONFIG,
        YAW_MOTOR_CONFIG,
        &getTurretMCBCanComm());

    algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
        turret.pitchMotor,
        chassis_rel::PITCH_PID_CONFIG);
    
    algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
        turret.yawMotor,
        chassis_rel::YAW_PID_CONFIG);
    
    algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(
        *drivers(),
        turret.yawMotor,
        world_rel_chassis_imu::YAW_PID_CONFIG);

    algorithms::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuController(
        *drivers(),
        turret.pitchMotor,
        world_rel_chassis_imu::PITCH_PID_CONFIG);
    
    tap::algorithms::SmoothPid worldFramePitchTurretImuPosPid(
        world_rel_turret_imu::PITCH_POS_PID_CONFIG);
    // tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidCv(
    //     world_rel_turret_imu::PITCH_POS_PID_AUTO_AIM_CONFIG);
    tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
        world_rel_turret_imu::PITCH_VEL_PID_CONFIG);
    
    // algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    //     getTurretMCBCanComm(),
    //     turret.pitchMotor,
    //     worldFramePitchTurretImuPosPid,
    //     worldFramePitchTurretImuVelPid);
    
    // algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerCv(
    //     getTurretMCBCanComm(),
    //     turret.pitchMotor,
    //     worldFramePitchTurretImuPosPidCv,
    //     worldFramePitchTurretImuVelPid);
    
    tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);
    tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);
    
    algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
        getTurretMCBCanComm(),
        turret.yawMotor,
        worldFrameYawTurretImuPosPid,
        worldFrameYawTurretImuVelPid);
    
    // turret commands
    user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
        drivers(),
        drivers()->controlOperatorInterface,
        &turret,
        &worldFrameYawChassisImuController,
        &worldFramePitchChassisImuController,
        &worldFrameYawChassisImuController,
        &worldFramePitchChassisImuController,
        // &worldFrameYawTurretImuController,
        // &worldFramePitchTurretImuController,
        USER_YAW_INPUT_SCALAR,
        USER_PITCH_INPUT_SCALAR);

    imu::ImuCalibrateCommand imuCalibrateCommand(
        drivers(),
        {{
            &getTurretMCBCanComm(),
            &turret,
            &chassisFrameYawTurretController,
            &chassisFramePitchTurretController,
            true,
        }},
        &chassisSubsystem);

    //flywheel
    RevMotorTester revMotorTester(drivers());
    
void initializeSubsystems(Drivers *drivers)
{
    chassisSubsystem.initialize();
    agitator.initialize();
    // m_FlyWheel.initialize();
    turret.initialize();
    revMotorTester.initialize();
}

void registerStandardSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitator);
    // drivers.commandScheduler.registerSubsystem(&m_FlyWheel);
    drivers->commandScheduler.registerSubsystem(&turret);
}

void setDefaultStandardCommands(Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(&chassisDriveCommand);
    // m_FlyWheel.setDefaultCommand(&m_FlyWheelCommand);
    turret.setDefaultCommand(&turretUserWorldRelativeCommand);
}

void startStandardCommands(Drivers *drivers) {
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
}

void registerStandardIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&leftMousePressed);
    // drivers.commandMapper.addMap(&rightMousePressed);
    // drivers.commandMapper.addMap(&leftSwitchUp);

}
}  // namespace standard_control

namespace src::standard
{
void initSubsystemCommands(src::standard::Drivers *drivers)
{
    standard_control::initializeSubsystems(drivers);
    standard_control::registerStandardSubsystems(drivers);
    standard_control::setDefaultStandardCommands(drivers);
    standard_control::startStandardCommands(drivers);
    standard_control::registerStandardIoMappings(drivers);
}
} //namespace src::standard

#endif