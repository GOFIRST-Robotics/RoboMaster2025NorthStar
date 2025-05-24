#ifdef TARGET_HERO

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "robot/hero/hero_drivers.hpp"

#include "drivers_singleton.hpp"

// chasis
#include "control/chassis/chassis_field_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/constants/chassis_constants.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "robot/hero/hero_agitator_shoot_command.hpp"
#include "robot/hero/hero_agitator_subsystem.hpp"

// turret
#include "communication/RevMotorTester.hpp"
#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "control/turret/user/turret_quick_turn_command.hpp"
#include "control/turret/user/turret_user_control_command.hpp"
#include "control/turret/user/turret_user_world_relative_command.hpp"

// flywheel
#include "robot/hero/hero_flywheel_constants.hpp"
#include "robot/hero/hero_flywheel_run_command.hpp"
#include "robot/hero/hero_flywheel_subsystem.hpp"

// imu
#include "control/imu/imu_calibrate_command.hpp"

// safe disconnect
#include "control/safe_disconnect.hpp"

// governor
#include "tap/control/governor/governor_limited_command.hpp"

#include "control/governor/fire_rate_limit_governor.hpp"
#include "control/governor/heat_limit_governor.hpp"
#include "control/governor/ref_system_projectile_launched_governor.hpp"
#include "robot/hero/hero_flywheel_on_governor.hpp"

#include "ref_system_constants.hpp"

using tap::can::CanBus;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::control::turret;
using namespace src::control;
using namespace src::agitator;
using namespace src::control::agitator;
using namespace src::hero;
using namespace src::control;
using namespace src::control::flywheel;
using namespace src::control::governor;
using namespace tap::control::governor;

driversFunc drivers = DoNotUse_getDrivers;

namespace hero_control
{
inline src::can::TurretMCBCanComm &getTurretMCBCanComm() { return drivers()->turretMCBCanCommBus2; }

// flywheel
HeroFlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, DOWN_MOTOR_ID, CAN_BUS);

HeroFlywheelRunCommand heroFlywheelRunCommand(&flywheel);

ToggleCommandMapping fPressed(
    drivers(),
    {&heroFlywheelRunCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::F})));

// agitator subsystem
HeroAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_CONFIG,
    constants::AGITATOR_PID_CONFIG);

// agitator commands
HeroAgitatorShootCommand agitatorShootCommand(&agitator);

// agitator governors
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    constants::HEAT_LIMIT_BUFFER);

HeroFlywheelOnGovernor flywheelOnGovernor(flywheel);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

ManualFireRateReselectionManager manualFireRateReselectionManager;

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    agitatorShootCommand,
    {&refSystemProjectileLaunchedGovernor, &fireRateLimitGovernor, &flywheelOnGovernor});

HoldRepeatCommandMapping leftMousePressed(
    drivers(),
    {&rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched},
    RemoteMapState(RemoteMapState::MouseButton::LEFT),
    false);

// turret subsystem
tap::motor::DjiMotor pitchMotor(drivers(), PITCH_MOTOR_ID, CAN_BUS_MOTORS, true, "PitchMotor");

tap::motor::DjiMotor yawMotor(drivers(), YAW_MOTOR_ID, CAN_BUS_MOTORS, false, "YawMotor");

// chassis subsystem
src::chassis::ChassisSubsystem chassisSubsystem(
    drivers(),
    src::chassis::ChassisConfig{
        .leftFrontId = src::chassis::LEFT_FRONT_MOTOR_ID,
        .leftBackId = src::chassis::LEFT_BACK_MOTOR_ID,
        .rightBackId = src::chassis::RIGHT_BACK_MOTOR_ID,
        .rightFrontId = src::chassis::RIGHT_FRONT_MOTOR_ID,
        .canBus = CanBus::CAN_BUS2,
        .wheelVelocityPidConfig = modm::Pid<float>::Parameter(
            src::chassis::VELOCITY_PID_KP,
            src::chassis::VELOCITY_PID_KI,
            src::chassis::VELOCITY_PID_KD,
            src::chassis::VELOCITY_PID_MAX_ERROR_SUM),
    },
    &drivers()->turretMCBCanCommBus2,
    &yawMotor);

src::chassis::ChassisFieldCommand chassisFieldCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

TurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

// turret controlers
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

tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

// algorithms::WorldFramePitchTurretImuCascadePidTurretController
// worldFramePitchTurretImuController(
//     getTurretMCBCanComm(),
//     turret.pitchMotor,
//     worldFramePitchTurretImuPosPid,
//     worldFramePitchTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

// algorithms::WorldFrameYawTurretImuCascadePidTurretController
// worldFrameYawTurretImuController(
//     getTurretMCBCanComm(),
//     turret.yawMotor,
//     worldFrameYawTurretImuPosPid,
//     worldFrameYawTurretImuVelPid);

// turret commands
// user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
//     drivers(),
//     drivers()->controlOperatorInterface,
//     &turret,
//     &worldFrameYawChassisImuController,
//     &worldFramePitchChassisImuController,
//     &worldFrameYawTurretImuController,
//     &worldFramePitchTurretImuController,
//     USER_YAW_INPUT_SCALAR,
//     USER_PITCH_INPUT_SCALAR);

// imu commands
// imu::ImuCalibrateCommand imuCalibrateCommand(
//     drivers(),
//     {{
//         &getTurretMCBCanComm(),
//         &turret,
//         &chassisFrameYawTurretController,
//         &chassisFramePitchTurretController,
//         true,
//     }},
//     &chassisSubsystem);

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

void initializeSubsystems(Drivers *drivers)
{
    chassisSubsystem.initialize();
    agitator.initialize();
    turret.initialize();
    flywheel.initialize();
}

void registerHeroSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&flywheel);
}

void setDefaultHeroCommands(Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(&chassisFieldCommand);
    // turret.setDefaultCommand(&turretUserWorldRelativeCommand);
}

void startHeroCommands(Drivers *drivers)
{
    // drivers->commandScheduler.addCommand(&imuCalibrateCommand);
}

void registerHeroIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&leftMousePressed);
    drivers->commandMapper.addMap(&fPressed);
}
}  // namespace hero_control

namespace src::hero
{
void initSubsystemCommands(src::hero::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &hero_control::remoteSafeDisconnectFunction);
    hero_control::initializeSubsystems(drivers);
    hero_control::registerHeroSubsystems(drivers);
    hero_control::setDefaultHeroCommands(drivers);
    hero_control::startHeroCommands(drivers);
    hero_control::registerHeroIoMappings(drivers);
}
}  // namespace src::hero

#endif  // TARGET_HERO