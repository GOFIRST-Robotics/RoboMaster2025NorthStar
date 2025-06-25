#ifdef TARGET_STANDARD

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/sequential_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/cycle_state_command_mapping.hpp"
#include "control/dummy_subsystem.hpp"
#include "robot/standard/standard_drivers.hpp"

#include "drivers_singleton.hpp"

// chasis
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_drive_distance_command.hpp"
#include "control/chassis/chassis_field_command.hpp"
#include "control/chassis/chassis_orient_drive_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_wiggle_command.hpp"
#include "control/chassis/constants/chassis_constants.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"

// turret
#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_can_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "control/turret/user/turret_quick_turn_command.hpp"
#include "control/turret/user/turret_user_control_command.hpp"
#include "control/turret/user/turret_user_world_relative_command.hpp"
#include "robot/standard/standard_turret_subsystem.hpp"

// cv
#include "control/agitator/multi_shot_cv_command_mapping.hpp"
#include "control/governor/cv_on_target_governor.hpp"
#include "control/turret/cv/turret_cv_control_command.hpp"

// flywheel
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/flywheel_run_command.hpp"
#include "control/flywheel/flywheel_subsystem.hpp"

// hopper
#include "control/hopper/hopper_subsystem.hpp"
#include "control/hopper/hopper_toggle_command.hpp"

// imu
#include "control/imu/imu_calibrate_command.hpp"

// safe disconnect
#include "control/safe_disconnect.hpp"

// governor
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"

#include "control/governor/fire_rate_limit_governor.hpp"
#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/flywheel_on_governor.hpp"
#include "control/governor/heat_limit_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "control/governor/ref_system_projectile_launched_governor.hpp"

#include "ref_system_constants.hpp"

// HUD
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "control/clientDisplay/client_display_command.hpp"
#include "control/clientDisplay/client_display_subsystem.hpp"
#include "control/clientDisplay/indicators/ammo_indicator.hpp"
#include "control/clientDisplay/indicators/circle_crosshair.hpp"
#include "control/clientDisplay/indicators/cv_aiming_indicator.hpp"
#include "control/clientDisplay/indicators/flywheel_indicator.hpp"
#include "control/clientDisplay/indicators/hud_indicator.hpp"
#include "control/clientDisplay/indicators/shooting_mode_indicator.hpp"
#include "control/clientDisplay/indicators/text_hud_indicators.hpp"
#include "control/clientDisplay/indicators/vision_indicator.hpp"

using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::standard;
using namespace src::control::turret;
using namespace src::control;
using namespace src::flywheel;
using namespace src::control::flywheel;
using namespace src::agitator;
using namespace src::control::agitator;
using namespace src::control::governor;
using namespace tap::control::governor;
using namespace src::control::client_display;
using namespace tap::communication::serial;
using namespace src::control::hopper;

driversFunc drivers = DoNotUse_getDrivers;

namespace standard_control
{
DummySubsystem dummySubsystem(drivers());

inline src::can::TurretMCBCanComm &getTurretMCBCanComm() { return drivers()->turretMCBCanCommBus2; }

// flywheel subsystem
FlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, UP_MOTOR_ID, CAN_BUS);

// flywheel commands
FlywheelRunCommand flywheelRunCommand(&flywheel);

// flywheel mappings
HoldCommandMapping leftSwitchUpFlywheels(
    drivers(),
    {&flywheelRunCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// turret subsystem
tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "PitchMotor",
    false,
    1,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    CAN_BUS_MOTORS,
    true,
    "YawMotor",
    false,
    1,
    YAW_MOTOR_CONFIG.startEncoderValue);

StandardTurretSubsystem turret(
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

tap::algorithms::SmoothPid worldFramePitchTurretPosPid(world_rel_turret_imu::PITCH_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretVelPid(world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretPosPid(world_rel_turret_imu::YAW_POS_PID_CONFIG);

tap::algorithms::SmoothPid worldFrameYawTurretVelPid(world_rel_turret_imu::YAW_VEL_PID_CONFIG);

// for imu can com giving imu data from turret to chassis
algorithms::
    WorldFramePitchTurretCanImuCascadePidTurretController worldFramePitchTurretCanImuController(
        getTurretMCBCanComm(),
        turret.pitchMotor,
        worldFramePitchTurretPosPid,
        worldFramePitchTurretVelPid);

algorithms::WorldFrameYawTurretCanImuCascadePidTurretController worldFrameYawTurretCanImuController(
    getTurretMCBCanComm(),
    turret.yawMotor,
    worldFrameYawTurretPosPid,
    worldFrameYawTurretVelPid);

// for imu fixed on turret
algorithms::WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
    *drivers(),
    turret.pitchMotor,
    worldFramePitchTurretPosPid,
    worldFramePitchTurretVelPid);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
    *drivers(),
    turret.yawMotor,
    worldFrameYawTurretPosPid,
    worldFrameYawTurretVelPid);

// turret commands
user::TurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchChassisImuController,  //&worldFramePitchTurretImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

cv::TurretCVControlCommand turretCVControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &turret,
    &worldFrameYawTurretImuController,
    &worldFramePitchChassisImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

user::TurretUserWorldRelativeCommand turretUserWorldRelativeCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &worldFrameYawChassisImuController,
    &worldFramePitchChassisImuController,
    &worldFrameYawTurretCanImuController,
    &worldFramePitchTurretCanImuController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR);

// agitator subsystem
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

// agitator commands
ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// agitator governors
HeatLimitGovernor heatLimitGovernor(
    *drivers(),
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    constants::HEAT_LIMIT_BUFFER);

FlywheelOnGovernor flywheelOnGovernor(flywheel);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

ManualFireRateReselectionManager manualFireRateReselectionManager;

SetFireRateCommand setFireRateCommandFullAuto(
    &dummySubsystem,
    manualFireRateReselectionManager,
    40,
    &rotateAgitator);
SetFireRateCommand setFireRateCommand10RPS(
    &dummySubsystem,
    manualFireRateReselectionManager,
    10,
    &rotateAgitator);

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor, &fireRateLimitGovernor, &flywheelOnGovernor});

CvOnTargetGovernor cvOnTargetGovernor(drivers(), drivers()->visionComms, turretCVControlCommand);

CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rPressedCVGovernoreToggle(
    drivers(),
    RemoteMapState({Remote::Key::R}),
    true,
    &cvOnTargetGovernor,
    &CvOnTargetGovernor::setGovernorEnabled);

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatAndCVLimiting(
    {&agitator},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched,
    {&heatLimitGovernor, &cvOnTargetGovernor});

MultiShotCvCommandMapping leftSwitchDownPressedShoot(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatAndCVLimiting,
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
    &manualFireRateReselectionManager,
    cvOnTargetGovernor,
    &rotateAgitator);

CycleStateCommandMapping<
    MultiShotCvCommandMapping::LaunchMode,
    MultiShotCvCommandMapping::NUM_SHOOTER_STATES,
    MultiShotCvCommandMapping>
    gOrVPressedCycleShotSpeed(
        drivers(),
        RemoteMapState({Remote::Key::G}),
        MultiShotCvCommandMapping::SINGLE,
        &leftSwitchDownPressedShoot,
        &MultiShotCvCommandMapping::setShooterState,
        RemoteMapState({Remote::Key::V}));

// agitator mappings
ToggleCommandMapping vPressed(
    drivers(),
    {&setFireRateCommandFullAuto},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::V})));

ToggleCommandMapping gPressed(
    drivers(),
    {&setFireRateCommand10RPS},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::G})));

// HoldRepeatCommandMapping leftMousePressedShoot(
//     drivers(),
//     {&rotateAndUnjamAgitator},  // TODO
//     RemoteMapState(RemoteMapState::MouseButton::LEFT),
//     false);

// chassis subsystem
src::chassis::ChassisSubsystem chassisSubsystem(
    drivers(),
    src::chassis::ChassisConfig{
        .leftFrontId = src::chassis::LEFT_FRONT_MOTOR_ID,
        .leftBackId = src::chassis::LEFT_BACK_MOTOR_ID,
        .rightBackId = src::chassis::RIGHT_BACK_MOTOR_ID,
        .rightFrontId = src::chassis::RIGHT_FRONT_MOTOR_ID,
        .canBus = CanBus::CAN_BUS1,
        .wheelVelocityPidConfig = modm::Pid<float>::Parameter(
            src::chassis::VELOCITY_PID_KP,
            src::chassis::VELOCITY_PID_KI,
            src::chassis::VELOCITY_PID_KD,
            src::chassis::VELOCITY_PID_MAX_ERROR_SUM),
    },
    &drivers()->turretMCBCanCommBus2,
    &yawMotor);

src::chassis::ChassisDriveCommand chassisDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::chassis::ChassisOrientDriveCommand chassisOrientDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::chassis::ChassisBeybladeCommand chassisBeyBladeSlowCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1,
    -1,
    M_PI_2,
    true);

src::chassis::ChassisBeybladeCommand chassisBeyBladeFastCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1,
    -1,
    M_PI,
    true);

src::chassis::ChassisWiggleCommand chassisWiggleCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    1.0f,
    M_TWOPI);

src::chassis::ChassisDriveDistanceCommand driveDist1(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    3,
    0,
    0.2);

src::chassis::ChassisDriveDistanceCommand driveDist2(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    0,
    3,
    0.2);

src::chassis::ChassisDriveDistanceCommand driveDist3(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    -3,
    0,
    0.2);

src::chassis::ChassisDriveDistanceCommand driveDist4(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface,
    0,
    -3,
    0.2);

// Chassis Governors

FiredRecentlyGovernor firedRecentlyGovernor(drivers(), 5000);

PlateHitGovernor plateHitGovernor(drivers(), 5000);

// GovernorWithFallbackCommand<2> beyBladeSlowOutOfCombat(
//     {&chassisSubsystem},
//     chassisBeyBladeSlowCommand,
//     chassisBeyBladeFastCommand,
//     {&firedRecentlyGovernor, &plateHitGovernor},
//     false);

// chassis Mappings
ToggleCommandMapping bPressedNotCntlPressedBeyblade(
    drivers(),
    {&chassisBeyBladeFastCommand},
    RemoteMapState({Remote::Key::B}, {Remote::Key::CTRL}));

ToggleCommandMapping rPressedOrientDrive(
    drivers(),
    {&chassisOrientDriveCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::R})));

ToggleCommandMapping zPressedWiggle(
    drivers(),
    {&chassisWiggleCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::Z})));

HoldRepeatCommandMapping rightSwiitchDownBeyblade(
    drivers(),
    {&chassisBeyBladeFastCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN),
    true);

HopperSubsystem hopperSubsystem(drivers(), 0.0585f, 0.025, tap::gpio::Pwm::Pin::C7);

HopperToggleCommand hopperToggleCommand(&hopperSubsystem);

ToggleCommandMapping ctrlVPressedHopperToggle(
    drivers(),
    {&hopperToggleCommand},
    RemoteMapState(RemoteMapState({Remote::Key::CTRL, Remote::Key::V})));

// imu commands
imu::ImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &turret,
        &chassisFrameYawTurretController,
        &chassisFramePitchTurretController,
        true,
    }},
    &chassisSubsystem);

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

ToggleCommandMapping xPressedIMUCalibrate(
    drivers(),
    {&imuCalibrateCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::X})));

// HUD
ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

AmmoIndicator ammoIndicator(refSerialTransmitter, drivers()->refSerial);

VisionIndicator visionIndicator(refSerialTransmitter, drivers()->refSerial, drivers()->visionComms);

CircleCrosshair circleCrosshair(refSerialTransmitter);

FlywheelIndicator flyWheelIndicator(refSerialTransmitter, drivers()->refSerial, flywheelOnGovernor);

ShootingModeIndicator shootingModeIndicator(
    refSerialTransmitter,
    drivers()->refSerial,
    leftSwitchDownPressedShoot);

CvAimingIndicator cvAimingIndicator(refSerialTransmitter, drivers()->refSerial, cvOnTargetGovernor);

TextHudIndicators textHudIndicators(
    *drivers(),
    agitator,
    // imuCalibrateCommand,
    {&chassisWiggleCommand, &chassisBeyBladeFastCommand},
    refSerialTransmitter);

std::vector<HudIndicator *> hudIndicators = {
    &ammoIndicator,
    &circleCrosshair,
    &textHudIndicators,
    &visionIndicator,
    &flyWheelIndicator,
    &shootingModeIndicator,
    &cvAimingIndicator};

ClientDisplayCommand clientDisplayCommand(*drivers(), clientDisplay, hudIndicators);

PressCommandMapping bCtrlPressedClientDisplay(
    drivers(),
    {&clientDisplayCommand},
    RemoteMapState({Remote::Key::CTRL, Remote::Key::B}));

void initializeSubsystems(Drivers *drivers)
{
    dummySubsystem.initialize();
    chassisSubsystem.initialize();
    agitator.initialize();
    flywheel.initialize();
    turret.initialize();
    hopperSubsystem.initialize();
}

void registerStandardSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&dummySubsystem);
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&flywheel);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&hopperSubsystem);
}

void setDefaultStandardCommands(Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(&chassisDriveCommand);  // chassisOrientDriveCommand);
    // turret.setDefaultCommand(&turretUserWorldRelaftiveCommand); // for use when can comm is
    // running
    turret.setDefaultCommand(&turretUserControlCommand);  // when mcb is mounted on turret
}

void startStandardCommands(Drivers *drivers)
{
    drivers->bmi088.setMountingTransform(
        tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(-45), 0));
    drivers->commandScheduler.addCommand(&imuCalibrateCommand);
}

void registerStandardIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&leftSwitchDownPressedShoot);
    // drivers->commandMapper.addMap(&vPressed);
    drivers->commandMapper.addMap(&leftSwitchUpFlywheels);
    drivers->commandMapper.addMap(&bPressedNotCntlPressedBeyblade);
    // drivers->commandMapper.addMap(&gPressed);
    drivers->commandMapper.addMap(&xPressedIMUCalibrate);
    drivers->commandMapper.addMap(&rPressedCVGovernoreToggle);
    drivers->commandMapper.addMap(&gOrVPressedCycleShotSpeed);
    drivers->commandMapper.addMap(&ctrlVPressedHopperToggle);
    drivers->commandMapper.addMap(&zPressedWiggle);
    drivers->commandMapper.addMap(&rPressedOrientDrive);
    drivers->commandMapper.addMap(&bCtrlPressedClientDisplay);
    drivers->commandMapper.addMap(&rightSwiitchDownBeyblade);
}
}  // namespace standard_control

namespace src::standard
{
imu::ImuCalibrateCommandBase *getImuCalibrateCommand()
{
    return &standard_control::imuCalibrateCommand;
}

void initSubsystemCommands(src::standard::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &standard_control::remoteSafeDisconnectFunction);
    standard_control::initializeSubsystems(drivers);
    standard_control::registerStandardSubsystems(drivers);
    standard_control::setDefaultStandardCommands(drivers);
    standard_control::startStandardCommands(drivers);
    standard_control::registerStandardIoMappings(drivers);
}
}  // namespace src::standard

#endif