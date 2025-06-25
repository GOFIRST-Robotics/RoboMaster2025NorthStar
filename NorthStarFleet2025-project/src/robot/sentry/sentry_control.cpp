#ifdef TARGET_SENTRY

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "../../robot-type/robot_type.hpp"
#include "control/cycle_state_command_mapping.hpp"
#include "control/dummy_subsystem.hpp"
#include "robot/sentry/sentry_drivers.hpp"

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
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
// #include "control/turret/user/turret_quick_turn_command.hpp"
#include "control/turret/algorithms/world_frame_pitch_chassis_imu_comp_turret_controller.hpp"
#include "robot/sentry/sentry_turret_subsystem.hpp"
#include "robot/sentry/sentry_turret_user_world_relative_command.hpp"

// cv
#include "control/agitator/multi_shot_cv_command_mapping.hpp"
#include "control/governor/cv_on_target_governor.hpp"
#include "robot/sentry/sentry_cv_manager_command.hpp"
#include "robot/sentry/sentry_scan_command.hpp"
#include "robot/sentry/sentry_turret_cv_control_command.hpp"

// flywheel
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/flywheel_run_command.hpp"
#include "control/flywheel/flywheel_subsystem.hpp"

// imu
#include "robot/sentry/sentry_imu_calibrate_command.hpp"

// governor
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"

#include "control/governor/fire_rate_limit_governor.hpp"
#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/flywheel_on_governor.hpp"
#include "control/governor/heat_limit_governor.hpp"
#include "control/governor/match_running_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "control/governor/ref_system_projectile_launched_governor.hpp"

// safe disconnect
#include "control/safe_disconnect.hpp"

using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::motor::MotorId;

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::sentry;
using namespace src::control::turret;
using namespace src::control;
using namespace src::flywheel;
using namespace src::control::flywheel;
using namespace src::agitator;
using namespace src::control::agitator;
using namespace src::control::governor;
using namespace tap::control::governor;

driversFunc drivers = DoNotUse_getDrivers;

namespace sentry_control
{
DummySubsystem dummySubsystem(drivers());

inline src::can::TurretMCBCanComm &getTurretMCBCanComm() { return drivers()->turretMCBCanCommBus2; }

// flywheel
FlywheelSubsystem flywheelBottom(
    drivers(),
    LEFT_MOTOR_ID_BOTTOM,
    RIGHT_MOTOR_ID_BOTTOM,
    UP_MOTOR_ID_BOTTOM,
    CAN_BUS);

FlywheelRunCommand flywheelRunCommandBottom(&flywheelBottom);

ToggleCommandMapping fNotCtrlPressed(
    drivers(),
    {&flywheelRunCommandBottom},
    RemoteMapState({Remote::Key::F}, {Remote::Key::CTRL}));

FlywheelSubsystem flywheelTop(
    drivers(),
    LEFT_MOTOR_ID_TOP,
    RIGHT_MOTOR_ID_TOP,
    UP_MOTOR_ID_TOP,
    CAN_BUS);

FlywheelRunCommand flywheelRunCommandTop(&flywheelTop);

ToggleCommandMapping fCtrlPressed(
    drivers(),
    {&flywheelRunCommandTop},
    RemoteMapState({Remote::Key::F, Remote::Key::CTRL}));

// turret subsystem
tap::motor::DjiMotor pitchMotorBottom(
    drivers(),
    PITCH_MOTOR_BOTTOM_ID,
    CAN_BUS_MOTORS,
    true,
    "Pitch Motor Bottom",
    false,
    1,
    PITCH_MOTOR_CONFIG_BOTTOM.startEncoderValue);

tap::motor::DjiMotor yawMotorBottom(
    drivers(),
    YAW_MOTOR_BOTTOM_ID,
    CAN_BUS_MOTORS,
    true,
    "Yaw Motor Bottom",
    false,
    1,
    YAW_MOTOR_CONFIG_BOTTOM.startEncoderValue);

tap::motor::DjiMotor pitchMotorTop(
    drivers(),
    PITCH_MOTOR_TOP_ID,
    CAN_BUS_MOTORS,
    true,
    "Pitch Motor Top",
    false,
    1,
    PITCH_MOTOR_CONFIG_TOP.startEncoderValue);

tap::motor::DjiMotor yawMotorTop(
    drivers(),
    YAW_MOTOR_TOP_ID,
    CAN_BUS_MOTORS,
    true,
    "Yaw Motor Top",
    false,
    1,
    YAW_MOTOR_CONFIG_TOP.startEncoderValue);

SentryTurretSubsystem sentryTurrets(
    drivers(),
    &pitchMotorBottom,
    &yawMotorBottom,
    &pitchMotorTop,
    &yawMotorTop,
    PITCH_MOTOR_CONFIG_BOTTOM,
    YAW_MOTOR_CONFIG_BOTTOM,
    PITCH_MOTOR_CONFIG_TOP,
    YAW_MOTOR_CONFIG_TOP,
    &getTurretMCBCanComm());

// turret controlers
algorithms::ChassisFramePitchTurretController chassisFramePitchTurretControllerBottom(
    sentryTurrets.pitchMotorBottom,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFramePitchTurretController chassisFramePitchTurretControllerTop(
    sentryTurrets.pitchMotorTop,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretControllerBottom(
    sentryTurrets.yawMotorBottom,
    chassis_rel::YAW_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretControllerTop(
    sentryTurrets.yawMotorTop,
    chassis_rel::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuControllerBottom(
    *drivers(),
    sentryTurrets.yawMotorBottom,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFrameYawChassisImuTurretController worldFrameYawChassisImuControllerTop(
    *drivers(),
    sentryTurrets.yawMotorTop,
    world_rel_chassis_imu::YAW_PID_CONFIG);

algorithms::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuControllerBottom(
    *drivers(),
    sentryTurrets.pitchMotorBottom,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

algorithms::WorldFramePitchChassisImuTurretController worldFramePitchChassisImuControllerTop(
    *drivers(),
    sentryTurrets.pitchMotorTop,
    world_rel_chassis_imu::PITCH_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidBottom(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG_BOTTOM);

tap::algorithms::SmoothPid worldFramePitchTurretImuVelPidBottom(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG_BOTTOM);

algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerBottom(
        *drivers(),
        sentryTurrets.pitchMotorBottom,
        worldFramePitchTurretImuPosPidBottom,
        worldFramePitchTurretImuVelPidBottom);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidTop(
    world_rel_turret_imu::PITCH_POS_PID_CONFIG_TOP);

tap::algorithms::SmoothPid worldFramePitchTurretImuVelPidTop(
    world_rel_turret_imu::PITCH_VEL_PID_CONFIG_TOP);

algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerTop(
        *drivers(),
        sentryTurrets.pitchMotorTop,
        worldFramePitchTurretImuPosPidTop,
        worldFramePitchTurretImuVelPidTop);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidBottom(
    world_rel_turret_imu::YAW_POS_PID_CONFIG_BOTTOM);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidBottom(
    world_rel_turret_imu::YAW_VEL_PID_CONFIG_BOTTOM);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerBottom(
    *drivers(),
    sentryTurrets.yawMotorBottom,
    worldFrameYawTurretImuPosPidBottom,
    worldFrameYawTurretImuVelPidBottom);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidTop(
    world_rel_turret_imu::YAW_POS_PID_CONFIG_TOP);

tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidTop(
    world_rel_turret_imu::YAW_VEL_PID_CONFIG_TOP);

algorithms::WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerTop(
    *drivers(),
    sentryTurrets.yawMotorTop,
    worldFrameYawTurretImuPosPidTop,
    worldFrameYawTurretImuVelPidTop);

algorithms::
    WorldFramePitchChassisImuCompTurretController worldFramePitchChassisImuCompControllerTop(
        *drivers(),
        sentryTurrets.pitchMotorTop,
        world_rel_chassis_imu::PITCH_PID_CONFIG,
        &sentryTurrets);

// turret commands
user::SentryTurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &sentryTurrets,
    &worldFrameYawTurretImuControllerBottom,
    &worldFramePitchChassisImuControllerBottom,
    &chassisFrameYawTurretControllerTop,          // controler for top turret
    &worldFramePitchChassisImuCompControllerTop,  // worldFramePitchChassisImuControllerTop,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    M_TWOPI);  // +- offset max rads

cv::SentryTurretCVControlCommand turretCVControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &sentryTurrets,
    &worldFrameYawTurretImuControllerBottom,
    &worldFramePitchChassisImuControllerBottom,
    &chassisFrameYawTurretControllerTop,          // controler for top turret
    &worldFramePitchChassisImuCompControllerTop,  // worldFramePitchChassisImuControllerTop,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    .01,       // max error
    M_TWOPI);  // +- offset max rads

cv::SentryScanCommand turretScanCommand(
    drivers(),
    &sentryTurrets,
    &worldFrameYawTurretImuControllerBottom,
    &worldFramePitchChassisImuControllerBottom,
    &chassisFrameYawTurretControllerTop,          // controler for top turret
    &worldFramePitchChassisImuCompControllerTop,  // worldFramePitchChassisImuControllerTop,
    M_TWOPI,
    .01,  // max error
    .0016);

cv::SentryCvManagerCommand cvManagerCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    drivers()->visionComms,
    &sentryTurrets,
    &worldFrameYawTurretImuControllerBottom,
    &worldFramePitchChassisImuControllerBottom,
    &chassisFrameYawTurretControllerTop,          // controler for top turret
    &worldFramePitchChassisImuCompControllerTop,  // worldFramePitchChassisImuControllerTop,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    M_TWOPI,
    .01,  // max error
    .0016);

// user::SentryTurretUserWorldRelativeCommand turretsUserWorldRelativeCommand(
//     drivers(),
//     drivers()->controlOperatorInterface,
//     &sentryTurrets,
//     &worldFrameYawChassisImuControllerBottom,
//     &worldFramePitchChassisImuControllerBottom,
//     &worldFrameYawTurretImuControllerBottom,
//     &worldFramePitchTurretImuControllerBottom,
//     &chassisFrameYawTurretControllerTop,
//     &worldFramePitchChassisImuControllerTop,
//     &worldFramePitchTurretImuControllerTop,
//     USER_YAW_INPUT_SCALAR,
//     USER_PITCH_INPUT_SCALAR,
//     M_TWOPI);

ToggleCommandMapping xCtrlPressed(
    drivers(),
    {&turretScanCommand},
    RemoteMapState({Remote::Key::X, Remote::Key::CTRL}));

ToggleCommandMapping xPressed(
    drivers(),
    {&turretUserControlCommand},
    RemoteMapState({Remote::Key::X}));

HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVControlCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));

// agitator subsystem
VelocityAgitatorSubsystem agitatorBottom(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG_BOTTOM);

// agitator commands
ConstantVelocityAgitatorCommand rotateAgitatorBottom(
    agitatorBottom,
    constants::AGITATOR_ROTATE_CONFIG_BOTTOM);

UnjamSpokeAgitatorCommand unjamAgitatorBottom(
    agitatorBottom,
    constants::AGITATOR_UNJAM_CONFIG_BOTTOM);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitatorBottom(
    *drivers(),
    agitatorBottom,
    rotateAgitatorBottom,
    unjamAgitatorBottom);

// agitator governors
HeatLimitGovernor heatLimitGovernorBottom(*drivers(), barrelIdBottom, constants::HEAT_LIMIT_BUFFER);

FlywheelOnGovernor flywheelOnGovernorBottom(flywheelBottom);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernorBottom(
    drivers()->refSerial,
    barrelIdBottom);

ManualFireRateReselectionManager manualFireRateReselectionManagerBottom;

SetFireRateCommand setFireRateCommandFullAutoBottom(
    &dummySubsystem,
    manualFireRateReselectionManagerBottom,
    40,
    &rotateAgitatorBottom);
SetFireRateCommand setFireRateCommand10RPSBottom(
    &dummySubsystem,
    manualFireRateReselectionManagerBottom,
    10,
    &rotateAgitatorBottom);

FireRateLimitGovernor fireRateLimitGovernorBottom(manualFireRateReselectionManagerBottom);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunchedBottom(
    {&agitatorBottom},
    rotateAndUnjamAgitatorBottom,
    {&refSystemProjectileLaunchedGovernorBottom,
     &fireRateLimitGovernorBottom,
     &flywheelOnGovernorBottom});

CvOnTargetGovernor cvOnTargetGovernorBottom(
    drivers(),
    drivers()->visionComms,
    turretCVControlCommand,
    bottomID,
    true);

CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rNotCtrlPressed(
    drivers(),
    RemoteMapState({Remote::Key::R}, {Remote::Key::CTRL}),
    true,
    &cvOnTargetGovernorBottom,
    &CvOnTargetGovernor::setGovernorEnabled);

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatAndCVLimitingBottom(
    {&agitatorBottom},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunchedBottom,
    {&heatLimitGovernorBottom, &cvOnTargetGovernorBottom});

MultiShotCvCommandMapping leftMouseNotCtrlPressed(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatAndCVLimitingBottom,
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::CTRL}),
    &manualFireRateReselectionManagerBottom,
    cvOnTargetGovernorBottom,
    &rotateAgitatorBottom);

CycleStateCommandMapping<
    MultiShotCvCommandMapping::LaunchMode,
    MultiShotCvCommandMapping::NUM_SHOOTER_STATES,
    MultiShotCvCommandMapping>
    gOrVNotCtrlPressed(
        drivers(),
        RemoteMapState({Remote::Key::G}, {Remote::Key::CTRL}),
        MultiShotCvCommandMapping::SINGLE,
        &leftMouseNotCtrlPressed,
        &MultiShotCvCommandMapping::setShooterState,
        RemoteMapState({Remote::Key::V}, {Remote::Key::CTRL}));

// agitator mappings
ToggleCommandMapping vNotCtrlPressed(
    drivers(),
    {&setFireRateCommandFullAutoBottom},
    RemoteMapState(RemoteMapState({Remote::Key::V}, {Remote::Key::CTRL})));

ToggleCommandMapping gNotCtrlPressed(
    drivers(),
    {&setFireRateCommand10RPSBottom},
    RemoteMapState(RemoteMapState({Remote::Key::G}, {Remote::Key::CTRL})));

VelocityAgitatorSubsystem agitatorTop(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG_TOP);

// agitator commands
ConstantVelocityAgitatorCommand rotateAgitatorTop(
    agitatorTop,
    constants::AGITATOR_ROTATE_CONFIG_TOP);

UnjamSpokeAgitatorCommand unjamAgitatorTop(agitatorTop, constants::AGITATOR_UNJAM_CONFIG_TOP);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitatorTop(
    *drivers(),
    agitatorTop,
    rotateAgitatorTop,
    unjamAgitatorTop);

// agitator governors
HeatLimitGovernor heatLimitGovernorTop(*drivers(), barrelIdTop, constants::HEAT_LIMIT_BUFFER);

FlywheelOnGovernor flywheelOnGovernorTop(flywheelTop);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernorTop(
    drivers()->refSerial,
    barrelIdTop);

ManualFireRateReselectionManager manualFireRateReselectionManagerTop;

SetFireRateCommand setFireRateCommandFullAutoTop(
    &dummySubsystem,
    manualFireRateReselectionManagerTop,
    40,
    &rotateAgitatorTop);
SetFireRateCommand setFireRateCommand10RPSTop(
    &dummySubsystem,
    manualFireRateReselectionManagerTop,
    10,
    &rotateAgitatorTop);

FireRateLimitGovernor fireRateLimitGovernorTop(manualFireRateReselectionManagerTop);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunchedTop(
    {&agitatorTop},
    rotateAndUnjamAgitatorTop,
    {&refSystemProjectileLaunchedGovernorTop, &fireRateLimitGovernorTop, &flywheelOnGovernorTop});

CvOnTargetGovernor cvOnTargetGovernorTop(
    drivers(),
    drivers()->visionComms,
    turretCVControlCommand,
    topID,
    true);

CycleStateCommandMapping<bool, 2, CvOnTargetGovernor> rCtrlPressed(
    drivers(),
    RemoteMapState({Remote::Key::R, Remote::Key::CTRL}),
    true,
    &cvOnTargetGovernorTop,
    &CvOnTargetGovernor::setGovernorEnabled);

GovernorLimitedCommand<2> rotateAndUnjamAgitatorWithHeatAndCVLimitingTop(
    {&agitatorTop},
    rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunchedTop,
    {&heatLimitGovernorTop, &cvOnTargetGovernorTop});

MultiShotCvCommandMapping leftMouseCtrlPressed(
    *drivers(),
    rotateAndUnjamAgitatorWithHeatAndCVLimitingTop,
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::CTRL}),
    &manualFireRateReselectionManagerTop,
    cvOnTargetGovernorTop,
    &rotateAgitatorTop);

CycleStateCommandMapping<
    MultiShotCvCommandMapping::LaunchMode,
    MultiShotCvCommandMapping::NUM_SHOOTER_STATES,
    MultiShotCvCommandMapping>
    gOrVCtrlPressed(
        drivers(),
        RemoteMapState({Remote::Key::G, Remote::Key::CTRL}),
        MultiShotCvCommandMapping::SINGLE,
        &leftMouseCtrlPressed,
        &MultiShotCvCommandMapping::setShooterState,
        RemoteMapState({Remote::Key::V, Remote::Key::CTRL}));

// agitator mappings
ToggleCommandMapping vCtrlPressed(
    drivers(),
    {&setFireRateCommandFullAutoTop},
    RemoteMapState(RemoteMapState({Remote::Key::V, Remote::Key::CTRL})));

ToggleCommandMapping gCtrlPressed(
    drivers(),
    {&setFireRateCommand10RPSTop},
    RemoteMapState(RemoteMapState({Remote::Key::G, Remote::Key::CTRL})));

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
    &yawMotorBottom);

src::chassis::ChassisDriveCommand chassisDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::chassis::ChassisOrientDriveCommand chassisOrientDriveCommand(
    &chassisSubsystem,
    &drivers()->controlOperatorInterface);

src::chassis::ChassisBeybladeCommand chassisBeyBladeCommand(
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
    2.0f,
    0.0f,
    0.2f);

// chassis Mappings
ToggleCommandMapping beyBlade(
    drivers(),
    {&chassisBeyBladeCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::B})));

ToggleCommandMapping orientDrive(
    drivers(),
    {&chassisOrientDriveCommand},
    RemoteMapState(RemoteMapState({tap::communication::serial::Remote::Key::R})));

// imu commands
imu::SentryImuCalibrateCommand imuCalibrateCommand(
    drivers(),
    {{
        &sentryTurrets,
        &chassisFrameYawTurretControllerTop,
        &chassisFramePitchTurretControllerTop,
        &chassisFrameYawTurretControllerBottom,
        &chassisFramePitchTurretControllerBottom,
        false,
        false,
    }},
    &chassisSubsystem);

MatchRunningGovernor matchRunning(drivers()->refSerial);

GovernorLimitedCommand<1> chassisDefault(
    {&chassisSubsystem},
    chassisBeyBladeCommand,
    {&matchRunning});
GovernorLimitedCommand<1> cvManagerGameDefault({&sentryTurrets}, cvManagerCommand, {&matchRunning});
GovernorLimitedCommand<1> flywheelBottomDefault(
    {&flywheelBottom},
    flywheelRunCommandBottom,
    {&matchRunning});
GovernorLimitedCommand<1> flywheelTopDefault(
    {&flywheelTop},
    flywheelRunCommandTop,
    {&matchRunning});
GovernorLimitedCommand<1> agitatorBottomDefault(
    {&agitatorBottom},
    rotateAndUnjamAgitatorWithHeatAndCVLimitingBottom,
    {&matchRunning});
GovernorLimitedCommand<1> agitatorTopDefault(
    {&agitatorTop},
    rotateAndUnjamAgitatorWithHeatAndCVLimitingTop,
    {&matchRunning});

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());
void initializeSubsystems(Drivers *drivers)
{
    chassisSubsystem.initialize();
    agitatorTop.initialize();
    agitatorBottom.initialize();
    flywheelTop.initialize();
    flywheelBottom.initialize();
    sentryTurrets.initialize();
}

void registerSentrySubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassisSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitatorTop);
    drivers->commandScheduler.registerSubsystem(&agitatorBottom);
    drivers->commandScheduler.registerSubsystem(&flywheelTop);
    drivers->commandScheduler.registerSubsystem(&flywheelBottom);
    drivers->commandScheduler.registerSubsystem(&sentryTurrets);
}

void setDefaultSentryCommands(Drivers *drivers)
{
    chassisSubsystem.setDefaultCommand(&chassisDriveCommand);
    sentryTurrets.setDefaultCommand(&turretUserControlCommand);  // turretUserControlCommand );
    // flywheelBottom.setDefaultCommand(&flywheelBottomDefault);
    // flywheelTop.setDefaultCommand(&flywheelTopDefault);
    // agitatorBottom.setDefaultCommand(&agitatorBottomDefault);
    // agitatorTop.setDefaultCommand(&agitatorTopDefault);
}

void startSentryCommands(Drivers *drivers)
{
    drivers->bmi088.setMountingTransform(
        tap::algorithms::transforms::Transform(0, 0, 0, 0, modm::toRadian(-45), 0));
}

void registerSentryIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&leftMouseCtrlPressed);
    drivers->commandMapper.addMap(&leftMouseNotCtrlPressed);
    drivers->commandMapper.addMap(&rCtrlPressed);
    drivers->commandMapper.addMap(&rNotCtrlPressed);
    drivers->commandMapper.addMap(&fCtrlPressed);
    drivers->commandMapper.addMap(&fNotCtrlPressed);
    drivers->commandMapper.addMap(&gOrVCtrlPressed);
    drivers->commandMapper.addMap(&gOrVNotCtrlPressed);
    drivers->commandMapper.addMap(&beyBlade);
    drivers->commandMapper.addMap(&orientDrive);
    drivers->commandMapper.addMap(&xPressed);
    drivers->commandMapper.addMap(&xCtrlPressed);
}
}  // namespace sentry_control

namespace src::sentry
{
imu::ImuCalibrateCommandBase *getImuCalibrateCommand()
{
    return &sentry_control::imuCalibrateCommand;
}

void initSubsystemCommands(src::sentry::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &sentry_control::remoteSafeDisconnectFunction);
    sentry_control::initializeSubsystems(drivers);
    sentry_control::registerSentrySubsystems(drivers);
    sentry_control::setDefaultSentryCommands(drivers);
    sentry_control::startSentryCommands(drivers);
    sentry_control::registerSentryIoMappings(drivers);
}
}  // namespace src::sentry

#endif