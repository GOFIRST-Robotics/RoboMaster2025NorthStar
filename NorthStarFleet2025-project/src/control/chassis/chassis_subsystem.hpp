#pragma once

#include <array>

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "communication/can/turret/turret_mcb_can_comm.hpp"
#include "control/chassis/algorithms/slew_rate_limiter.hpp"
#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/geometry/angle.hpp"


#define FIELD

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

namespace src::chassis
{
struct ChassisConfig
{
    tap::motor::MotorId leftFrontId;
    tap::motor::MotorId leftBackId;
    tap::motor::MotorId rightBackId;
    tap::motor::MotorId rightFrontId;
    tap::can::CanBus canBus;
    modm::Pid<float>::Parameter wheelVelocityPidConfig;
};

///
/// @brief This subsystem encapsulates four motors that control the chassis.
///
class ChassisSubsystem : public tap::control::Subsystem
{
public:
    /// @brief Motor ID to index into the velocityPid and motors object.
    enum class MotorId : uint8_t
    {
        LF = 0,  ///< Left front
        LB,      ///< Left back
        RF,      ///< Right front
        RB,      ///< Right back
        NUM_MOTORS,
    };

    using Pid = modm::Pid<float>;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::DjiMotor;
#endif

    static constexpr float MAX_WHEELSPEED_RPM = 7000;

    ChassisSubsystem(
        tap::Drivers* drivers,
        const ChassisConfig& config,
        src::can::TurretMCBCanComm* turretMCBCanComm);

    ///
    /// @brief Initializes the drive motors.
    ///
    void initialize() override;

    ///
    /// @brief Control the chassis using tank drive. Sets the wheel velocity of the four drive
    /// motors based on the input left/right desired velocity.
    ///
    /// @param left Desired chassis speed in m/s of the left side of the chassis. Positive speed is
    /// forward, negative is backwards.
    /// @param right Desired chassis speed in m/s of the right side of the chassis.
    ///
    mockable void setVelocityDrive(
        float forward,
        float sideways,
        float rotational,
        float turretRot);

    ///
    /// @brief Runs velocity PID controllers for the drive motors.
    ///
    void refresh() override;

    const char* getName() { return "Chassis"; }

    float getYaw();

private:
    inline float mpsToRpm(float mps)
    {
        static constexpr float GEAR_RATIO = 19.0f;
        static constexpr float WHEEL_DIAMETER_M = 0.076f;
        static constexpr float WHEEL_CIRCUMFERANCE_M = M_PI * WHEEL_DIAMETER_M;
        static constexpr float SEC_PER_M = 60.0f;

        return (mps / WHEEL_CIRCUMFERANCE_M) * SEC_PER_M * GEAR_RATIO;
    }

    src::can::TurretMCBCanComm* turretMcbCanComm;

    /// Desired wheel output for each motor
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> desiredOutput;

    /// PID controllers. Input desired wheel velocity, output desired motor current.
    std::array<Pid, static_cast<uint8_t>(MotorId::NUM_MOTORS)> pidControllers;

    std::array<src::chassis::algorithms::SlewRateLimiter, static_cast<uint8_t>(MotorId::NUM_MOTORS)>
        rateLimiters;

protected:
    /// Motors.
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> motors;
};  // class ChassisSubsystem
}  // namespace src::chassis
