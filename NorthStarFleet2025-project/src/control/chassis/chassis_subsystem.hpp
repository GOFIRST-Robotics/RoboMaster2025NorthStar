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

class ChassisSubsystem : public tap::control::Subsystem
{
public:
    enum class MotorId : uint8_t
    {
        LF = 0,
        LB,
        RF,
        RB,
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
        src::can::TurretMCBCanComm* turretMCBCanComm,
        tap::motor::DjiMotor* yawMotor);

    void initialize() override;

    mockable void setVelocityTurretDrive(float forward, float sideways, float rotational);

    mockable void setVelocityFieldDrive(float forward, float sideways, float rotational);

    mockable void setVelocityBeyBladeDrive(float forward, float sideways, float rotational);

    void driveBasedOnHeading(float forwards, float sideways, float rotational, float heading);

    float getChassisTurretOffset();

    void refresh() override;

    const char* getName() { return "Chassis"; }

    float getYaw();

private:
    inline float mpsToRpm(float mps)
    {
        static constexpr float GEAR_RATIO = 19.0f;
        static constexpr float WHEEL_DIAMETER_M = 0.0575f;  // 0.076
        static constexpr float WHEEL_CIRCUMFERANCE_M = M_PI * WHEEL_DIAMETER_M;
        static constexpr float SEC_PER_M = 60.0f;

        return (mps / WHEEL_CIRCUMFERANCE_M) * SEC_PER_M * GEAR_RATIO;
    }

    src::can::TurretMCBCanComm* turretMcbCanComm;

    tap::motor::DjiMotor* yawMotor;

    float beyBladeRotationSpeed = 0.0f;

    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> desiredOutput;

    std::array<Pid, static_cast<uint8_t>(MotorId::NUM_MOTORS)> pidControllers;

    inline float getTurretYaw();

protected:
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> motors;
};  // class ChassisSubsystem
}  // namespace src::chassis
