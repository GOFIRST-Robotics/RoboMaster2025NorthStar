#ifdef TARGET_HERO

#ifndef HERO_FLYWHEEL_SUBSYSTEM
#define HERO_FLYWHEEL_SUBSYSTEM

#include <modm/container/pair.hpp>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/flywheel/flywheel_constants.hpp"
#include "modm/math/filter/pid.hpp"

namespace src::control::flywheel
{
class HeroFlywheelSubsystem : public tap::control::Subsystem
{
public:
    HeroFlywheelSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::motor::MotorId downMotorId,
        tap::can::CanBus canBus);

    void initialize() override;

    mockable void setDesiredSpin(u_int16_t spin);

    mockable float getDesiredSpin() const { return desiredSpin; }

    mockable void setDesiredLaunchSpeed(float speed);

    mockable float getDesiredLaunchSpeedLeft() const { return desiredLaunchSpeedLeft; }
    mockable float getDesiredLaunchSpeedRight() const { return desiredLaunchSpeedRight; }
    mockable float getDesiredLaunchSpeedDown() const { return desiredLaunchSpeedDown; }

    mockable float getDesiredFlywheelSpeedLeft() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedLeft);
    }
    mockable float getDesiredFlywheelSpeedRight() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedRight);
    }
    mockable float getDesiredFlywheelSpeedDown() const
    {
        return launchSpeedToFlywheelRpm(desiredLaunchSpeedDown);
    }

    float getCurrentLeftFlywheelMotorRPM() const { return leftWheel.getShaftRPM(); }

    float getCurrentRightFlywheelMotorRPM() const { return rightWheel.getShaftRPM(); }

    float getCurrentDownFlywheelMotorRPM() const { return downWheel.getShaftRPM(); }

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        leftWheel.setDesiredOutput(0);  // TODO CHANGE
        rightWheel.setDesiredOutput(0);
        downWheel.setDesiredOutput(0);
    }

    const char *getName() const override { return "Flywheels"; }

protected:
    static constexpr float MAX_DESIRED_LAUNCH_SPEED = 10000;  // TODO

    tap::Drivers *drivers;

private:
    modm::Pid<float> velocityPidLeftWheel;
    modm::Pid<float> velocityPidRightWheel;
    modm::Pid<float> velocityPidDownWheel;

    float desiredLaunchSpeedLeft;
    float desiredLaunchSpeedRight;
    float desiredLaunchSpeedDown;

    Spin desiredSpin = SPIN_100;
    u_int16_t desiredSpinValue = 100;  // percent of spin

    uint32_t prevTime = 0;

    tap::algorithms::Ramp desiredRpmRampLeft;
    tap::algorithms::Ramp desiredRpmRampRight;
    tap::algorithms::Ramp desiredRpmRampDown;

    tap::motor::DjiMotor leftWheel;
    tap::motor::DjiMotor rightWheel;
    tap::motor::DjiMotor downWheel;

    float launchSpeedToFlywheelRpm(float launchSpeed) const;

    std::array<std::array<modm::Pair<float, float>, 5>, SPIN_COUNT> spinToRPMMap;
};

}  // namespace src::control::flywheel

#endif  // HERO_FLYWHEEL_SUBSYSTEM

#endif  // TARGET_HERO