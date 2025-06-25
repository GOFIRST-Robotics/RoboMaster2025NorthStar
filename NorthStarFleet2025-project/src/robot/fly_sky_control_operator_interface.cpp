//#define FLY_SKY
#ifdef FLY_SKY

#include "robot/fly_sky_control_operator_interface.hpp"

#include <random>

#include <tap/architecture/clock.hpp>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace src
{
namespace control
{
float ControlOperatorInterface::getTurretYawInput(uint8_t turretID)
{
    float input;
    switch (turretID)
    {
        case 0:
            input = -remote.getChannel(FlySky::Channel::RIGHT_HORIZONTAL) *
                    remote.getChannel(FlySky::Channel::WHEEL_B);
            if (!compareFloatClose(input, 0, .01))
            {
                return input;
            }
            return 0;
        case 1:
            input = -remote.getChannel(FlySky::Channel::LEFT_HORIZONTAL) *
                    remote.getChannel(FlySky::Channel::WHEEL_B);
            if (!compareFloatClose(input, 0, .01))
            {
                return input;
            }
            return 0;
        default:
            return 0;
    }
}

float ControlOperatorInterface::getTurretPitchInput(uint8_t turretID)
{
    float input;
    switch (turretID)
    {
        case 0:
            input = -remote.getChannel(FlySky::Channel::RIGHT_VERTICAL) *
                    remote.getChannel(FlySky::Channel::WHEEL_B);
            if (!compareFloatClose(input, 0, .01))
            {
                return input;
            }
            return 0;
        case 1:
            input = -remote.getChannel(FlySky::Channel::LEFT_VERTICAL) *
                    remote.getChannel(FlySky::Channel::WHEEL_B);
            if (!compareFloatClose(input, 0, .01))
            {
                return input;
            }
            return 0;
        default:
            return 0;
    }
}

template <typename T>
int getSign(T val)
{
    return (T(0) < val) - (val < T(0));
}

float ControlOperatorInterface::applyChassisSpeedScaling(float value)
{
    if (isSlowMode())
    {
        value *= 0.333;  // SPEED_REDUCTION_SCALAR;
    }
    return value;
}

// bool ControlOperatorInterface::isSlowMode() { return remote.keyPressed(Remote::Key::CTRL);
// }

/**
 * @param[out] ramp Ramp that should have acceleration applied to. The ramp is updated some
 * increment based on the passed in acceleration values. Ramp stores values in some units.
 * @param[in] maxAcceleration Positive acceleration value to apply to the ramp in units/time^2.
 * @param[in] maxDeceleration Negative acceleration value to apply to the ramp, in units/time^2.
 * @param[in] dt Change in time since this function was last called, in units of some time.
 */
static inline void applyAccelerationToRamp(
    tap::algorithms::Ramp &ramp,
    float maxAcceleration,
    float maxDeceleration,
    float dt)
{
    if (getSign(ramp.getTarget()) == getSign(ramp.getValue()) &&
        abs(ramp.getTarget()) > abs(ramp.getValue()))
    {
        // we are trying to speed up
        ramp.update(maxAcceleration * dt);
    }
    else
    {
        // we are trying to slow down
        ramp.update(maxDeceleration * dt);
    }
}
// STEP 2 (Tank Drive): Add getChassisTankLeftInput and getChassisTankRightInput function
// definitions

float ControlOperatorInterface::getDrivetrainHorizontalTranslation()
{
    float input;
    input = -remote.getChannel(FlySky::Channel::LEFT_HORIZONTAL) *
            remote.getChannel(FlySky::Channel::WHEEL_A);
    if (!compareFloatClose(input, 0, .05))
    {
        return input;
    }

    return 0.0f;
}

// float ControlOperatorInterface::getMecanumHorizontalTranslationKeyBoard()
// {
//     uint32_t updateCounter = remote.getUpdateCounter();
//     uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
//     uint32_t dt = currTime - prevChassisXInputCalledTime;
//     prevChassisXInputCalledTime = currTime;

//     if (prevUpdateCounterX != updateCounter)
//     {
//         chassisXInput.update(remote.getChannel(FlySky::Channel::LEFT_VERTICAL), currTime);
//         prevUpdateCounterX = updateCounter;
//     }

//     float keyInput = remote.keyPressed(Remote::Key::W) - remote.keyPressed(Remote::Key::S);

//     const float maxChassisSpeed = 7000;

//     float finalX = maxChassisSpeed *
//                    limitVal(chassisXInput.getInterpolatedValue(currTime) + keyInput,
//                    -1.0f, 1.0f);

//     chassisXInputRamp.setTarget(applyChassisSpeedScaling(finalX));

//     applyAccelerationToRamp(
//         chassisXInputRamp,
//         MAX_ACCELERATION_X,
//         MAX_DECELERATION_X,
//         static_cast<float>(dt) / 1E3F);
//     return chassisXInputRamp.getValue();
// }

float ControlOperatorInterface::getDrivetrainVerticalTranslation()
{
    float input;
    input = -remote.getChannel(FlySky::Channel::LEFT_VERTICAL) *
            remote.getChannel(FlySky::Channel::WHEEL_A);
    if (!compareFloatClose(input, 0, .05))
    {
        return input;
    }

    return 0.0f;
}

// float ControlOperatorInterface::getMecanumVerticalTranslationKeyBoard()
// {
//     uint32_t updateCounter = remote.getUpdateCounter();
//     uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
//     uint32_t dt = currTime - prevChassisYInputCalledTime;
//     prevChassisYInputCalledTime = currTime;

//     if (prevUpdateCounterY != updateCounter)
//     {
//         chassisYInput.update(-remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), currTime);
//         prevUpdateCounterY = updateCounter;
//     }

//     float keyInput = remote.keyPressed(Remote::Key::A) - remote.keyPressed(Remote::Key::D);

//     const float maxChassisSpeed = 7000;

//     float finalY = maxChassisSpeed *
//                    limitVal(chassisYInput.getInterpolatedValue(currTime) + keyInput,
//                    -1.0f, 1.0f);

//     chassisYInputRamp.setTarget(applyChassisSpeedScaling(finalY));

//     applyAccelerationToRamp(
//         chassisYInputRamp,
//         MAX_ACCELERATION_Y,
//         MAX_DECELERATION_Y,
//         static_cast<float>(dt) / 1E3F);

//     return chassisYInputRamp.getValue();
// }

// float ControlOperatorInterface::getDrivetrainRotation()
// {
//     if (remote.getChannel(FlySky::Channel::SWITCH_C == FlySky::SwitchState::UP))
//     {
//         return 0.2f;
//     }
//     else
//     {
//         return 0;
//     }
// }

int count = 250;
float beyBladeValue = 1;
float prevBeyBladeValue = 0.1f * sin(beyBladeValue) + 0.9f;
float speed = 0;

bool beyBlade = false;
bool isHeld = false;

float ControlOperatorInterface::getDrivetrainRotationalTranslation()
{
    if (remote.getChannel(FlySky::Channel::SWITCH_C) == SwitchState::UP)
    {
        return -0.3f;
    }
    else if (remote.getChannel(FlySky::Channel::SWITCH_C) == SwitchState::DOWN)
    {
        return 0.3f;
    }
    else
    {
        return 0.0f;
    }
}

// float ControlOperatorInterface::getMecanumRotationKeyBoard()
// {
//     uint32_t updateCounter = remote.getUpdateCounter();
//     uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
//     uint32_t dt = currTime - prevChassisRInputCalledTime;
//     prevChassisRInputCalledTime = currTime;

//     if (prevUpdateCounterR != updateCounter)
//     {
//         chassisRInput.update(-remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), currTime);
//         prevUpdateCounterR = updateCounter;
//     }

//     float keyInput = remote.keyPressed(Remote::Key::Q) - remote.keyPressed(Remote::Key::E);

//     const float maxChassisSpeed = 2500;

//     float finalR = maxChassisSpeed *
//                    limitVal(chassisRInput.getInterpolatedValue(currTime) + keyInput,
//                    -1.0f, 1.0f);

//     chassisRInputRamp.setTarget(finalR);

//     applyAccelerationToRamp(
//         chassisRInputRamp,
//         MAX_ACCELERATION_R,
//         MAX_DECELERATION_R,
//         static_cast<float>(dt) / 1E3);

//     return chassisRInputRamp.getValue();
// }

// bool ControlOperatorInterface::isRightSwitchUp()
// {
//     return (remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP);
// }

// bool ControlOperatorInterface::isGKeyPressed() { return
// (remote.keyPressed(Remote::Key::G)); }

// void ControlOperatorInterface::checkToggleBeyBlade()
// {
//     if (remote.keyPressed(Remote::Key::B))
//     {
//         if (!isHeld)
//         {
//             beyBlade = !beyBlade;
//             isHeld = true;
//         }
//     }
//     else
//     {
//         isHeld = false;
//     }
// }

}  // namespace control

}  // namespace src

#endif  // FLY_SKY