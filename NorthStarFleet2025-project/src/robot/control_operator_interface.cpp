
/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "robot/control_operator_interface.hpp"

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
            input = -remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) +
                    static_cast<float>(limitVal<int16_t>(
                        -remote.getMouseX(),
                        -USER_MOUSE_YAW_MAX,
                        USER_MOUSE_YAW_MAX)) *
                        USER_MOUSE_YAW_SCALAR * !remote.keyPressed(Remote::Key::CTRL);
            if (!compareFloatClose(input, 0, .01))
            {
                return input;
            }
            return 0;
        case 1:
            input = -remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) +
                    static_cast<float>(limitVal<int16_t>(
                        -remote.getMouseX(),
                        -USER_MOUSE_YAW_MAX,
                        USER_MOUSE_YAW_MAX)) *
                        USER_MOUSE_YAW_SCALAR * remote.keyPressed(Remote::Key::CTRL);
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
            input = -remote.getChannel(Remote::Channel::RIGHT_VERTICAL) +
                    static_cast<float>(limitVal<int16_t>(
                        -remote.getMouseY(),
                        -USER_MOUSE_YAW_MAX,
                        USER_MOUSE_YAW_MAX)) *
                        USER_MOUSE_YAW_SCALAR * !remote.keyPressed(Remote::Key::CTRL);
            if (!compareFloatClose(input, 0, .01))
            {
                return input;
            }
            return 0;
        case 1:
            input = -remote.getChannel(Remote::Channel::LEFT_VERTICAL) +
                    static_cast<float>(limitVal<int16_t>(
                        -remote.getMouseY(),
                        -USER_MOUSE_YAW_MAX,
                        USER_MOUSE_YAW_MAX)) *
                        USER_MOUSE_YAW_SCALAR * remote.keyPressed(Remote::Key::CTRL);
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

bool ControlOperatorInterface::isSlowMode() { return remote.keyPressed(Remote::Key::CTRL); }

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
    if (remote.keyPressed(Remote::Key::A) && !remote.keyPressed(Remote::Key::SHIFT))
    {
        return -0.3f;
    }
    else if (remote.keyPressed(Remote::Key::A) && remote.keyPressed(Remote::Key::SHIFT))
    {
        return -0.6f;
    }
    else if (remote.keyPressed(Remote::Key::D) && !remote.keyPressed(Remote::Key::SHIFT))
    {
        return 0.3f;
    }
    else if (remote.keyPressed(Remote::Key::D) && remote.keyPressed(Remote::Key::SHIFT))
    {
        return 0.6f;
    }
    else
    {
        return 0.0f;
    }
}

float ControlOperatorInterface::getMecanumHorizontalTranslationKeyBoard()
{
    uint32_t updateCounter = remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisXInputCalledTime;
    prevChassisXInputCalledTime = currTime;

    if (prevUpdateCounterX != updateCounter)
    {
        chassisXInput.update(remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterX = updateCounter;
    }

    float keyInput = remote.keyPressed(Remote::Key::W) - remote.keyPressed(Remote::Key::S);

    const float maxChassisSpeed = 7000;

    float finalX = maxChassisSpeed *
                   limitVal(chassisXInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisXInputRamp.setTarget(applyChassisSpeedScaling(finalX));

    applyAccelerationToRamp(
        chassisXInputRamp,
        MAX_ACCELERATION_X,
        MAX_DECELERATION_X,
        static_cast<float>(dt) / 1E3F);
    return chassisXInputRamp.getValue();
}

float ControlOperatorInterface::getDrivetrainVerticalTranslation()
{
    if (remote.keyPressed(Remote::Key::W) && !remote.keyPressed(Remote::Key::SHIFT))
    {
        return 0.3f;
    }
    else if (remote.keyPressed(Remote::Key::W) && remote.keyPressed(Remote::Key::SHIFT))
    {
        return 0.6f;
    }
    else if (remote.keyPressed(Remote::Key::S) && !remote.keyPressed(Remote::Key::SHIFT))
    {
        return -0.3f;
    }
    else if (remote.keyPressed(Remote::Key::S) && remote.keyPressed(Remote::Key::SHIFT))
    {
        return -0.6f;
    }
    else
    {
        return 0.0f;
    }
}

float ControlOperatorInterface::getMecanumVerticalTranslationKeyBoard()
{
    uint32_t updateCounter = remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisYInputCalledTime;
    prevChassisYInputCalledTime = currTime;

    if (prevUpdateCounterY != updateCounter)
    {
        chassisYInput.update(-remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), currTime);
        prevUpdateCounterY = updateCounter;
    }

    float keyInput = remote.keyPressed(Remote::Key::A) - remote.keyPressed(Remote::Key::D);

    const float maxChassisSpeed = 7000;

    float finalY = maxChassisSpeed *
                   limitVal(chassisYInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisYInputRamp.setTarget(applyChassisSpeedScaling(finalY));

    applyAccelerationToRamp(
        chassisYInputRamp,
        MAX_ACCELERATION_Y,
        MAX_DECELERATION_Y,
        static_cast<float>(dt) / 1E3F);

    return chassisYInputRamp.getValue();
}

float ControlOperatorInterface::getDrivetrainRotation()
{
    if (remote.keyPressed(Remote::Key::CTRL))
    {
        return 0.2f;
    }
    else
    {
        return 0;
    }
}

int count = 250;
float beyBladeValue = 1;
float prevBeyBladeValue = 0.1f * sin(beyBladeValue) + 0.9f;
float speed = 0;

bool beyBlade = false;
bool isHeld = false;

float ControlOperatorInterface::getDrivetrainRotationalTranslation()
{
    if (remote.keyPressed(Remote::Key::Q) && !remote.keyPressed(Remote::Key::SHIFT))
    {
        return -0.3f;
    }
    else if (remote.keyPressed(Remote::Key::Q) && remote.keyPressed(Remote::Key::SHIFT))
    {
        return -0.6f;
    }
    else if (remote.keyPressed(Remote::Key::E) && !remote.keyPressed(Remote::Key::SHIFT))
    {
        return 0.3f;
    }
    else if (remote.keyPressed(Remote::Key::E) && remote.keyPressed(Remote::Key::SHIFT))
    {
        return 0.6f;
    }
    else
    {
        return 0.0f;
    }
}

float ControlOperatorInterface::getMecanumRotationKeyBoard()
{
    uint32_t updateCounter = remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevChassisRInputCalledTime;
    prevChassisRInputCalledTime = currTime;

    if (prevUpdateCounterR != updateCounter)
    {
        chassisRInput.update(-remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), currTime);
        prevUpdateCounterR = updateCounter;
    }

    float keyInput = remote.keyPressed(Remote::Key::Q) - remote.keyPressed(Remote::Key::E);

    const float maxChassisSpeed = 2500;

    float finalR = maxChassisSpeed *
                   limitVal(chassisRInput.getInterpolatedValue(currTime) + keyInput, -1.0f, 1.0f);

    chassisRInputRamp.setTarget(finalR);

    applyAccelerationToRamp(
        chassisRInputRamp,
        MAX_ACCELERATION_R,
        MAX_DECELERATION_R,
        static_cast<float>(dt) / 1E3);

    return chassisRInputRamp.getValue();
}

bool ControlOperatorInterface::isRightSwitchUp()
{
    return (remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP);
}

bool ControlOperatorInterface::isGKeyPressed() { return (remote.keyPressed(Remote::Key::G)); }

void ControlOperatorInterface::checkToggleBeyBlade()
{
    if (remote.keyPressed(Remote::Key::B))
    {
        if (!isHeld)
        {
            beyBlade = !beyBlade;
            isHeld = true;
        }
    }
    else
    {
        isHeld = false;
    }
}

}  // namespace control

}  // namespace src
