//#define FLY_SKY
#ifdef FLY_SKY
#include "robot/fly_sky_control_operator_interface.hpp"
#else
/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_

// mm tasty imports
#include <tap/algorithms/linear_interpolation_predictor.hpp>
#include <tap/algorithms/ramp.hpp>

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

namespace src
{
namespace control
{
/**
 * A class for interfacing with the remote IO inside of Commands. While the
 * CommandMapper handles the scheduling of Commands, this class is used
 * inside of Commands to interact with the remote. Filtering and normalization
 * is done in this class.
 */
class ControlOperatorInterface
{
public:
    static constexpr int16_t USER_MOUSE_YAW_MAX = 1000;
    static constexpr int16_t USER_MOUSE_PITCH_MAX = 1000;
    static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / USER_MOUSE_YAW_MAX);
    static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / USER_MOUSE_PITCH_MAX);
    static constexpr float SPEED_REDUCTION_SCALAR = (1.0f / 3.0f);
    static constexpr float USER_STICK_SENTRY_DRIVE_SCALAR = 5000.0f;

    ControlOperatorInterface(tap::Drivers *drivers) : remote(drivers->remote) {}

    /**
     * @return the value used for turret yaw rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     */
    mockable float getTurretYawInput(uint8_t turretID);

    /**
     * @returns the value used for turret pitch rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     */
    mockable float getTurretPitchInput(uint8_t turretID);

    static constexpr float MAX_ACCELERATION_X = 10'000.0f;
    static constexpr float MAX_DECELERATION_X = 20'000.0f;

    static constexpr float MAX_ACCELERATION_Y = 9'000.0f;
    static constexpr float MAX_DECELERATION_Y = 20'000.0f;

    static constexpr float MAX_ACCELERATION_R = 40'000.0f;
    static constexpr float MAX_DECELERATION_R = 50'000.0f;

    // STEP 1 (Tank Drive): Add getChassisTankLeftInput and getChassisTankRightInput function
    // declarations
    float getDrivetrainHorizontalTranslation();

    float getMecanumHorizontalTranslationKeyBoard();

    float getDrivetrainVerticalTranslation();

    float getMecanumVerticalTranslationKeyBoard();

    float getDrivetrainRotation();

    float getDrivetrainRotationalTranslation();

    float getMecanumRotationKeyBoard();

    /**
     * @returns whether or not the key to disable diagonal drive is pressed.
     * The key is shared with the speed scaling key.
     */
    bool isSlowMode();

    bool isRightSwitchUp();

    bool isGKeyPressed();

    void checkToggleBeyBlade();

    /**
     * Scales `value` when ctrl/shift are pressed and returns the scaled value.
     */
    float applyChassisSpeedScaling(float value);

private:
    tap::communication::serial::Remote &remote;

    uint32_t prevUpdateCounterX = 0;
    uint32_t prevUpdateCounterY = 0;
    uint32_t prevUpdateCounterR = 0;

    tap::algorithms::LinearInterpolationPredictor chassisXInput;
    tap::algorithms::LinearInterpolationPredictor chassisYInput;
    tap::algorithms::LinearInterpolationPredictor chassisRInput;

    tap::algorithms::Ramp chassisXInputRamp;
    tap::algorithms::Ramp chassisYInputRamp;
    tap::algorithms::Ramp chassisRInputRamp;

    uint32_t prevChassisXInputCalledTime = 0;
    uint32_t prevChassisYInputCalledTime = 0;
    uint32_t prevChassisRInputCalledTime = 0;
    /**
     * Scales `value` when ctrl/shift are pressed and returns the scaled value.
     */
};
}  // namespace control

}  // namespace src

#endif  // CONTROL_OPERATOR_INTERFACE_HPP_

#endif  // FLY_SKY