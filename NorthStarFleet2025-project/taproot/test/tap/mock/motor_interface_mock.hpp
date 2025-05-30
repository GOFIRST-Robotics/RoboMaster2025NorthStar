/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_MOTOR_INTERFACE_MOCK_HPP_
#define TAPROOT_MOTOR_INTERFACE_MOCK_HPP_

#include <cstdint>

#include <gmock/gmock.h>

#include "tap/motor/motor_interface.hpp"

#include "encoder_interface_mock.hpp"

namespace tap::mock
{
class MotorInterfaceMock : public tap::motor::MotorInterface
{
public:
    MotorInterfaceMock();
    virtual ~MotorInterfaceMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, setDesiredOutput, (int32_t), (override));
    MOCK_METHOD(bool, isMotorOnline, (), (const override));
    MOCK_METHOD(int16_t, getOutputDesired, (), (const override));
    MOCK_METHOD(int8_t, getTemperature, (), (const override));
    MOCK_METHOD(int16_t, getTorque, (), (const override));

    EncoderInterfaceMock* getEncoder() const override
    {
        return const_cast<testing::NiceMock<tap::mock::EncoderInterfaceMock>*>(&encoder);
    }

private:
    testing::NiceMock<tap::mock::EncoderInterfaceMock> encoder;
};

}  // namespace tap::mock

#endif  //  TAPROOT_MOTOR_INTERFACE_MOCK_HPP_
