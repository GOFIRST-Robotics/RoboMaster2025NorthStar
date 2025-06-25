/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef REMOTE_SWITCH_GOVERNOR_HPP_
#define REMOTE_SWITCH_GOVERNOR_HPP_

#include <cstdint>

#include "tap/architecture/timeout.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

#include "robot/control_operator_interface.hpp"

namespace src::control::governor
{
/**
 * A governor that tracks game state. Is ready if the game state is unknown (offline)
 * or if the game is running
 */
class RemoteSwitchGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    RemoteSwitchGovernor(
        src::control::ControlOperatorInterface* controlOperatorInterface,
        short pos)
        : controlOperatorInterface(controlOperatorInterface),
          pos(pos)
    {
    }

    bool isReady() override
    {
        if (pos == 0)
        {
            return controlOperatorInterface->isRightSwitchUp();
        }
        if (pos == 1)
        {
            return controlOperatorInterface->isRightSwitchDown();
        }
        return false;
    }

    bool isFinished() override { return false; }

private:
    src::control::ControlOperatorInterface* controlOperatorInterface;
    short pos;
};

}  // namespace src::control::governor

#endif  // REMOTE_SWITCH_GOVERNOR_HPP_
