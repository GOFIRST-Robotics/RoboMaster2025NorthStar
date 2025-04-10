#ifndef STANDARD_DRIVERS_HPP_
#define STANDARD_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/imu_terminal_serial_handler_mock.hpp"

// #include "src/mock/turret_mcb_can_comm_mock.hpp"

#else
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

#include "../../src/communication/can/turret/turret_mcb_can_comm.hpp"
#include "robot/control_operator_interface.hpp"

#endif

namespace src::standard
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : tap::Drivers(),
          controlOperatorInterface(this),
          turretMCBCanCommBus1(this, tap::can::CanBus::CAN_BUS2)
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanCommBus1;
#else
public:
    control::ControlOperatorInterface controlOperatorInterface;
    can::TurretMCBCanComm turretMCBCanCommBus1;
#endif
};  // class src::StandardDrivers
}  // namespace src::standard

#endif  // STANDARD_DRIVERS_HPP_
