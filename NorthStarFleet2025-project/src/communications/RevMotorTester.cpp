#include "RevMotorTester.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"

//#define THING

#include <cmath>

using tap::algorithms::limitVal;

namespace Communications::Rev
{
// STEP 1 (Tank Drive): create constructor
    RevMotorTester::RevMotorTester(tap::Drivers& drivers)
        : Subsystem(&drivers),
        motor1(&drivers, tap::motor::REVMotorId::REV_MOTOR1, tap::can::CanBus::CAN_BUS1, false, "REV Motor 1")
    {
    }
   
// STEP 2 (Tank Drive): initialize function
    void RevMotorTester::initialize() {
        motor1.initialize();
        motor1.setControlValue(0.1f);
    }


    void RevMotorTester::refresh() {
        motor1.setControlValue(0.1f);
    }
// STEP
}  // namespace control::chassis
