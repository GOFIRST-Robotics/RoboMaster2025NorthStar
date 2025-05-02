#include "RevMotorTester.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"

//#define THING

#include <cmath>

using tap::algorithms::limitVal;

namespace Communications::Rev
{
// STEP 1 (Tank Drive): create constructor
RevMotorTester::RevMotorTester(tap::Drivers* drivers)
    : Subsystem(drivers),
      motor1(
          drivers,
          tap::motor::REVMotorId::REV_MOTOR1,
          tap::can::CanBus::CAN_BUS2,
          false,
          "REV Motor 1"),
      motor2(
          drivers,
          tap::motor::REVMotorId::REV_MOTOR2,
          tap::can::CanBus::CAN_BUS2,
          true,
          "REV Motor 2"),
      motor3(
          drivers,
          tap::motor::REVMotorId::REV_MOTOR5,
          tap::can::CanBus::CAN_BUS2,
          false,
          "REV Motor 3")
{
}

// STEP 2 (Tank Drive): initialize function
void RevMotorTester::initialize()
{
    motor1.initialize();
    motor2.initialize();
    motor3.initialize();
    // motor1.setTargetVoltage(0.1f); //causing error bacause setTargetVoltage dosen't exist
    motor1.setControlValue(0.25f);
    motor2.setControlValue(0.25f);
    motor3.setControlValue(0.25f);
}

void RevMotorTester::refresh()
{
    // motor1.setTargetVoltage(0.1f);
}
// STEP
}  // namespace Communications::Rev
