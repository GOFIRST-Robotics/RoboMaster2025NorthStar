#ifdef TURRET

#include "tap/drivers.hpp"
#include "drivers_singleton.hpp"
#include "../../robot-type/robot_type.hpp"

#include "tap/util_macros.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "robot/turret/turret_drivers.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_drive_command.hpp"

using namespace src::gyro;

driversFunc drivers = DoNotUse_getDrivers;

namespace turret_control
{
    
    
void initializeSubsystems(Drivers *drivers)
{
    
}

void registerSoldierSubsystems(Drivers *drivers)
{
   
}

void setDefaultSoldierCommands(Drivers *drivers)
{
    
}

void startSoldierCommands(Drivers *drivers) {}

void registerSoldierIoMappings(Drivers *drivers)
{
   
}
}  // namespace turret_control

namespace src::gyro
{
void initSubsystemCommands(src::gyro::Drivers *drivers)
{
    turret_control::initializeSubsystems(drivers);
    turret_control::registerSoldierSubsystems(drivers);
    turret_control::setDefaultSoldierCommands(drivers);
    turret_control::startSoldierCommands(drivers);
    turret_control::registerSoldierIoMappings(drivers);
}
} //namespace src::gyro

#endif