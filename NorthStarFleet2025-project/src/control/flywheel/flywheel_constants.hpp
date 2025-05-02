#ifndef FLYWHEEL_CONSTANTS_HPP_
#define FLYWHEEL_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#ifdef TARGET_STANDARD
#include "robot/standard/standard_flywheel_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_flywheel_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_flywheel_constants.hpp"
#elif TURRET
#include "robot/standard/standard_flywheel_constants.hpp"
#endif

namespace src::flywheel
{
}  // namespace src::flywheel

#endif  // FLYWHEEL_CONSTANTS_HPP_
