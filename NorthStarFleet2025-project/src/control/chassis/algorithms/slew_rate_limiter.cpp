#include "slew_rate_limiter.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace src::chassis::algorithms
{
float SlewRateLimiter::runLimiter(float desiredVelocity, float currentVelocity)
{
    float delta = desiredVelocity - currentVelocity;

    // If stationary and should move, apply a boost
    float startBoost = 1000.0f;
    if (std::abs(currentVelocity) < startBoost && std::abs(desiredVelocity) > startBoost)
    {
        return (desiredVelocity > 0) ? startBoost : -startBoost;
    }

    float step = tap::algorithms::limitVal(delta, -rateLimit, rateLimit);
    return currentVelocity + step;
}

}  // namespace src::chassis::algorithms