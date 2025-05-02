namespace src::chassis::algorithms
{
class SlewRateLimiter
{
public:
    SlewRateLimiter(float rateLimit, float maxError)
        : rateLimit(rateLimit / 500),
          maxError(maxError / 500)
    {
    }
    float runLimiter(float desiredVelocity, float currentVelocity);

private:
    float rateLimit;  // Motor rotations per minute per second
    float maxError;   // Rotations per minute
};

}  // namespace src::chassis::algorithms
