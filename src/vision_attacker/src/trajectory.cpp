#include "vision_attacker/trajectory.hpp"
#include <cmath>

double solveTrajectory(double &flyTime, const double x, const double z, const double lagTime, const double muzzleSpeed, const double air_k)
{
    const double gravity = 9.78;
    double theta;
    double time;
    double aimZ;
    double realZ;
    double targetZ = z;

    aimZ = z;
    for (size_t i = 0; i < 20; i++)
    {
        theta = atan2(aimZ, x);
        time = (exp(air_k * x) - 1) / (air_k * muzzleSpeed * cos(theta));
        realZ = muzzleSpeed * sin(theta) * time - gravity * (time * time) / 2;
        aimZ = aimZ + (z - realZ);
        if (std::abs(realZ - z) < 0.001)
        {
            time += lagTime;
            flyTime = time;
            targetZ = aimZ;
            break;
        }
        else
        {
            continue;
        }
    }
    return atan2(targetZ, x);
}
