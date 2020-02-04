#ifndef UTILS_H
#define UTILS_H

#include <cmath>

class Utils
{
public:
    Utils();

    static float DegToRad(float x)
    {
        return x / 180.f * M_PI;
    }

    static float RadToDeg(float x)
    {
        return x / M_PI * 180.f;
    }
};

#endif // UTILS_H
