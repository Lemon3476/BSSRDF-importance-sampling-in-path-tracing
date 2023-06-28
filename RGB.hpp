#pragma once
#include "Vector.hpp"
#include "global.hpp"

typedef Vector3f RGB;
#define RGB_BLACK RGB(0.0f)
#define RGB_WHITE RGB(1.e20f)

inline bool ColorIsZero(const RGB& c, float epsilon = EPSILON)
{
   return abs(c.x) < epsilon && abs(c.y) < epsilon && abs(c.z) < epsilon;
}