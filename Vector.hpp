#pragma once
#ifndef RAYTRACING_VECTOR_H
#define RAYTRACING_VECTOR_H

#include <iostream>
#include <cmath>
#include <algorithm>
#include "global.hpp"

class Vector3f {
public:
    float x, y, z;
    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float xx) : x(xx), y(xx), z(xx) {}
    Vector3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
    Vector3f operator * (const float &r) const { return Vector3f(x * r, y * r, z * r); }
    Vector3f operator / (const float &r) const { return Vector3f(x / r, y / r, z / r); }

    float norm() {return std::sqrt(x * x + y * y + z * z);}
    Vector3f normalized() {
        float n = std::sqrt(x * x + y * y + z * z);
        return Vector3f(x / n, y / n, z / n);
    }

    Vector3f operator * (const Vector3f &v) const { return Vector3f(x * v.x, y * v.y, z * v.z); }
    Vector3f operator - (const Vector3f &v) const { return Vector3f(x - v.x, y - v.y, z - v.z); }
    Vector3f operator + (const Vector3f &v) const { return Vector3f(x + v.x, y + v.y, z + v.z); }
    Vector3f operator - () const { return Vector3f(-x, -y, -z); }
    Vector3f& operator += (const Vector3f &v) { x += v.x, y += v.y, z += v.z; return *this; }
    friend Vector3f operator * (const float &r, const Vector3f &v)
    { return Vector3f(v.x * r, v.y * r, v.z * r); }
    friend std::ostream & operator << (std::ostream &os, const Vector3f &v)
    { return os << v.x << ", " << v.y << ", " << v.z; }
    // double       operator[](int index) const;
    // double&      operator[](int index);
    float operator[](int index) const;
    float& operator[](int index);


    static Vector3f Min(const Vector3f &p1, const Vector3f &p2) {
        return Vector3f(std::min(p1.x, p2.x), std::min(p1.y, p2.y),
                       std::min(p1.z, p2.z));
    }

    static Vector3f Max(const Vector3f &p1, const Vector3f &p2) {
        return Vector3f(std::max(p1.x, p2.x), std::max(p1.y, p2.y),
                       std::max(p1.z, p2.z));
    }
};

// inline double Vector3f::operator[](int index) const {
//     return (&x)[index];
// }
inline float Vector3f::operator[](int index) const {
    return (&x)[index];
}

inline float& Vector3f::operator[](int index) {
    return (&x)[index];
}



class Vector2f
{
public:
    Vector2f() : x(0), y(0) {}
    Vector2f(float xx) : x(xx), y(xx) {}
    Vector2f(float xx, float yy) : x(xx), y(yy) {}
    Vector2f operator * (const float &r) const { return Vector2f(x * r, y * r); }
    Vector2f operator + (const Vector2f &v) const { return Vector2f(x + v.x, y + v.y); }
    float x, y;
};

inline Vector3f lerp(const Vector3f &a, const Vector3f& b, const float &t)
{ return a * (1 - t) + b * t; }

inline Vector3f normalize(const Vector3f &v)
{
    float mag2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if (mag2 > 0) {
        float invMag = 1 / sqrtf(mag2);
        return Vector3f(v.x * invMag, v.y * invMag, v.z * invMag);
    }

    return v;
}

inline float dotProduct(const Vector3f &a, const Vector3f &b)
{ return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vector3f crossProduct(const Vector3f &a, const Vector3f &b)
{
    return Vector3f(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
    );
}

inline bool V3AEqualsToB(const Vector3f &a, const Vector3f &b)
{
    for (int i = 0; i < 3; ++i) {
        if(a[i]-b[i]>EPSILON)
        {
            return false;
        }
    }

    return true;
}

inline Vector3f getRandomPerpendicularVector(const Vector3f &vector) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    Vector3f randomVector;

    do {
        for (int i = 0; i < 3; ++i) {
            randomVector[i] = dist(gen); 
        }
    } while (V3AEqualsToB(randomVector, vector));

    Vector3f perpendicularVector(3);
    perpendicularVector[0] = vector[1] * randomVector[2] - vector[2] * randomVector[1];
    perpendicularVector[1] = vector[2] * randomVector[0] - vector[0] * randomVector[2];
    perpendicularVector[2] = vector[0] * randomVector[1] - vector[1] * randomVector[0];

    return perpendicularVector;
}

inline float V3Length(const Vector3f& a)
{
   return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
}

inline float V3Dist(const Vector3f& a, const Vector3f& b)
{
   return sqrtf(sqrt(a.x-b.x) + sqrt(a.y-b.y) + sqrt(a.z-b.z));
}

inline void V3RotateToFrame(Vector3f& a, const Vector3f& u, const Vector3f& v, const Vector3f& w)
{
   a = u * a.x + v * a.y + w * a.z;
}

inline bool V3IsZero(const Vector3f& a, float epsilon = EPSILON)
{
   return abs(a.x) < epsilon && abs(a.y) < epsilon && abs(a.z) < epsilon;
}

#endif //RAYTRACING_VECTOR_H
