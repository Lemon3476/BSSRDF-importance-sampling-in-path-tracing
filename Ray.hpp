#define RAY_GENERIC 0xFF
#define RAY_SUBSURFACE 0x10

#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H
#include "Vector.hpp"
struct Ray{
    //Destination = origin + t*direction
    Vector3f origin;
    Vector3f direction, direction_inv;
    double t;//transportation time,
    double t_min, t_max;
    unsigned short type;

    Ray(const Vector3f& ori, const Vector3f& dir, const double _t = 0.0, const unsigned short _type = RAY_GENERIC)
    : origin(ori), direction(dir),t(_t), type(_type) {
        direction_inv = Vector3f(1./direction.x, 1./direction.y, 1./direction.z);
        t_min = 0.0;
        t_max = std::numeric_limits<double>::max();

    }

    Vector3f operator()(double t) const{return origin+direction*t;}

    float getLength(){
        return V3Length(direction*t);
    }

    friend std::ostream &operator<<(std::ostream& os, const Ray& r){
        os<<"[origin:="<<r.origin<<", direction="<<r.direction<<", time="<< r.t<<"]\n";
        return os;
    }

    
};
#endif //RAYTRACING_RAY_H
