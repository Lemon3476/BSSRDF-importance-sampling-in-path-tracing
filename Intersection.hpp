#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H

#include "Vector.hpp"
#include "RGB.hpp"
#include "Material.hpp"
class Object;
class Sphere;
class Triangle;
class MeshTriangle;

struct Intersection;

struct Intersection
{
    Intersection(){
        happened=false;
        shadeSSS=false;
        coords=Vector3f();
        tcoords=Vector3f();
        normal=Vector3f();
        sampleU=Vector3f();
        sampleV=Vector3f();
        emit=Vector3f();
        sampleDisp=Vector3f();
        color=RGB_BLACK;
        distance= std::numeric_limits<double>::max();
        sampleRadius=0.0f;
        probeDepth=0;
        obj=nullptr;
        mesh=nullptr;
        m=nullptr;
        ray=nullptr;

        for (unsigned int i = 0; i < kMaxProbeDepth; i++) {
            sampleIsects[i] = nullptr;
        }
    }

    bool happened;
    bool shadeSSS;
    Vector3f coords;
    Vector3f tcoords;
    Vector3f normal;
    Vector3f sampleU;
    Vector3f sampleV;
    Vector3f emit;
    Vector3f sampleDisp;
    RGB color;
    double distance;
    double sampleRadius;
    unsigned int probeDepth;
    Object* obj;
    MeshTriangle* mesh;
    Material* m;
    Ray* ray;

    Intersection* sampleIsects[kMaxProbeDepth];
};

#endif //RAYTRACING_INTERSECTION_H
