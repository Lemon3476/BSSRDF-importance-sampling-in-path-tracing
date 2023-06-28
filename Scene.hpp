#pragma once

#include <vector>
#include <cassert>
#include "Vector.hpp"
#include "RGB.hpp"
#include "Object.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "AreaLight.hpp"
#include "BVH.hpp"
#include "Ray.hpp"

class Scene
{
public:
    // setting up options
    int width = 1280;
    int height = 960;
    double fov = 40;
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    int maxDepth = 1;
    float RussianRoulette = 0.8;

    Scene(int w, int h) : width(w), height(h)
    {}

    void Add(Object *object) { objects.push_back(object); }
    void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

    const std::vector<Object*>& get_objects() const { return objects; }
    const std::vector<std::unique_ptr<Light> >&  get_lights() const { return lights; }
    Intersection intersect(const Ray& ray) const;
    BVHAccel *bvh;
    void buildBVH();
    Intersection castRay(const Ray &ray, Vector3f eye_pos) const;
    void sampleLight(Intersection &pos, float &pdf) const;
    Intersection shade(Intersection pos, Vector3f wo) const;
    bool trace(const Ray &ray, const std::vector<Object*> &objects, float &tNear, uint32_t &index, Object **hitObject);
    std::tuple<Vector3f, Vector3f> HandleAreaLight(const AreaLight &light, const Vector3f &hitPoint, const Vector3f &N,
                                                   const Vector3f &shadowPointOrig,
                                                   const std::vector<Object *> &objects, uint32_t &index,
                                                   const Vector3f &dir, float specularExponent);




    // creating the scene (adding objects and lights)
    std::vector<Object* > objects;
    std::vector<std::unique_ptr<Light> > lights;

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }



    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }


    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
    }
};

void probeRayExtension(Ray& probeRay, Bounds3& box);





namespace rls
{

class   NDProfile
{
public:
    static int     selectDistLobe(float &x)
    {
        if (x < 0.3333f) {
            x = LINEARSTEP(0.0f, 0.3333f, x);
            return 0;
        } else if (x > 0.6666f) {
            x = LINEARSTEP(0.6666f, 1.0f, x);
            return 2;
        }

        x = LINEARSTEP(0.3333f, 0.6666f, x);
        return 1;
    }

    inline float   maxRadius() const
    {
        return mMaxRadius;
    }

    void    setDistance(const Vector3f &dist);

    float   getRadius(float rx) const;

    float   getPdf(float r) const;

    RGB   evalProfile(float r) const;

public:
    Vector3f    mDistance;
    Vector3f    mC1, mC2;
    float       mMaxRadius;
};

//! Sub-surface scattering based on normalized diffusion.
class   SssSampler
{
public:
    // static const int kMaxProbeDepth = 12;


public:
    SssSampler(const Vector3f &dist)
    {
        mProfile.setDistance(dist);
    }


    bool shadeSSS(Intersection *isect)
    {
        if(V3AEqualsToB(isect->color, RGB_BLACK))
        {
            return false;
        }
        RGB result = RGB_BLACK;
        // Combine the shading results from each proble samples.
        for (auto i = 0u; i < isect->probeDepth; i++) {
            auto sample = isect->sampleIsects[i];
            auto Rd = mProfile.evalProfile(V3Length(sample->sampleDisp));
            auto pdf = mProfile.getPdf(sample->sampleRadius);
            auto NV = abs(dotProduct(sample->ray->direction, sample->normal)); 

            auto weight = Rd/pdf/NV;
// std::cout<<"color of sample point: "<<sample->color<<std::endl;
// std::cout<<"probe ray direction: "<<sample->ray->direction<<std::endl;
// std::cout<<"dot product between U and normal: "<<dotProduct(isect->sampleU, sample->normal)<<std::endl;
// std::cout<<"dot product between V and normal: "<<dotProduct(isect->sampleV, sample->normal)<<std::endl;
// std::cout<<"dot product between N and normal: "<<dotProduct(isect->normal, sample->normal)<<std::endl;

// std::cout<<"weight: "<<weight<<std::endl;
            result.x += weight.x*sample->color.x;
            result.y += weight.y*sample->color.y;
            result.z += weight.z*sample->color.z;

        }
        isect->color += result/isect->probeDepth;
        return true;
    }

public:
    void    alignDir(const Vector3f &refDir, Vector3f &dir)
    {
        if (dotProduct(refDir, dir) < 0.0f) {
            dir = dir * -1.0f;
        }
    }

    //! Return the probe ray to find intersection near x_0.
    float   getProbeRay(float rx, float ry, Intersection *isect, Ray &ray) const
    {
        ray.type = RAY_SUBSURFACE;

        auto idx = 0;

        if (rx < 0.5f) {
            idx = 0;
            rx = LINEARSTEP(0.0f, 0.5f, rx);
        } else if (rx < 0.75f) {
            idx = 2;
            rx = LINEARSTEP(0.5f, 0.75f, rx);
        } else {
            idx = 3;
            rx = LINEARSTEP(0.75f, 1.0f, rx);
        }

        float r = mProfile.getRadius(rx);
        assert(std::isfinite(r) && r >= 0.0f);

        float rmax = mProfile.maxRadius();
        float phi = 2 * M_PI * ry;

        Vector3f offset;
        offset.x = cosf(phi) * r;
        offset.z = sinf(phi) * r;
        offset.y = sqrt(rmax * rmax - r * r);
// std::cout<<"offset of x y and z: "<<offset.x<<" "<<offset.y<<" "<<offset.z<<std::endl;

        Vector3f U = getRandomPerpendicularVector(isect->normal);
        isect->sampleU = normalize(U);
        isect->sampleV = normalize(crossProduct(isect->normal, U));

        if ((idx & 0x03) < 2) {
            // Use normal as probe direction
            ray.direction = -isect->normal;
        } else if ((idx & 0x03) == 2) {
            ray.direction = (idx & 0x04) > 0 ? -isect->sampleU : isect->sampleU;
        } else {
            ray.direction = (idx & 0x04) > 0 ? -isect->sampleV : isect->sampleV;
        }

        ray.origin = isect->coords + offset;
// std::cout<<"origin of probe ray from the function: "<<ray.origin<<std::endl;
// std::cout<<"direction of probe ray from the function: "<<ray.direction<<std::endl;
// std::cout<<"offset calculated from ray and shading point: "<<ray.origin-isect->coords<<std::endl;
        return r;
    }

public:
    NDProfile     mProfile;
};

}
