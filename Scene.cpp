#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
        return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;//[0~1]*13650    
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){//random get the first area > p light,return                
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}



bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}


Intersection Scene::shade(Intersection hit_obj, Vector3f wo) const
{
    if (hit_obj.m->hasEmission())
    {
        // return hit_obj.m->getEmission();
        hit_obj.color = hit_obj.m->getEmission();
    }
    const float epsilon = EPSILON;
    Vector3f Lo_dir;
    {
		float light_pdf;
		Intersection hit_light;
		sampleLight(hit_light, light_pdf);
		Vector3f obj2Light = hit_light.coords - hit_obj.coords;
        Vector3f obj2LightDir = obj2Light.normalized();
       
        auto t = intersect(Ray(hit_obj.coords, obj2LightDir));
        if (t.distance - obj2Light.norm() > -epsilon)
        {
			Vector3f f_r = hit_obj.m->eval(obj2LightDir, wo, hit_obj.normal);
			float r2 = dotProduct(obj2Light, obj2Light);
            float cosA = std::max(.0f, dotProduct(hit_obj.normal,obj2LightDir));
            float cosB = std::max(.0f, dotProduct(hit_light.normal,-obj2LightDir));
			Lo_dir = hit_light.emit * f_r * cosA * cosB / r2 / light_pdf;
        }
    }

    Vector3f Lo_indir;
    {
		if (get_random_float() < RussianRoulette)
		{
			Vector3f dir2NextObj = hit_obj.m->sample(wo, hit_obj.normal).normalized();
            float pdf = hit_obj.m->pdf(wo, dir2NextObj, hit_obj.normal);
            if (pdf > epsilon)
            {
                Intersection nextObj = intersect(Ray(hit_obj.coords, dir2NextObj));
				if (nextObj.happened && !nextObj.m->hasEmission())
				{
					Vector3f f_r = hit_obj.m->eval(dir2NextObj, wo, hit_obj.normal); //BRDF
					float cos = std::max(.0f, dotProduct(dir2NextObj, hit_obj.normal));
					Lo_indir = shade(nextObj, -dir2NextObj).color * f_r * cos / pdf / RussianRoulette;
				}
            }
		}
    }

    // return Lo_dir + Lo_indir;
    hit_obj.color = Lo_dir + Lo_indir;
// std::cout<<"base color: "<<hit_obj.color<<std::endl;
    return hit_obj;
}



// Implementation of Path Tracing
Intersection Scene::castRay(const Ray &ray, Vector3f eye_pos) const
{
int myCounter=0;
	auto hitObj = intersect(ray);
	if (!hitObj.happened) return {};
    hitObj = shade(hitObj,-ray.direction);
    Ray probeRay(Vector3f(0.0f), Vector3f(0.0f));
    if(hitObj.m->m_type == TRANSLUCENT)
    {
        rls::SssSampler sampler(Vector3f(50.0f)); 
        float   samples[2];
        // hitObj.probeDepth = 0;
// std::cout<<"test test test!!!"<<std::endl;
        unsigned int hitRecords = 0;
        while(hitRecords < kMaxProbeDepth)
        {
            Bounds3 box = hitObj.mesh->getBounds();
            samples[0] = get_random_float();
            samples[1] = get_random_float();
            float r = sampler.getProbeRay(samples[0], samples[1], &hitObj, probeRay);
            if (r < sampler.mProfile.maxRadius())
            {
                probeRayExtension(probeRay, box);
                auto sampleIsect = intersect(probeRay);
                if (sampleIsect.happened == true && sampleIsect.mesh == hitObj.mesh)
                {
                    sampleIsect.sampleRadius = r;
                    sampleIsect.sampleDisp = hitObj.coords - sampleIsect.coords;
                    if (V3Length(sampleIsect.sampleDisp) < sampler.mProfile.maxRadius())
                    {
                        hitRecords++;
                        Vector3f dir = normalize(sampleIsect.coords - eye_pos);
                        sampleIsect = shade(sampleIsect, -dir);
                        if (V3Length(sampleIsect.color - RGB_BLACK) > EPSILON)
                        {
                            hitObj.sampleIsects[hitObj.probeDepth] = new Intersection(sampleIsect);
                            hitObj.sampleIsects[hitObj.probeDepth]->ray = new Ray(probeRay);
                            hitObj.probeDepth++;
                        }
                    }
                }
            }
        }
    }
    return hitObj;
}


void probeRayExtension(Ray& probeRay, Bounds3& box)
{
    probeRay.direction_inv = Vector3f(1./probeRay.direction.x, 1./probeRay.direction.y, 1./probeRay.direction.z);
    probeRay.direction = -probeRay.direction;
    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = int(probeRay.direction.x >= 0);
    dirIsNeg[1] = int(probeRay.direction.y >= 0);
    dirIsNeg[2] = int(probeRay.direction.z >= 0);
    box.IntersectBound(probeRay, probeRay.direction_inv, dirIsNeg);
    float t = std::min(abs(probeRay.t_min), abs(probeRay.t_max));
    probeRay.origin += (t+100000*EPSILON)*probeRay.direction;
    probeRay.direction = -probeRay.direction;
// std::cout<<"t_max and t_min: "<<probeRay.t_max<<" "<<probeRay.t_min<<std::endl;
// std::cout<<"origin of probe ray: "<<probeRay.origin<<std::endl;
// std::cout<<"direction of probe ray: "<<probeRay.direction<<std::endl;
}




void    rls::NDProfile::setDistance(const Vector3f &dist)
{
    mDistance = dist;
    // Scale the max tracing distance empirically.
    // mMaxRadius = std::max(mDistance.x, std::max(mDistance.y, mDistance.z)) * 3.0f;
    mMaxRadius = std::max(mDistance.x, std::max(mDistance.y, mDistance.z));

    for (auto i = 0; i < 3; i++) {
        auto d = mDistance[i];
        mC1[i] = 1.0f - exp(-mMaxRadius / d);
        mC2[i] = 1.0f - exp(-mMaxRadius / d / 3.0f);
    }
}

float   rls::NDProfile::getRadius(float rx) const
{
    if (mMaxRadius < EPSILON) {
        return 0.0f;
    }

    // Since the profile is integrated to one for any positive d over the disk,
    // thus we could uniformly select the distance value.
    int distIdx = selectDistLobe(rx);
    auto d = mDistance[distIdx];
    if (d < EPSILON) {
        return 0.0f;
    }

    /*auto w1 = 1.0f - exp(-mMaxDist / d);
    auto w2 = 1.0f - exp(-mMaxDist / d / 3.0f);*/
    auto w1 = mC1[distIdx];
    auto w2 = mC2[distIdx];
    float w = w1 / (w1 + w2 * 3.0f);

    auto r = 1.0f;
    if (rx > w) {
        rx = LINEARSTEP(w, 1.0f, rx);
        r = log(1.0f - rx * w2) * (-d * 3.0f);
    } else {
        rx = LINEARSTEP(0.0f, w, rx);
        r = log(1.0f - rx * w1) * (-d);
    }

    return r;
}

float   rls::NDProfile::getPdf(float r) const
{
    if (mMaxRadius < EPSILON) {
        return 1.0f;
    }

    float pdf = 0.0f;

    for (auto i = 0u; i < 3; i++) {
        auto d = std::max((float)mDistance[i], EPSILON);
        auto p1 = exp(-r / d);
        auto p2 = exp(-r / d / 3.0f);
        pdf += (p1 + p2) / d / (mC1[i] + mC2[i] * 3.0f);
    }

    return pdf / (2 * M_PI * r * 3.0f);
}

RGB   rls::NDProfile::evalProfile(float r) const
{
    if (mMaxRadius < EPSILON) {
        return RGB_BLACK;
    } else if (r < EPSILON) {
        return RGB_WHITE;
    }

    auto denom = 8.0f * M_PI * r;

    RGB result = RGB_BLACK;
    for (auto i = 0u; i < 3; i++) {
        // Remap the distance for artistic control
        auto d = mDistance[i];
        result[i] = d < EPSILON ? 1.0f
            : (exp(-r / d) + exp(-r / (3.0f * d))) / (denom * d);
    }

    return result;
}