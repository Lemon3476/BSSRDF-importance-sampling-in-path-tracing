#include <fstream>
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = (1.0e-4f);

void Renderer::Render(const Scene& scene)
{
    std::vector<Intersection> framebuffer(scene.width * scene.height); 
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(0, 250, -765);
    // Vector3f eye_pos(0, -350, -800);

    int m = 0;
    Intersection* sampleIsects[kMaxProbeDepth];

    // change the spp value to change sample ammount
    int spp = 16;
    // path tracing and BSSRDF importacne sampling
    std::cout << "SPP: " << spp << "\n";
    std::cout << "BSSRDF" << "\n";
    std::cout << "path tracing and BSSRDF importacne sampling" << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
            Vector3f dir = normalize(Vector3f(-x, y, 1));
            RGB colorBase = RGB_BLACK;
            // number of right sampled points
            int s=0; 
            for (int k = 0; k < spp; k++){
                framebuffer[m] = scene.castRay(Ray(eye_pos, dir), eye_pos);
                colorBase += framebuffer[m].color;
                if(framebuffer[m].happened)
                {
                    if(framebuffer[m].m->m_type==TRANSLUCENT)
                    {
                        for(int fn=0;fn<framebuffer[m].probeDepth;fn++)
                        {
                            sampleIsects[s]=framebuffer[m].sampleIsects[fn];
                            if(s<kMaxProbeDepth)
                            {s++;}
                        }
                    }
                }

            }
            colorBase.x /= spp;
            colorBase.y /= spp; 
            colorBase.z /= spp;
            framebuffer[m].color = colorBase;
            if(framebuffer[m].happened)
            {
                if(framebuffer[m].m->m_type==TRANSLUCENT)
                {
                    framebuffer[m].probeDepth = s;
// std::cout<<"probe depth:"<<framebuffer[m].probeDepth<<std::endl;
                    for(int h=0;h<framebuffer[m].probeDepth;h++)
                    {
                        framebuffer[m].sampleIsects[h] = sampleIsects[h];
                    }
                }
            }   
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);
    std::cout<<std::endl;


    //SSS shading
    rls::SssSampler sampler(Vector3f(50.0f)); 
    std::cout << "SSS shading" << "\n";
    for (int n=0; n < m; ++n) 
    {
// std::cout<<"base color: "<<framebuffer[n].color<<std::endl;
        if(framebuffer[n].happened)
        {
            if(framebuffer[n].m->m_type==TRANSLUCENT)
            {
std::cout<<"base color: "<<framebuffer[n].color<<std::endl;
                    framebuffer[n].shadeSSS = sampler.shadeSSS(&framebuffer[n]);
std::cout<<"final color: "<<framebuffer[n].color<<std::endl;
// std::cout<<"probe depth: "<<framebuffer[n].probeDepth<<std::endl;
            }
        }
    }


    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].color.x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].color.y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].color.z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
