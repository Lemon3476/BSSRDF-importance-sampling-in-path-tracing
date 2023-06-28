#include "Renderer.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>


int main(int argc, char** argv)
{

    Scene scene(784, 784);
    // Scene scene(1280, 960);

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.65f);
    Material* MyWhite = new Material(TRANSLUCENT, Vector3f(0.0f));
    MyWhite->Kd = Vector3f(0.725f, 0.71f, 0.68f);

    MeshTriangle MyFloor("../models/cornellbox/MyFloor.obj", white);
    MeshTriangle MyLeft("../models/cornellbox/MyLeft.obj", red);
    MeshTriangle MyRight("../models/cornellbox/MyRight.obj", green);
    MeshTriangle MyLight("../models/cornellbox/MyLight.obj", light);
    MeshTriangle teapot("../models/teapot/teapot.obj", MyWhite);
    scene.Add(&MyFloor);
    scene.Add(&MyLeft);
    scene.Add(&MyRight);
    scene.Add(&MyLight);
    scene.Add(&teapot);
 
    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}
