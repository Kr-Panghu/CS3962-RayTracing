#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    Scene scene(1280, 960);
    int option = 0;
    if(argc == 2 && std::string(argv[1]) == "SAH") {
        option = 1;
    }

    MeshTriangle bunny("./models/bunny/bunny.obj", option);

    scene.Add(&bunny);
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 1));
    scene.Add(std::make_unique<Light>(Vector3f(20, 70, 20), 1));

    bool check_mode = false;
    if (argc == 2 && std::string(argv[1]) == "check")
    {
        std::cout << "Rendering using Check mode\n";
        check_mode = true;
    }else{
        if(option == 1) {
            std::cout << std::endl << "🚀Rendering by SPLIT_BY_SAH" << std::endl << std::endl;
        } else {
            std::cout << std::endl << "🚀Rendering by SPLIT_BY_COUNT" << std::endl << std::endl;
        }
        scene.buildBVH(option);
    }


    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene,check_mode);

    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}