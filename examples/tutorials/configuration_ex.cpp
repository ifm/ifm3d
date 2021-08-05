#include <iostream>
#include <memory>
#include <fstream>
#include <ifm3d/camera.h>

int main(){

    // Create the camera object
    auto cam = ifm3d::Camera::MakeShared();

    // Get the current configuration of the camera in JSON format
    json conf = cam->ToJSON();
    // Display and then write to file
    std::cout << conf << std::endl;
    std::ofstream file("conf.json");
    file << conf;

    

    return 0;
}