#include <iostream>
#include <iomanip>
#include <memory>
#include <fstream>
#include <ifm3d/camera.h>

int main(){

    // Create the camera object
    auto cam = ifm3d::Camera::MakeShared();

    // Get the current configuration of the camera in JSON format
    json conf = cam->ToJSON();
    // Display and then write to file
    std::cout << std::setw(4) << conf << std::endl;
    std::ofstream file_get("/home/usmasslo/O3R/ifm3d/examples/tutorials/conf_get.json");
    file_get << std::setw(4) << conf;

    // Configure the device from a configuration file
    std::ifstream file_set("/home/usmasslo/O3R/ifm3d/examples/tutorials/conf_set.json");
    file_set >> conf;
    cam->FromJSON(conf);

    // Check that the configuration worked
    conf = cam->ToJSON();
    std::cout << std::setw(4) << conf << std::endl;


    

    return 0;
}