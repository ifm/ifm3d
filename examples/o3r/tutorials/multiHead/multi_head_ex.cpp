/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/camera/camera_o3r.h>
#include <iostream>
#include <iomanip>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>


int main(){

    // Declare the device object (one object only, corresponding to the VPU)
    auto cam = ifm3d::O3RCamera::MakeShared();
    // Retreive ports configuration 
    json conf = cam->ToJSON();
    auto ports = conf["ports"];
    std::vector<int> pcic_ports;
    std::vector<std::string> types;
    std::vector<ifm3d::FrameGrabber::Ptr> fgs;
    std::vector<ifm3d::ImageBuffer::Ptr> ims;


    std::cout << "Available connections:" << std::endl;
    // Create lists of connected PCIC ports along with types
    // Create list of FrameGrabber and ImageBuffer objects for connected ports
    for (const auto& port : ports.items())
    {
        auto pcic = port.value()["data"]["pcicTCPPort"];
        auto type = port.value()["info"]["features"]["type"];
        pcic_ports.push_back(pcic);
        types.push_back(type);        
        auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, 10, pcic);
        fgs.push_back(fg);
        auto im = std::make_shared<ifm3d::ImageBuffer>();
        ims.push_back(im);
        std::cout << "Port: " << pcic << "\t Type: " << type << std::endl;
    }

    return 0;
}