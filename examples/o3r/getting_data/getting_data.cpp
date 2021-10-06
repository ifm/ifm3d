/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <iostream>
#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/stlimage.h>
#include <ifm3d/fg/distance_image_info.h>


int main(){

    //////////////////////////
    // Declare the objects:
    //////////////////////////
    // Declare the device object (one object only, corresponding to the VPU)
    auto cam = std::make_shared<ifm3d::O3RCamera>();
    // Declare the FrameGrabber and ImageBuffer objects. 
    // One FrameGrabber per camera head (define the port number).
    const auto FG_PCIC_PORT = cam->Get({"/ports/port2/data/pcicTCPPort"});
    auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::DEFAULT_SCHEMA_MASK, FG_PCIC_PORT);
    auto im =  std::make_shared<ifm3d::StlImageBuffer>(); 

    //////////////////////////
    // Get a frame:
    //////////////////////////
    if (! fg->WaitForFrame(im.get(), 3000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }
    
    //////////////////////////
    // Example for 3D data:
    //////////////////////////
    auto dist = im->DistanceImage();

    std::cout << dist.height() << " " << dist.width() << std::endl;

    return 0;
}