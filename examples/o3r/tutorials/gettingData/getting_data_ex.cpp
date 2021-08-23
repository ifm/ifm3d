/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <opencv2/core/core.hpp>
#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>

int main(){


    // Declare the device object (one object only, corresponding to the VPU)
    auto cam = ifm3d::CameraBase::MakeShared();
    // Declare the FrameGrabber and ImageBuffer objects. 
    // One FrameGrabber per camera head (define the port number).
    auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, 10, 50012);
    auto im =  std::make_shared<ifm3d::ImageBuffer>();
    // TODO create multiple grabbers and buffers (one per head). Use vectors?

    // Get a frame
    if (! fg->WaitForFrame(im.get(), 3000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }
    
    // Do something with the data
    // For 3D data:
    cv::Mat amp;
    cv::Mat dist;  

    amp = im->AmplitudeImage();
    dist = im->DistanceImage();

    std::cout << dist << std::endl; 
    
   /*
    // For 2D data:
    cv::Mat rgb;
    rgb = im->JPEGImage();
    std::cout << rgb << std::endl;
    */

    return 0;
}