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
    auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::DEFAULT_SCHEMA_MASK, 50012);
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
    ifm3d::Image dist;
    dist = im->DistanceImage();

    std::cout << dist.height() << " " << dist.width() << std::endl;

    // Calculate mean and std dev on a 3*3 pixel area:
    // cv::Mat mean, std_dev;
    // cv::meanStdDev(dist(cv::Range(height/2,height/2+3), cv::Range(width/2,width/2+3)), mean, std_dev);

    // std::cout << "Distance values of 3*3 center area: " << dist(cv::Range(height/2,height/2+3), cv::Range(width/2,width/2+3)) << std::endl; 
    // std::cout << "Mean: " << mean << std::endl;
    // std::cout << "Standard deviation: " << std_dev << std::endl;

    //////////////////////////
    // For 2D data:
    /////////////////////////
    /*
    ifm3d::Image rgb;
    rgb = im->JPEGImage();
    // Do something here  
    */

    return 0;
}