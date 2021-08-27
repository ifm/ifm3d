/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <opencv2/core/core.hpp>
#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
// #include <ifm3d/fg/distance_image_info.h>


int main(){

    //////////////////////////
    // Declare the objects:
    //////////////////////////
    // Declare the device object (one object only, corresponding to the VPU)
    auto cam = ifm3d::CameraBase::MakeShared();
    // Declare the FrameGrabber and ImageBuffer objects. 
    // One FrameGrabber per camera head (define the port number).
    auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, 10, 50012);
    auto im =  std::make_shared<ifm3d::ImageBuffer>();

    //////////////////////////
    // Get a frame:
    //////////////////////////
    if (! fg->WaitForFrame(im.get(), 3000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }
    
    // Do something with the data
    //////////////////////////
    // Example for 3D data:
    // Calculate the mean and std dev of a 3*3 pixel area
    //////////////////////////
    cv::Mat dist;
    dist = im->DistanceImage();

    int height = dist.rows;
    int width = dist.cols;

    std::cout << height << " " << width << std::endl;

    // Calculate mean and std dev on a 3*3 pixel area:
    cv::Mat mean, std_dev;
    cv::meanStdDev(dist(cv::Range(height/2,height/2+3), cv::Range(width/2,width/2+3)), mean, std_dev);

    std::cout << "Distance values of 3*3 center area: " << dist(cv::Range(height/2,height/2+3), cv::Range(width/2,width/2+3)) << std::endl; 
    std::cout << "Mean: " << mean << std::endl;
    std::cout << "Standard deviation: " << std_dev << std::endl;

    //////////////////////////
    // For 2D data:
    /////////////////////////
    /*
    cv::Mat rgb;
    rgb = im->JPEGImage();
    // Do something here  
    */

    return 0;
}