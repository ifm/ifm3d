/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <iostream>
#include <fstream>
#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/stlimage.h>
#include <ifm3d/fg/distance_image_info.h>
#include <opencv2/core/core.hpp>

cv::Mat ConvertImageToMatNoCopy(ifm3d::Buffer& img, int data_type)
{
  return cv::Mat(img.height(), img.width(), data_type, img.ptr(0));
}

std::ostream& Serialize(std::ostream& outfile, int** arr, int rows, int cols) {
    outfile << rows << " ";
    outfile << cols << " ";
    for (int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            outfile << arr[i][j] << " ";
        }
    }
    return outfile;
}

int** Deserialize(std::istream& file, int& rows, int& cols) {
    file >> rows;
    file >> cols;
    int** arr = new int*[rows];
    for (int i = 0; i < rows; i++) {
        arr[i] = new int[cols];
        for(int j = 0; j < cols; j++)
            file >> arr[i][j];
    }
    return arr;
}

int main(){

    auto cam = std::make_shared<ifm3d::O3RCamera>();
    const auto FG_PCIC_PORT = cam->Get()["/ports/port0/data/pcicTCPPort"_json_pointer];
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
    auto height = dist.height();
    auto width = dist.width();

    auto dist_cv = ConvertImageToMatNoCopy(dist, CV_16U);
    int arr[2][3] = {
        {1, 2, 3},
        {4, 5, 6}
    };
    std::ofstream dfile("Distance.txt");
    Serialize(dfile, **arr, height, width);
    dfile.close();
    return 0;
}