#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <pcl/point_cloud.h>


int main(int argc, const char **argv)
{
  // Declare the objects (one framegrabber per camera head)
  auto cam = ifm3d::Camera::MakeShared();
  //auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, 10, 50013);
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, 10, 50010);
  auto im = std::make_shared<ifm3d::ImageBuffer>();
  cv::Mat amp;
  cv::Mat xyz;


  
//   while (true)
//   {
    if (! fg->WaitForFrame(im.get(), 1000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

    amp = im->AmplitudeImage();
    xyz = im->XYZImage();
    // std::cout << amp << std::endl;
    // std::cout << xyz << std::endl;
    std::cout << "Amp size " << amp.size() << std::endl;
    std::cout << "Amp dims " << amp.dims << std::endl; 
    std::cout << "XYZ size " << xyz.size() << std::endl;
    std::cout << "XYZ dims " << xyz.dims << std::endl;

    std::cout << xyz.at<int>(100, 100) << std::endl;
    std::cout << xyz.type() << std::endl;


//   }

  return 0;
}