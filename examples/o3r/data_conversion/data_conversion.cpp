#include <iostream>
#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/stlimage.h>
#include <ifm3d/fg/distance_image_info.h>
#include <opencv2/core/core.hpp>
#include <librealsense2/rs.hpp>


cv::Mat ConvertImageToMatNoCopy(ifm3d::Buffer& img, int data_type)
{
  return cv::Mat(img.height(), img.width(), data_type, img.ptr(0));
}

rs2_intrinsics ConvertIntrinsics(std::vector<float>& intr, ifm3d::Buffer& img)
{
  return 
    {
      img.width(),
      img.height(),
      intr[static_cast<uint32_t>(ifm3d::intrinsic_param::M_X)],
      intr[static_cast<uint32_t>(ifm3d::intrinsic_param::M_Y)],
      intr[static_cast<uint32_t>(ifm3d::intrinsic_param::F_X)],
      intr[static_cast<uint32_t>(ifm3d::intrinsic_param::F_Y)],
      rs2_distortion::RS2_DISTORTION_FTHETA,
      {
        intr[static_cast<uint32_t>(ifm3d::intrinsic_param::K1)], 
        intr[static_cast<uint32_t>(ifm3d::intrinsic_param::K2)], 
        intr[static_cast<uint32_t>(ifm3d::intrinsic_param::K3)], 
        intr[static_cast<uint32_t>(ifm3d::intrinsic_param::K4)], 
        intr[static_cast<uint32_t>(ifm3d::intrinsic_param::K5)]
        },
    };
}

int main(){
    auto o3r = std::make_shared<ifm3d::O3RCamera>();
    auto fg = std::make_shared<ifm3d::FrameGrabber>(
            o3r, 
            ifm3d::DEFAULT_SCHEMA_MASK,
            50010);
    auto im =  std::make_shared<ifm3d::StlImageBuffer>();

    if (! fg->WaitForFrame(im.get(), 3000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

    // Get data
    auto dist = im->DistanceImage();
    auto intr = im->Intrinsics();
    // Convert data 
    // Warning: no copy conversion! 
    // Make sure `dist` is not destroyed
    auto dist_cv = ConvertImageToMatNoCopy(dist, CV_16U);

    auto rs_like_intr = ConvertIntrinsics(intr, dist);

    std::cout << rs_like_intr.fx << std::endl;

    return 0;
}