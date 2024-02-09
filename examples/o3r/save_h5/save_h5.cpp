#include <iostream>
#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/stlimage.h>
#include <ifm3d/fg/distance_image_info.h>
#include <opencv2/core/core.hpp>
#include <librealsense2/rs.hpp>
#include <H5Cpp.h>

int main(){
    auto o3r = std::make_shared<ifm3d::O3RCamera>();
    auto fg = std::make_shared<ifm3d::FrameGrabber>(
            o3r, 
            ifm3d::DEFAULT_SCHEMA_MASK,
            50012);
    auto im =  std::make_shared<ifm3d::StlImageBuffer>();

    if (! fg->WaitForFrame(im.get(), 3000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

    return 0;
}