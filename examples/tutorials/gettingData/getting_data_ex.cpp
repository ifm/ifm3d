#include <opencv2/core/core.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>

int main(){


    // Declare the device object (one object only, corresponding to the VPU)
    auto cam = ifm3d::Camera::MakeShared();
    // Declare the FrameGrabber and ImageBuffer objects. 
    // One FrameGrabber per camera head (define the port number).
    auto fg_0 = std::make_shared<ifm3d::FrameGrabber>(cam, 10, 50010);
    auto im_0 =  std::make_shared<ifm3d::ImageBuffer>();
    // TODO create multiple grabbers and buffers (one per head). Use vectors?

    // Get a frame
    fg_0->WaitForFrame(im_0.get(), 1000);

    // Do something with the data
    cv::Mat amp;
    cv::Mat dist;  

    amp = im_0->AmplitudeImage();
    dist = im_0->DistanceImage();

    std::cout << amp.at<int>(100, 100) << std::endl;
    std::cout << dist.at<int>(100, 100) << std::endl;


    return 0;
}