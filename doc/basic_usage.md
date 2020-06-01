
ifm3d Basic Library Usage
=========================

A primary objective of `ifm3d` is to make it as simple and performant as
possible to acquire pixel data from an ifm 3D camera. Additionally, those data
should be encoded in a useful format for performing computer vision and/or
robotics perception tasks. A typical `ifm3d` client program will follow the
structure of a control loop whereby images are continuously acquired from the
camera and acted upon in some application-specific way. The example code below
shows how this may look:

```c++
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>

int main(int argc, const char **argv)
{
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  cv::Mat amp;
  cv::Mat xyz;

  while (true)
  {
    if (! fg->WaitForFrame(im.get(), 1000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

    amp = im->AmplitudeImage();
    xyz = im->XYZImage();

    // now do something with `amp` and `xyz`
  }

  return 0;
}
```

Let's break down this simple program to get a better sense of what is going on.

Inside of `main` on lines 10 - 12 we create the three core structures (camera,
framegrabber, and an image container)  that will be utilized by most `ifm3d`
programs:

```c++
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();
```

A few things must be pointed out here. First, `ifm3d` encourages the use of
`std::shared_ptr` as a means to manage the ownership and lifetimes of the core
actors in an `ifm3d` program. Indeed, you will notice that there is no need to
explictly allocate and deallocate memory. Second, to instantiate the
`ifm3d::Camera` object, a call to `ifm3d::Camera::MakeShared()` is made, rather
than calling `std::make_shared` directly. This wrapper function is used to
handle direct hardware probing to determine the type of camera that is
connected. For example, this may be an O3D303, O3X, or something
else. Regardless, a `std::shared_ptr` is returned from this call.

On lines 14 - 15, we declare a couple of OpenCV image containers to hold the
most recent amplitude and point cloud data frames from the camera:

```c++
 cv::Mat amp;
 cv::Mat xyz;
```

Then, in line 17 we begin our control loop. At the top of the control loop we
see the following conditional on lines 19 - 23:

```c++
    if (! fg->WaitForFrame(im.get(), 1000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }
```

The `WaitForFrame` method is called on our `ifm3d::FrameGrabber` pointer. It is
passed the *raw* pointer to the `ifm3d::ImageBuffer` and a timeout in
milliseconds. If a new frame from the camera cannot be acquired within the
timeout, `WaitForFrame` will return false, and this program will terminate with
an error message. As new data are acquired from the camera, the program will
continue in a loop grabbing the latest image from the camera and populating the
internal structure of the `ImageBuffer` with each new frame. For the curious,
we pass the raw pointer to the `ImageBuffer` as opposed to the smart pointer
because it is more performant (the `std::shared_ptr` reference count does not
need to be incremented and decremented with every call to `WaitForFrame` and,
semantically, the `FrameGrabber` via the `WaitForFrame` call does not
participate in the ownership of the `ImageBuffer`).

Finally, on lines 25 - 26, we call the `ifm3d::ImageBuffer` accessors to gain
access to the `cv::Mat` encoded image data. At this point we can unleash the
full power of OpenCV (or just exploit the `cv::Mat` array container utilizing
our own algorithms) on our data to help build our application.

```c++
   amp = im->AmplitudeImage();
   xyz = im->XYZImage();
```

This was just a simple example to give users of `ifm3d` a quick start to begin
utilizing ifm 3D cameras and the `ifm3d` library. There is much more that the
library and ecosystem are capable of, we look forward to seeing what you can
build. For additional help, we urge you to utilize our
[issue tracker](https://github.com/ifm/ifm3d/issues) and to look at the
examples provided [here](https://github.com/ifm/ifm3d-examples).
