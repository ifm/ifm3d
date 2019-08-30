UDP functionality in ifm3d
--------------------------

Version 1.50.4855 of the O3D3XX firmware introduced a UDP-based protocol for
transferring frame data in real time with minimal latency. This protocol is
implemented in the `framegrabberudp` module, introduced in v0.16.0 of the
`ifm3d` library.

`framegrabberudp` exposes data through the same `ByteBuffer` interface used by
current image containers.

## Building the Module

The `framegrabberudp` module is disabled by default. To build and install,
follow the instructions outlined in [source_build](source_build.md), but
include `-DBUILD_MODULE_FRAMEGRABBERUDP=ON` on the cmake command line. 

## Camera Configuration

The camera must be configured for UDP operation. These settings, like all
device settings, are exposed via XML-RPC and can be accessed through the
`camera` module. This is a one-time setup: once the device is configured, no
TCP communication is required for future data collection.

The device has the following schema for UDP parameters:

```
"Udp": {
      "Channels": "confidence_image,normalized_amplitude_image,amplitude_image",
      "Enabled": "true",
      "MaxPayloadSize": "1492",
      "Port": "50042",
      "TargetIP": "192.168.0.2"
    },
``` 

These parameters can be manipulated directly. For convenience, two helpers have
been provided for enabling and disabling UDP: `ifm3d::Camera::EnableUdp` and
`ifm3d::Camera::DisableUdp`. An example of usage follows below.

Alternatively, UDP settings can also be modified through the command line (via
the `tools` module: 

``` 
ifm3d udp --help 

usage: ifm3d [<global options>] udp [<udp options>]

global options:
  -h [ --help ]            Produce this help message and exit
  --ip arg (=192.168.0.69) IP address of the sensor
  --xmlrpc-port arg (=80)  XMLRPC port of the sensor
  --password arg           Password for establishing an edit-session with the
                           sensor

udp options:
  --disable                      Disable UDP functionality
  --target-ip arg                IP address of the receive endpoint in a
                                 unicast set-up
  -p [ --port ] arg (=50042)     The port number to which data needs to be sent
  --mask arg (=10)               The schema mask describing which channels to
                                 transmit
  --max-payload-size arg (=1492) The maximum payload size for each UDP packet
```

The `TargetIP` parameter refers to the unicast or multicast address of the
target (the recipient of the frame data). This value may be set as the
environment variable `IFM3D_UDP_TARGET_IP`. If set, this will be used
as the default in API calls.

## Receiving Frames

Once the camera has been set up, an instance of `ifm3d::FrameGrabberUdp` needs
to be created, associated with the proper port.

`ifm3d::FrameGrabberUdp` operates the same way as `ifm3d::FrameGrabber`. An
important caveat is that, due to the nature of UDP traffic, images specified in
the schema are not guaranteed to appear in the image buffer. Consumers must
first check the size of the resulting image buffers prior to usage.

## Example Usage

```c++
#include <cstdint>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg_udp.h>
#include <ifm3d/image.h>

int main(int argc, const char **argv)
{
  // 1. Camera provisioning (via TCP -- XML-RPC)
  //
  // This example is written as a single function, but in reality there is no
  // need to reconfigure the camera every time. After the initial provisioning
  // step, the runtime environment can be pure UDP based traffic.
  auto cam = ifm3d::Camera::MakeShared();
  cam->EnableUdp(ifm3d::IMG_RDIS | ifm3d::IMG_AMP, // PCIC-style schema
                 "192.168.0.2");                   // target machine IP address

  // 2. Image Acquisition
  // Create our image buffer to hold frame data from the camera
  // The UDP version of the FrameGrabber works on the same underlying structures
  // as the regular FrameGrabber, and so we can use the same image containers
  ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();

  // Create the UDP framegrabber, not associated with any particular camera
  ifm3d::FrameGrabberUdp::Ptr fg =
    std::make_shared<ifm3d::FrameGrabberUdp>();

  // Objects to hold image data
  cv::Mat conf, dist, amp;

  while (true)
    {
      if( !fg->WaitForFrame(img.get(), 1000))
        {
          std::cerr << "Timeout waiting for camera!" << std::endl;
          continue;
        }
      else
        {
          conf = img->ConfidenceImage();
          dist = img->DistanceImage();
          amp = img->AmplitudeImage();

          // Recall, at this point a frame is delivered, but the UDP protocol
          // delivers best-effort data. There is no guarantee that each channel
          // specified in the schema is included in this particular frame.
          //
          // One must inspect the size of the data for validity. For example:
          if (!dist.empty())
            {
              std::cout << "Distance image is valid!" << std::endl;
            }
          else
            {
              std::cout << "Distance image is not valid; skipping..."
                        << std::endl;
            }
        }
    }
  return 0;
}
```
