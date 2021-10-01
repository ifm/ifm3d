# How to: receive an image
Corresponding code example: [getting_data_ex.cpp](getting_data_ex.cpp)

You successfully installed the ifm3d library which contains all you need to use the O3R. This library wraps around PCIC and XML-RPC communication protocols to facilitate the communication with the device.

At the end of this 'how to', you should be able to receive images and know the basic usage of the `O3RCamera`, `FrameGrabber` and `StlImageBuffer` classes.

## `O3RCamera`, `FrameGrabber` and `StlImageBuffer`

ifm3d provides three main classes:
- `O3RCamera` holds the configuration of the camera heads, handles the connection, etc;
- `FrameGrabber` receives frames (images);
- `StlImageBuffer` stores the image data.

Instantiating these objects is as follows:
```cpp
auto cam = std::make_shared<ifm3d::O3RCamera>();
auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::DEFAULT_SCHEMA_MASK, 50012);
auto im =  std::make_shared<ifm3d::StlImageBuffer>();
```
The `O3RCamera` class, counter-intuitively, refers to the computing unit (the VPU). It inherits its name from previous ifm 3D devices that only used one camera, with no distinction between sensing and computing units. It is able to distinguish between the different devices (O3R, O3D, O3X, ...) and therefore access the relevant functionalities.
You can input:
- `ip`: the IP address of the device;
- `xmlrpc_port`: the XML_RPC port (it is fixed at the moment);
- `password`: the password to connect to the VPU, if any.

The `FrameGrabber` stores a reference to the passed in camera shared pointer and starts a worker thread to stream in pixel data from the device.
Its inputs:
- `cam`: The camera instance (the image processing platform) that handles the connection the the camera heads;
- `mask`: A bitmask encoding the image acquisition schema to stream in from the camera;
- `port`: Port number of the camera head to grab data from (not the physical port number);

The `StlImageBuffer` class simply serves to store the received data. It allocates space for each individual image.

> Note: instantiating the objects is done the same way for any imager type (2D, 3D, different resolutions, etc).

## Receive an image

You just need to call the `WaitForFrame` function. Input an `ImageBuffer` object as well as a timeout value in ms.
```cpp
fg->WaitForFrame(im.get(), 1000);
```

And you're good to go! Now you can do something with all this data.

## Access the data

Accessing the received data is done through the `StlImageBuffer`. Different data types are available depending on whether the camera is a 2D or a 3D camera. Simply access the image like so:
```cpp
// For 3D data:
ifm3d::Image dist;  
dist = im->DistanceImage();
// For 2D data:
ifm3d::Image rgb;
rgb = im->JPEGImage();
```