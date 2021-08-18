# How to: receive an image
Corresponding code example: [getting_data_ex.cpp](getting_data_ex.cpp)

You successfully installed the ifm3d library which contains all you need to use the O3R. This library wraps around PCIC and XML-RPC communication protocols to facilitate the communication with the device.

At the end of this 'how to', you should be able to receive images and know the basic usage of the `Device`, `FrameGrabber` and `ImageBuffer` classes.

## `Camera`, `FrameGrabber` and `ImageBuffer`

ifm3d provides three main classes:
- `Camera` holds the configuration of the camera heads, handles the connection, etc;
- `FrameGrabber` receives frames (images);
- `ImageBuffer` stores the image data.

Instantiating these objects is as follows:
```cpp
auto cam = ifm3d::Camera::MakeShared();
auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, 10, 50010);
auto im =  std::make_shared<ifm3d::ImageBuffer>();
```
The `Camera` class, counter-intuitively, refers to the computing unit (the VPU). It inherits its name from previous ifm 3D devices that only used one camera, with no distinction between sensing and computing units. 
You can input:
- `ip`: the IP address of the device;
- `xmlrpc_port`: the XML_RPC port (it is fixed at the moment);
- `password`: the password to connect to the VPU, is any.

The `FrameGrabber` stores a reference to the passed in camera shared pointer and starts a worker thread to stream in pixel data from the device.
Its inputs:
- `cam`: The camera instance to grab frames from;
- `mask`: A bitmask encoding the image acquisition schema to stream in from the camera. #TODO: add link to schema mask doc;
- `port`: Port number of the camera head to grab data from;

The `ImageBuffer` class simply serves to store the received data. It allocates space for each individual image.

> Note: instantiating the objects is done the same way for any imager type (2D, 3D, different resolutions, etc).

## Receive an image

You just need to call the `WaitForFrame` function. Input an `ImageBuffer` object as well as a timeout value in ms.
```cpp
fg->WaitForFrame(im.get(), 1000);
```

And you're good to go! Now you can do something with all this data.

## Access the data

Accessing the received data is done through the `ImageBuffer`. Different data types are available depending on whether the camera is a 2D or a 3D camera. Simply access the image like so:
```cpp
// For 3D data:
cv::Mat dist;  
dist = im->DistanceImage();
// For 2D data:
cv::Mat rgb;
rgb = im->JPEGImage();
```