# How to: receive an image
Corresponding code example: [getting_data_ex.cpp](getting_data_ex.cpp)

You successfully installed the ifm3d library which contains all you need to use the O3R. This library wraps around PCIC and XML-RPC communication protocols to facilitate the communication with the device.

At the end of this 'how to', you should be able to receive images and know the basic usage of the `Device`, `FrameGrabber` and `ImageBuffer` classes.

## `Camera`, `FrameGrabber` and `ImageBuffer`

ifm3d provides three main classes:
- `Device` holds the configuration of the camera head, handles the connection and sets the camera in the proper state (RUN or CONF);
- `FrameGrabber` receives frames (images);
- `ImageBuffer` stores the image data.

Instantiating these objects is as follows:
```cpp
auto cam = ifm3d::Camera::MakeShared();
auto fg_0 = std::make_shared<ifm3d::FrameGrabber>(cam, 10, 50010);
auto im_0 =  std::make_shared<ifm3d::ImageBuffer>();
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

## Receive an image

You just need to call the `WaitForFrame` function. Input an `ImageBuffer` object as well as a timeout value in ms.
```cpp
fg_->WaitForFrame(im.get(), 1000);
```

And you're good to go! Now you can do something with all this data.

