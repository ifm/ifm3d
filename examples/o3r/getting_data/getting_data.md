# How to: receive an image

A primary objective of `ifm3d` is to make it as simple and performant as possible to acquire pixel data from an ifm 3D camera. 
Additionally, those data should be encoded in a useful format for performing computer vision and/or robotics perception tasks. 
A typical `ifm3d` client program will follow the structure of a control loop whereby images are continuously acquired from the camera and acted upon in some application-specific way. 

At the end of this 'how to', you should be able to receive images and know the basic usage of the `O3RCamera`, `FrameGrabber` and `StlImageBuffer` classes.

>Note: for O3D or O3X devices, simply use the `Camera` class in place of the `O3RCamera` in the following code.

## O3RCamera, FrameGrabber and StlImageBuffer

ifm3d provides three main classes:
- `O3RCamera` holds the configuration of the camera heads, handles the connection, etc;
- `FrameGrabber` receives frames (images);
- `StlImageBuffer` stores the image data.

Instantiating these objects is as follows:
:::::{tabs}
::::{group-tab} Python
:::python
cam = O3RCamera()
fg = FrameGrabber(o3r, pcic_port=50012)
im =  ImageBuffer()
:::
::::
::::{group-tab} C++
:::cpp
auto cam = std::make_shared<ifm3d::O3RCamera>();
auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::DEFAULT_SCHEMA_MASK, 50012);
auto im =  std::make_shared<ifm3d::StlImageBuffer>();
:::
::::
:::::

The `O3RCamera` class, counter-intuitively, refers to the computing unit (the VPU). It inherits its name from previous ifm 3D devices that only used one camera, with no distinction between sensing and computing units. 
You can input:
- `ip`: the IP address of the device;
- `xmlrpc_port`: the XML_RPC port (it is fixed at the moment);
- `password`: the password to connect to the device (unused for the O3R).

The `FrameGrabber` stores a reference to the passed in camera shared pointer and starts a worker thread to stream in pixel data from the device.
Its inputs:
- `cam`: The camera instance (the image processing platform) that handles the connection the the camera heads;
- `mask`: A bitmask encoding the image acquisition schema to stream in from the camera;
- `port`: Port number of the camera head to grab data from (not the physical port number);

The `StlImageBuffer` class simply serves to store the received data. It allocates space for each individual image.

> Note: instantiating the objects is done the same way for any imager type (2D, 3D, different resolutions, etc).

Note that `ifm3d` encourages the use of `std::shared_ptr` as a means to manage the ownership and lifetimes of the core actors in an `ifm3d` program. 
Indeed, you will notice that there is no need to explicitly allocate and deallocate memory. 
To instantiate the `ifm3d::O3RCamera` object (or `ifm3d::Camera`), a call to `ifm3d::O3RCamera::MakeShared()` is made (or `ifm3d::Camera::MakeShared()`), rather than calling `std::make_shared` directly. 
This wrapper function is used to handle direct hardware probing to determine the type of camera that is connected. 
For example, this may be an O3R, O3D303, O3X, or something else. Regardless, a `std::shared_ptr` is returned from this call.

## Receive an image

You just need to call the `WaitForFrame` function. Input an `ImageBuffer` object as well as a timeout value in ms. Make sure the camera head is in "RUN" mode.
:::::{tabs}
::::{group-tab} Python
:::python
fg.wait_for_frame(im, 1000)
:::
::::
::::{group-tab} C++
:::cpp
fg->WaitForFrame(im.get(), 1000);
:::
::::
:::::

And you're good to go! Now you can do something with all this data.
> Note : 
> The `WaitForFrame` method is called on our `ifm3d::FrameGrabber` pointer. 
> It is passed the *raw* pointer to the `ifm3d::ImageBuffer` and a timeout in milliseconds. 
> If a new frame from the camera cannot be acquired within the timeout, `WaitForFrame` will return false.
> For the curious, we pass the raw pointer to the `ImageBuffer` as opposed to the smart pointer because it is more performant (the `std::shared_ptr` reference count does not need to be incremented and decremented with every call to `WaitForFrame` and, semantically, the `FrameGrabber` via the `WaitForFrame` call does not participate in the ownership of the `ImageBuffer`).

## Access the data

Accessing the received data is done through the `StlImageBuffer`. Different data types are available depending on whether the camera is a 2D or a 3D camera. Simply access the image like so:
:::::{tabs}
::::{group-tab} Python
:::python
# For 3D data:
dist = im.distance_image();
# For 2D data:
rgb = im.jpeg_image();
:::
::::
::::{group-tab} C++
:::cpp
// For 3D data:
auto dist = im->DistanceImage();
// For 2D data:
auto rgb = im->JPEGImage();
:::
::::
:::::

## The full example
:::::{tabs}
::::{group-tab} Python
:::{literalinclude} getting_data.py
:language: python
:::
::::

::::{group-tab} C++
:::{literalinclude} getting_data.cpp
:language: cpp
:::
::::
:::::
```