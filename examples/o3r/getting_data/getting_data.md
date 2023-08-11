# How to: receive an image

The primary objective of `ifm3d` is to make it as simple and performant as possible to acquire pixel data from an ifm 3D camera of the O3xxxx series.
Additionally, the data should be encoded in a useful format for performing computer vision and/or robotics perception tasks.
A typical `ifm3d` client program follows the structure of a control loop whereby images are continuously acquired from the camera and acted upon in some application-specific way.

At the end of this 'how to', you should be able to receive images and know the basic usage of the `O3R`, `FrameGrabber` and `Frame` classes.

>Note: for O3D or O3X devices, simply use the `O3D`/`O3X` class in place of the `O3R` in the following code.

## O3R, FrameGrabber and Frame

ifm3d provides three main classes:
- `O3R` holds the configuration of the camera heads, handles the connection, etc;
- `FrameGrabber` receives frames (images);
- `Frame` stores the image buffers.

Instantiating these objects is as follows:
:::::{tabs}
::::{group-tab} Python
:::python
o3r = O3R()
fg = FrameGrabber(o3r, pcic_port=50012)
:::
::::
::::{group-tab} C++
:::cpp
auto o3r = std::make_shared<ifm3d::O3R>();
auto fg = std::make_shared<ifm3d::FrameGrabber>(o3r, 50012);
:::
::::
:::::

>Note: The example above assumes that an O3R camera head is connected to the VPU at the physical port 2, corresponding to the PCIC TCP port 50012. If you're unsure about the connectivity, please see the section hardware unboxing under O3R/Getting Started.

The `O3R` class, counter-intuitively, refers to the computing unit (the VPU). It inherits its name from previous ifm 3D devices that only used one camera, with no distinction between sensing and computing units.
You can input:
- `ip`: the IP address of the device;
- `xmlrpc_port`: the XML_RPC port (it is fixed at the moment);
- `password`: the password to connect to the device (unused for the O3R).

The `FrameGrabber` stores a reference to the passed in camera shared pointer and starts a worker thread to stream in pixel data from the device.
Its inputs:
- `o3r`: The o3r instance (the image processing platform) that handles the connection the the camera heads;
- `port`: PCIC port number of the camera head to grab data from (not the physical port number);

> Note: instantiating the objects is done the same way for any imager type (2D, 3D, different resolutions, etc).

Note that `ifm3d` encourages the use of `std::shared_ptr` as a means to manage the ownership and lifetimes of the core actors in an `ifm3d` program.
Indeed, you will notice that there is no need to explicitly allocate and deallocate memory.
To instantiate the `ifm3d::O3R` object (or `ifm3d::O3D`/`ifm3d::O3X`), a call to `ifm3d::Device::MakeShared()` is made, rather than calling `std::make_shared` directly.
This wrapper function is used to handle direct hardware probing to determine the type of camera that is connected.
For example, this may be an O3R, O3D303, O3X, or something else. Regardless, a `std::shared_ptr` is returned from this call.

## Set the schema and start the FrameGrabber

To start the data stream, the `Start` function must be used. This `Start` needs a schema which is a `std::vector<ifm3d::buffer_id>` and contains the `buffer_id` of the data needed for the application:
:::::{tabs}
::::{group-tab} Python
:::python
fg.start([buffer_id.NORM_AMPLITUDE_IMAGE,buffer_id.RADIAL_DISTANCE_IMAGE,buffer_id.XYZ])
:::
::::
::::{group-tab} C++
:::cpp
fg->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE, ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,ifm3d::buffer_id::XYZ});
:::
::::
:::::

The device provides many types of buffers. All the buffers might not be required for application. Setting the schema provides a way to enable only `buffer_id`s required for the application. 
This also reduce the bandwidth required to get the data from the device.

## Receive an image

### Register a callback
The recommended way to receive a frame is to use the callback function. You can register a callback function that will be executed for every received frame, until the program exits.

:::::{tabs}
::::{group-tab} Python
:::python
def callback(frame):
    # Read the distance image and display a pixel in the center
    dist = frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
    (width, height) = dist.shape
    print(dist[width//2,height//2])
    pass

...
fg.start([buffer_id.RADIAL_DISTANCE_IMAGE])
fg.on_new_frame(callback)
...
fg.stop()
:::
::::
::::{group-tab} c++
:::cpp
void Callback(ifm3d::Frame::Ptr frame){
  auto dist = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
  std::cout << dist.height() << " " << dist.width() << std::endl;
}

int
main()
{
  ...
  fg->Start({ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE});
  fg->OnNewFrame(&Callback);
  ...
  fg->Stop();
}
:::
::::
:::::

### Alternatively: wait for a frame
You just need to call the `WaitForFrame` function. This `Framegrabber` method returns `std::Future<ifm3d::Frame>`. The `Frame` class stores all the received data. Make sure the camera head is in "RUN" mode.
:::::{tabs}
::::{group-tab} Python
:::python
frame = fg.wait_for_frame().wait() # wait without timeout
<!-- # OR -->
[ok, frame] = fg.wait_for_frame().wait_for(500) # wait with 500ms timeout
<!-- # OR -->
frame = await fg.wait_for_frame() # using asyncio
:::
::::
::::{group-tab} C++
:::cpp
auto frame = fg->WaitForFrame().get();
:::
::::
:::::

## Access the data

Accessing the received data is done through the `Frame`. Different data types are available depending on whether the camera is a 2D or a 3D camera.
Simply access the image by calling `get_buffer` passing the `buffer_id` of the required image as a parameter.
:::::{tabs}
::::{group-tab} Python
:::python
# For 3D data:
dist = frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
# For 2D data:
rgb = frame.get_buffer(buffer_id.JPEG_IMAGE)
:::
::::
::::{group-tab} C++
:::cpp
// For 3D data:
auto dist = frame->get_buffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
// For 2D data:
auto rgb = frame->get_buffer(ifm3d::buffer_id::JPEG_IMAGE);
:::
::::
:::::

## The full example

### Using a callback
:::::{tabs}
::::{group-tab} Python
:::{literalinclude} getting_data_callback.py
:language: python
:::
::::

::::{group-tab} C++
:::{literalinclude} getting_data_callback.cpp
:language: cpp
:::
::::
:::::

### Using the polling mode
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
