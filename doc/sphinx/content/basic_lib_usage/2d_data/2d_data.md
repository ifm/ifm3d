# How to: receive and use the 2D RGB image

Receiving RGB data with ifm3d is done similarly as 3D data: the core objects have to be instantiated, and a frame has to be retrieved (see full code below). 
The important part is how to access the RGB image and how to decode it for further use.

## Access the data
The RGB image is stored in JPEG format and can be retrieved as follows:
:::::{tabs}
::::{group-tab} Python
:::python
jpeg = frame.get_buffer(buffer_id.JPEG_IMAGE)
:::
::::
::::{group-tab} C++
:::cpp
auto jpeg = frame->GetBuffer(ifm3d::buffer_id::JPEG_IMAGE);
:::
::::
:::::

## Decode the data
Once accessed, the RGB image has to be decoded. We use OpenCV in this example:
:::::{tabs}
::::{group-tab} Python
:::python
rgb_decode = cv2.imdecode(jpeg, cv2.IMREAD_UNCHANGED)
:::
::::
::::{group-tab} C++
:::cpp
auto rgb_decode = cv::imdecode(jpeg, cv::IMREAD_UNCHANGED);
:::
::::
:::::

## Display (optional)
The decoded image can then be displayed, for instance with OpenCV. 
> Note that in c++, the image first has to be converted to a cv::Mat.
> Follow the full example for the conversion to cv::Mat with or without copy.
:::::{tabs}
::::{group-tab} Python
:::python
cv2.startWindowThread()
cv2.namedWindow("2D image", cv2.WINDOW_NORMAL)
# get frame
# ...
... 
cv2.imshow('RGB image', rgb_decode)
cv2.waitKey(0)
:::
::::
::::{group-tab} C++
:::cpp
cv::startWindowThread();
cv2.namedWindow("RGB image", cv2::WINDOW_NORMAL)

cv::imshow("RGB image", rgb_decode);
cv::waitKey(0);
:::
::::
:::::

## The full example
:::::{tabs}
::::{group-tab} Python
:::{literalinclude} ../../ifm3d-examples/ovp8xx/python/ovp8xxexamples/core/2d_data.py

:language: python
:::
::::

::::{group-tab} C++
:::{literalinclude} ../../ifm3d-examples/ovp8xx/cpp/core/2d_data/2d_data.cpp
:language: cpp
:::
::::
:::::