
ifm3d - Schema Description and usage
====================================

`ifm3d` provides `FrameGrabber` class to grab the data from the device,
ifm devices provide many type of data and images, user can configure
`FrameGrabber` for data required by the application, this reduces
the network traffic while grabbing the data. To configure the `FrameGrabber`
object for particular image or/and data user needs to provide the 16 bit value,
called as the schema mask. 12 bits out of 16 bits are used to configure the 12 types
data and images provided by ifm 3d vision devices. 4 bits are reserved for
future. To create schema mask bitwise OR the 16 bit values of the
data/Image needed for application
 
Following table shows  the macro and values used to create schema mask:

| Data/Image                     | ifm3d Macro  | Value | Description                                                                       |
|--------------------------------|--------------|-------|-----------------------------------------------------------------------------------|
| Radial distance image          | IMG_RDIS     | 1     | Image pixel shows the radial distance  of the object considering camera as center |
| Amplitude image                | IMG_AMP      | 2     | Normalized mmplitude image from the camera                                        |
| Raw amplitude image            | IMG_RAMP     | 4     | Raw Amplitude Image                                                               |
| Cartesian coordinate           | IMG_CART     | 8     | Cartesian coordinate (X,Y,Z) at each pixel considering camera at (0,0,0)          |
| Uint vector                    | IMG_UVEC     | 16    | 3 channel Image showing unit vector at each pixel                                 |
| Exposure time                  | EXP_TIME     | 32    | Time to capture phase data for a frame                                            |
| Grayscale Image                | IMG_GRAY     | 64    | Ambient light image                                                               |
| Illumination temperature       | ILLU_TEMP    | 128   | Temperature of the illumination unit of the camera                                |
| Internal calibration           | INTR_CAL     | 256   | Internal calibration parameters                                                   |
| Inverse internal  calibration  | INV_INTR_CAL | 512   | Inverse Internal calibration parameters                                           |
| JSON model                     | JSON_MODEL   | 1024  | Application output in JSON format                                                 |
| Distance Noise Image           | IMG_DIS_NOISE| 2048  | Distance noise image                                                              |

Following code shows the configuration of the `FrameGrabber` object for radial distance and amplitude image

```cpp
auto cam = std::make_shared<ifm3d::Camera>();
auto fg = std::make_shared<ifm3d::FramGrabber>(cam,ifm3d::IMG_RDIS|ifm3d::IMG_AMP);
auto img = std::make_shared<ifm3d::ImageBuffer>();

if (! fg->WaitForFrame(img.get(), 1000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }

auto amp = img->AmplitudeImage();
auto rdist = img->DistanceImage();

```
Passing `ifm3d::IMG_RDIS|ifm3d::IMG_AMP` as a parameter to `FrameGrabber` will configure the
device connection for only radial distance and amplitude image. to obtain these images
call corresponding funtion on container buffers as shown in code snippet. Calling
a function for data or image which is not configured through schema will result in empty buffers.

Default value of schema is set to `ifm3d::IMG_AMP|ifm3d::IMG_CART` and can be override by
passing the schema value to `Framegrabber` or by setting the env variable `IFM3D_MASK`.

To get the schema value for a combination of data/image add the decimal values of the required data/Images
and set the value to env variable IFM3D_MASK.

Setting a mask of Radial distance, Amplitude, Cartesian Image ( 1+ 2+ 8 = 11)

```
on windows

> set IFM3D_MASK = 11

on linux

$ export IFM3D_MASK = 11
```

User passed schema mask to 'FrameGrabber' will take higest priority over `IFM3D_MASK`
and `default value` whereas `IFM3D_MASK` will take priority over `default value`.
