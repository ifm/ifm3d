# Buffer types

`ifm3d` supports multiple device families (O3R, O3D3xx, O3X1xx).

Each device natively provides different types of data, and the `ifm3d` library adds additional buffers for convenience.

The tables below list all available buffer types and which device they apply to. 
We also specify if the data is provided directly by the device, meaning it is streamed from the embedded device, or calculated by `ifm3d` API on the receiver device. 
This is important information to be considered when trying to work around bandwidth limitations and delays.

In this tables, the NA abbreviation stands for "Not Applicable."

## O3R Available buffers

| `buffer_id`                     | Description                                                                                                                                                                                                        | Native or calculated in ifm3d |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------- |
| `ALGO_DEBUG`                    | Proprietary data used for troubleshooting by ifm. For internal use only.                                                                                                                                           | Native                        |
| `CONFIDENCE_IMAGE`              | 2D image containing information about each pixel status. Only available for the 3D ports.                                                                                                                          | Native                        |
| `EXTRINSIC_CALIB`               | Current values of the optical system to user coordinate transformation for the port currently targeted by the `FrameGrabber`. Only available for the 3D port.<br>Can also be retrieved with the `TOF_INFO` buffer. | Calculated                    |
| `INTRINSIC_CALIB`               | The intrinsic parameters for the port or camera currently targeted by the`FrameGrabber`. Only available for the 3D port.     <br>Can also be retrieved with the `TOF_INFO` buffer.                                 | Native                        |
| `INVERSE_INTRINSIC_CALIBRATION` | The inverse intrinsic parameters for the port or camera currently targeted by the`FrameGrabber`. <br>  Only available for the 3D port.     <br>Can also be retrieved with the `TOF_INFO`                           | Native                        |
| `JPEG_IMAGE`                    | The 2D RGB image.                                                                                                                                                                                                  | Native                        |
| `NORM_AMPLITUDE_IMAGE`          | The normalized amplitude image.                                                                                                                                                                                    | Native                        |
| `O3R_ODS_INFO`                  | The zones output of ODS. Only available for devices with an ODS license.                                                                                                                                           | Native                        |
| `O3R_ODS_OCCUPANCY_GRID`        | The occupancy grid output of ODS. Only available for devices with an ODS license.                                                                                                                                  | Native                        |
| `O3R_RESULT_ARRAY2D`            | Contains the result for O3R applications like PDS and MCC.                                                                                                                                                         | Native                        |
| `O3R_RESULT_IMU`                | The IMU data for the OVP on-board IMU at port 6.                                                                                                                                                                   | Native                        |
| `O3R_RESULT_JSON`               | Contains the result for O3R applications like PDS and MCC.                                                                                                                                                         | Native                        |
| `RADIAL_DISTANCE_IMAGE`         | 2D image containing the radial distance value for each pixel.                                                                                                                                                      | Native                        |
| `RADIAL_DISTANCE_NOISE`         | 2D image containing the distance noise value for each pixel.                                                                                                                                                       | Native                        |
| `REFLECTIVITY`                  | 2D image containing the estimated IR reflectivity (in percent). Only available for the 3D ports.                                                                                                                   | Native                        |
| `RGB_INFO`                      | Contains various information about the RGB camera and the latest RGB frame (see[`RGBInfoV1`](https://api.ifm3d.com/stable/_autosummary/ifm3dpy.deserialize.RGBInfoV1.html))                                        | Native                        |
| `TOF_INFO`                      | Contains various information about the TOF camera and the latest TOF frame (see[`TOFInfoV4`](https://api.ifm3d.com/stable/_autosummary/ifm3dpy.deserialize.TOFInfoV4.html))                                        | Native                        |
| `XYZ`                           | The point cloud in Cartesian coordinates.                                                                                                                                                                          | Calculated                    |


## O3D Available buffers

| `buffer_id`                     | Description                                                                                                                   | Native or calculated in ifm3d |
| ------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- | ----------------------------- |
| `AMPLITUDE_IMAGE`               | 2D image containing amplitude information for each pixel.                                                                     | Native                        |
| `CARTESIAN_X_COMPONENT`         | Cartesian coordinate along X axis.                                                                                            | Calculated                    |
| `CARTESIAN_Y_COMPONENT`         | Cartesian coordinate along Y axis.                                                                                            | Calculated                    |
| `CARTESIAN_Z_COMPONENT`         | Cartesian coordinate along Z axis.                                                                                            | Calculated                    |
| `CONFIDENCE_IMAGE`              | 2D image containing information about each pixel status.                                                                      | Native                        |
| `DIAGNOSTIC`                    | Provides some diagnostic information. See [details below](#diagnostic-buffer).                                                | Native                        |
| `EXPOSURE_TIME`                 | Exposure times used for the last frame.                                                                                       | Calculated                    |
| `EXTRINSIC_CALIB`               | Current values of the optical system to user coordinate transformation for the port currently targeted by the `FrameGrabber`. | Calculated                    |
| `ILLUMINATION_TEMP`             | The temperature of the illumination module                                                                                    | Native                        |
| `INTRINSIC_CALIB`               | The intrinsic parameters for the port or camera currently targeted by the`FrameGrabber`.                                      | Native                        |
| `INVERSE_INTRINSIC_CALIBRATION` | The inverse intrinsic parameters for the port or camera currently targeted by the`FrameGrabber`.                              | Native                        |
| `JSON_DIAGNOSTIC`               | Diagnostic information in JSON format.                                                                                        | Native                        |
| `JSON_MODEL`                    | For internal use only.                                                                                                        | ??                            |
| `NORM_AMPLITUDE_IMAGE`          | The normalized amplitude image.                                                                                               | Native                        |
| `RADIAL_DISTANCE_IMAGE`         | 2D image containing the radial distance value for each pixel.                                                                 | Native                        |
| `UNIT_VECTOR_ALL`               | Unit vectors that can be used to calculate the point cloud directly from the distance image.                                  | Native                        |
| `XYZ`                           | The point cloud in Cartesian coordinates.                                                                                     | Calculated                    |

:::{note}
The `CARTESIAN_ALL` buffer is deprecated, and the `XYZ` buffer should be used instead. 
:::


### `DIAGNOSTIC` buffer

This buffer provides diagnostic information such as temperature, frame duration and framerate.

To unpack the information in the `DIAGNOSTIC` buffer, use the following structure:


| Content                                                                                                             | Type                    | Size    |
| ------------------------------------------------------------------------------------------------------------------- | ----------------------- | ------- |
| Illumination temperature                                                                                            | 32-bit signed int       | 4 bytes |
| Frontend Temperature 1                                                                                              | 32-bit signed int       | 4 bytes |
| Frontend Temperature 2                                                                                              | 32-bit signed int       | 4 bytes |
| i.mx6 Temperature                                                                                                   | 32-bit signed int       | 4 bytes |
| Frame duration: time between the last trigger and the readiness for a new trigger. Only relevant in triggered mode. | 32-bit unsigned integer | 4 bytes |
| Framerate. Only relevant in continuous mode (also called free running mode).                                        | 32-bit unsigned integer | 4 bytes |

:::{note}
All the temperatures are displayed in 0.1 °C.
Invalid temperatures have the value 0x7FFF (32767).
Some temperatures are only measured for specific part numbers, which might be why they are invalid.
:::

In Python, the `DIAGNOSTIC` data can be unpacked with:

```python
# Necessary imports
import struct
import time
from ifm3dpy.device import O3D
from ifm3dpy.framegrabber import FrameGrabber, buffer_id

# Create the objects
o3d = O3D() #EDIT here when using a non-default IP address
fg = FrameGrabber(o3d)

#Collect a frame
fg.start([buffer_id.DIAGNOSTIC,])
time.sleep(5) # Grace period after initialization of the data stream
[ok, frame] = fg.wait_for_frame().wait_for(500)

# Get the data from the collected frame 
data = frame.get_buffer(buffer_id.DIAGNOSTIC)
# Unpack the data 
unpacked_data = struct.unpack('<4i2I', data)

fg.stop()
```

## O3X Available buffers

| `buffer_id`             | Description                                                   | Native or calculated in ifm3d |
| ----------------------- | ------------------------------------------------------------- | ----------------------------- |
| `CARTESIAN_X_COMPONENT` | Cartesian coordinate along X axis.                            | Calculated                    |
| `CARTESIAN_Y_COMPONENT` | Cartesian coordinate along Y axis.                            | Calculated                    |
| `CARTESIAN_Z_COMPONENT` | Cartesian coordinate along Z axis.                            | Calculated                    |
| `CONFIDENCE_IMAGE`      | 2D image containing information about each pixel status.      | Native                        |
| `NORM_AMPLITUDE_IMAGE`  | The normalized amplitude image.                               | Native                        |
| `RADIAL_DISTANCE_IMAGE` | 2D image containing the radial distance value for each pixel. | Native                        |
| `RADIAL_DISTANCE_NOISE` | 2D image containing the distance noise value for each pixel.  | Native                        |
| `XYZ`                   | The point cloud in Cartesian coordinates.                     | Calculated                    |

:::{note}
Even though the `GRAYSCALE_IMAGE` buffer is shown to be available in the O3X JSON configuration, no acquisition mode currently provides this buffer, so we have not included it in the list.
:::

## Data formats

`ifm3d` supports multiple devices. These devices support multiple buffers and data types which can have different data formats on different devices.

We recommend to retrieve the format of a specific `buffer_id` programmatically.
One can use the code below (adapt to the specific `buffer_id`):
:::::{tabs}
::::{group-tab} C++

```cpp
ifm3d::Buffer xyz = frame->GetBuffer(ifm3d::buffer_id::XYZ);

// Query the data format and channel.
std::cout << xyz.nchannels() << std::endl;
std::cout << static_cast<int>(xyz.dataFormat()) << std::endl;
```

::::
::::{group-tab} Python

```python
print(type(frame.get_buffer(buffer_id.XYZ)))
print(np.shape(frame.get_buffer(buffer_id.XYZ)))
print(frame.get_buffer(buffer_id.RADIAL_DISTANCE_NOISE).dtype)
```

::::
:::::

In C++, one has to refer to the pixel format correspondence defined below:

```
FORMAT_8U = 0,
FORMAT_8S = 1,
FORMAT_16U = 2,
FORMAT_16S = 3,
FORMAT_32U = 4,
FORMAT_32S = 5,
FORMAT_32F = 6,
FORMAT_64U = 7,
FORMAT_64F = 8,
FORMAT_16U2 = 9,
FORMAT_32F3 = 10
```

For instance, the format returned for `ifm3d::buffer_id::XYZ` is `2`, which corresponds to `FORMAT_32F` (32 bit float).

See the full examples in the `ifm3d-examples` repository, [in Python](https://github.com/ifm/ifm3d-examples/blob/main/common/python/get_data_format.py), and [in C++](https://github.com/ifm/ifm3d-examples/blob/main/common/cpp/get_data_format.cpp).