# Buffer types

`ifm3d` supports multiple devices (O3R, O3D, O3X).

Each device provides different types of data natively, and the `ifm3d` library add extra buffers for convenience.

## Available buffers

The table below lists all available buffer types and which device they apply to. 
We also specify if the data is provided directly by the device or calculated by `ifm3d`. This is important information when trying to work around bandwidth limitations.

In this table, the NA abbreviation stands for "Not Applicable."

| `buffer_id`                     | Description                                                                                                                                                                        | O3R                    | O3D                                                                                                    | O3X               | Native or calculated in ifm3d |
| ------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------- | ------------------------------------------------------------------------------------------------------ | ----------------- | ----------------------------- |
| `ALGO_DEBUG`                    | Proprietary data used for troubleshooting by ifm. For internal use only.                                                                                                           | Yes                    | NA                                                                                                     | NA                | Native                        |
| `AMPLITUDE_IMAGE`               | 2D image containing amplitude information for each pixel.                                                                                                                          | NA                     | Yes                                                                                                    | NA                | Native                        |
| `CARTESIAN_ALL`                 | This buffer is deprecated and the `XYZ` buffer should be used instead.                                                                                                             | NA                     | Not working: should work? A buffer with the requested `buffer_id` is not available.: `buffer_id`: 203      | NA                | Calculated                    |
| `CARTESIAN_X_COMPONENT`         | Cartesian coordinate along X axis.                                                                                                                                                 | NA                     | Yes                                                                                                    | Yes               | Calculated                    |
| `CARTESIAN_Y_COMPONENT`         | Cartesian coordinate along Y axis.                                                                                                                                                 | NA                     | Yes                                                                                                    | Yes               | Calculated                    |
| `CARTESIAN_Z_COMPONENT`         | Cartesian coordinate along Z axis.                                                                                                                                                 | NA                     | Yes                                                                                                    | Yes               | Calculated                    |
| `CONFIDENCE_IMAGE`              | 2D image containing information about each pixel status.                                                                                                                           | Yes - Only 3D          | Yes                                                                                                    | Yes               | Native                        |
| `DIAGNOSTIC`                    | Provides some diagnostic information. See [details below](#diagnostic-buffer).                                                                                                     | NA                     | Yes                                                                                                    | NA                | Native                        |
| `EXPOSURE_TIME`                 | Exposure times used for the last frame.                                                                                                                                            | NA                     | Yes                                                                                                    | NA                | Calculated                    |
| `EXTRINSIC_CALIB`               | Current values of the optical system to user coordinate transformation for the port currently targeted by the`FrameGrabber`. <br>Can also be retrieved with the `TOF_INFO` buffer. | Yes - Only 3D          | Yes                                                                                                    | NA                | Calculated                    |
| `GRAYSCALE_IMAGE`               | ??                                                                                                                                                                                 | NA                     | NA                                                                                                     | NA - Should work? | ??                            |
| `INTRINSIC_CALIB`               | The intrinsic parameters for the port or camera currently targeted by the`FrameGrabber`.                                                                                           | Yes - Only 3D          | Yes                                                                                                    | NA                | Native                        |
| `INVERSE_INTRINSIC_CALIBRATION` | The inverse intrinsic parameters for the port or camera currently targeted by the`FrameGrabber`. <br>                                                                              | Yes - Only 3D          | Yes                                                                                                    | NA                | Native                        |
| `JPEG_IMAGE`                    | The 2D RGB image.                                                                                                                                                                  | Yes - Only RGB         | NA                                                                                                     | NA                | Native                        |
| `JSON_DIAGNOSTIC`               | Diagnostic information in JSON format.                                                                                                                                             | NA                     | Not working. Should work? Lib: A buffer with the requested `buffer_id` is not available.: `buffer_id`: 305 | NA                | Native                        |
| `JSON_MODEL`                    | For internal use only.                                                                                                                                                             | NA                     | Yes                                                                                                    | NA                | ??                            |
| `MONOCHROM_2D`                  | For O2I devices only.                                                                                                                                                              | NA                     | NA                                                                                                     | NA                | NA                            |
| `MONOCHROM_2D_12BIT`            | For O2I devices only.                                                                                                                                                              | NA                     | NA                                                                                                     | NA                | NA                            |
| `NORM_AMPLITUDE_IMAGE`          | The normalized amplitude image                                                                                                                                                     | Yes - Only 3D          | Yes                                                                                                    | Yes               | Native                        |
| `O3R_ODS_INFO`                  | The zones output of ODS.                                                                                                                                                           | Yes - Only ODS         | NA                                                                                                     | NA                | Native                        |
| `O3R_ODS_OCCUPANCY_GRID`        | The occupancy grid output of ODS.                                                                                                                                                  | Yes - Only ODS         | NA                                                                                                     | NA                | Native                        |
| `O3R_RESULT_ARRAY2D`            | Contains the result for O3R applications like PDS and MCC.                                                                                                                         | Yes - Only PDS and MCC | NA                                                                                                     | NA                | Native                        |
| `O3R_RESULT_IMU`                | The IMU data for the OVP on-board IMU at port 6.                                                                                                                                   | Yes - Only IMU port    | NA                                                                                                     | NA                | Native                        |
| `O3R_RESULT_JSON`               | Contains the result for O3R applications like PDS and MCC.                                                                                                                         | Yes - Only PDS and MCC | NA                                                                                                     | NA                | Native                        |
| `RADIAL_DISTANCE_IMAGE`         | 2D image containing the radial distance value for each pixel.                                                                                                                      | Yes - Only 3D          | Yes                                                                                                    | Yes               | Native                        |
| `RADIAL_DISTANCE_NOISE`         | 2D image containing the distance noise value for each pixel.                                                                                                                       | Yes - Only 3D          | NA                                                                                                     | Yes               | Native                        |
| `REFLECTIVITY`                  | 2D image containing the estimated IR reflectivity (in percent).                                                                                                                    | Yes - Only 3D          | NA                                                                                                     | NA                | Native                        |
| `RGB_INFO`                      | Contains various information about the RGB camera and the latest RGB frame (see[`RGBInfoV1`](https://api.ifm3d.com/stable/_autosummary/ifm3dpy.deserialize.RGBInfoV1.html))        | Yes - Only RGB         | NA                                                                                                     | NA                | Native                        |
| `TOF_INFO`                      | Contains various information about the TOF camera and the latest TOF frame (see[`TOFInfoV4`](https://api.ifm3d.com/stable/_autosummary/ifm3dpy.deserialize.TOFInfoV4.html))        | Yes - Only 3D          | NA                                                                                                     | NA                | Native                        |
| `UNIT_VECTOR_ALL`               | Unit vectors that can be used to calculate the point cloud directly from the distance image.                                                                                       | NA                     | Error: Unsupported ifm3d::image type                                                                   | NA                | Native                        |
| `XYZ`                           | The point cloud in Cartesian coordinates.                                                                                                                                          | Yes - Only 3D          | Yes                                                                                                    | Yes               | Calculated                    |

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
