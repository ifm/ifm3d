# Buffer types

```ifm3d``` supports multiple devices (O3R, O3D, O3X).

Each device provides different types of data natively, and the `ifm3d` library add extra buffers for convenience.

## Available buffers


| `buffer_id`                     | Description                                                                                                                                                                        | O3R                        | O3D                                                                                               | O3X | Native or calculated in ifm3d |
| --------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------- | --------------------------------------------------------------------------------------------------- | ----- | ------------------------------- |
| `ALGO_DEBUG`                    | Proprietary data used for troubleshooting by ifm.                                                                                                                                  | Not working: should work?  | Not working: should work?                                                                         | ??  | Native                        |
| `AMPLITUDE_IMAGE`               | 2D image containing amplitude information for each pixel.                                                                                                                          | NA                         | Yes                                                                                               | ??  | ??                            |
| `CARTESIAN_ALL`                 | Cartesian coordinates or each point in the point cloud.                                                                                                                            | NA                         | Not working: should work? A buffer with the requested buffer_id is not available.: buffer_id: 203 | ??  | Calculated                    |
| `CARTESIAN_X_COMPONENT`         | Cartesian coordinate along X axis.                                                                                                                                                 | NA                         | Yes                                                                                               | ??  | Calculated                    |
| `CARTESIAN_Y_COMPONENT`         | Cartesian coordinate along Y axis.                                                                                                                                                 | NA                         | Yes                                                                                               | ??  | Calculated                    |
| `CARTESIAN_Z_COMPONENT`         | Cartesian coordinate along Z axis.                                                                                                                                                 | NA                         | Yes                                                                                               | ??  | Calculated                    |
| `CONFIDENCE_IMAGE`              | 2D image containing information about each pixel status.                                                                                                                           | x                          | Yes                                                                                               | ??  | Native                        |
| `DIAGNOSTIC`                    | ??                                                                                                                                                                                 | NA                         | Yes                                                                                               | ??  | ??                            |
| `EXPOSURE_TIME`                 | Exposure times used for the last frame.                                                                                                                                            | Not working - should work? | Yes                                                                                               | ??  | ??                            |
| `EXTRINSIC_CALIB`               | Current values of the optical system to user coordinate transformation for the port currently targeted by the`FrameGrabber`. <br>Can also be retrieved with the `TOF_INFO` buffer. | x                          | Yes                                                                                               | ??  | Calculated                    |
| `GRAYSCALE_IMAGE`               | ??                                                                                                                                                                                 | NA                         | NA                                                                                                | ??  | ??                            |
| `INTRINSIC_CALIB`               | The intrinsic parameters for the port currently targeted by the`FrameGrabber`. <br>Can also be retrieved with the `TOF_INFO` buffer.                                               | x                          | Yes                                                                                               | ??  | Native                        |
| `INVERSE_INTRINSIC_CALIBRATION` | The inverse intrinsic parameters for the port currently targeted by the`FrameGrabber`. <br>Can also be retrieved with the `TOF_INFO` buffer.                                       | x                          | Yes                                                                                               | ??  | Native                        |
| `JPEG_IMAGE`                    | The 2D RGB image.                                                                                                                                                                  | x                          | NA                                                                                                | NA  | Native                        |
| `JSON_DIAGNOSTIC`               | ??                                                                                                                                                                                 | ??                         | NA                                                                                                | ??  | ??                            |
| `JSON_MODEL`                    | ??                                                                                                                                                                                 | NA                         | Yes                                                                                               | NA  | ??                            |
| `MONOCHROM_2D`                  | ??                                                                                                                                                                                 | NA                         | NA                                                                                                | ??  | ??                            |
| `MONOCHROM_2D_12BIT`            | ??                                                                                                                                                                                 | NA                         | NA                                                                                                | ??  | ??                            |
| `NORM_AMPLITUDE_IMAGE`          | The normalized amplitude image                                                                                                                                                     | x                          | Yes                                                                                               | x   |                               |
| `O3R_ODS_INFO`                  | The zones output of ODS.                                                                                                                                                           | x                          | NA                                                                                                | NA  | Native                        |
| `O3R_ODS_OCCUPANCY_GRID`        | The occupancy grid output of ODS.                                                                                                                                                  | x                          | NA                                                                                                | NA  | Native                        |
| `O3R_RESULT_ARRAY2D`            | Contains the result for O3R applications like PDS and MCC.                                                                                                                         | x                          | NA                                                                                                | NA  | Native                        |
| `O3R_RESULT_IMU`                | The IMU data for the OVP on-board IMU at port 6.                                                                                                                                   | x                          | NA                                                                                                | NA  | Native                        |
| `O3R_RESULT_JSON`               | Contains the result for O3R applications like PDS and MCC.                                                                                                                         | x                          | NA                                                                                                | NA  | Native                        |
| `RADIAL_DISTANCE_IMAGE`         | 2D image containing the radial distance value for each pixel.                                                                                                                      | x                          | Yes                                                                                               | ??  | Native                        |
| `RADIAL_DISTANCE_NOISE`         | 2D image containing the distance noise value for each pixel.                                                                                                                       | x                          | NA                                                                                                | NA  | Native                        |
| `REFLECTIVITY`                  | 2D image containing the estimated IR reflectivity (in percent).                                                                                                                    | x                          | NA                                                                                                | NA  | Native                        |
| `RGB_INFO`                      | Contains various information about the RGB camera and the latest RGB frame (see[`RGBInfoV1`](ADDLINK))                                                                             | x                          | NA                                                                                                | NA  | Native                        |
| `TOF_INFO`                      | Contains various information about the TOF camera and the latest TOF frame (see[`TOFInfoV4`](ADDLINK))                                                                             | x                          | NA                                                                                                | NA  | Native                        |
| `UNIT_VECTOR_ALL`               | Unit vectors that can be used to calculate the point cloud directly from the distance image.                                                                                       | NA                         | Error: Unsupported ifm3d::image type                                                              | NA  | Native                        |
| `XYZ`                           | The point cloud in Cartesian coordinates.                                                                                                                                          | x                          | Yes                                                                                               | NA  | Calculated                    |

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
