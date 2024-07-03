# Buffer types

```ifm3d``` supports multiple devices (O3R, O3D, O3X). 

Each device provides different types of data natively, and the `ifm3d` library add extra buffers for convenience.

## Available buffers

| `buffer_id`                     | Description                                                                                                                                                                         | O3R | O3D | O3X | Native or calculated in ifm3d |
| ------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --- | --- | --- | ----------------------------- |
| `ALGO_DEBUG`                    | Proprietary data used for troubleshooting by ifm.                                                                                                                                   | Not working: should work?  | ??  | ??  | Native                        |
| `AMPLITUDE_IMAGE`               | 2D image containing amplitude information for each pixel.                                                                                                                           | ??  | ??  | ??  | ??                            |
| `CARTESIAN_ALL`                 | Cartesian coordinates or each point in the point cloud.                                                                                                                             | NA  | x   | x   | Calculated                    |
| `CARTESIAN_X_COMPONENT`         | Cartesian coordinate along X axis.                                                                                                                                                  | NA  | x   | x   | Calculated                    |
| `CARTESIAN_Y_COMPONENT`         | Cartesian coordinate along Y axis.                                                                                                                                                  | NA  | x   | x   | Calculated                    |
| `CARTESIAN_Z_COMPONENT`         | Cartesian coordinate along Z axis.                                                                                                                                                  | NA  | x   | x   | Calculated                    |
| `CONFIDENCE_IMAGE`              | 2D image containing information about each pixel status.                                                                                                                            | x   | x   | x   | Native                        |
| `DIAGNOSTIC`                    | ??                                                                                                                                                                                  | NA  | ??  | ??  | ??                            |
| `EXPOSURE_TIME`                 | Exposure times used for the last frame.                                                                                                                                             | ??  | Not working - should work?   | ??  | ??                            |
| `EXTRINSIC_CALIB`               | Current values of the optical system to user coordinate transformation for the port currently targeted by the `FrameGrabber`. <br>Can also be retrieved with the `TOF_INFO` buffer. | x   | ??  | ??  | Calculated                    |
| `GRAYSCALE_IMAGE`               | ??                                                                                                                                                                                  | NA  | ??  | ??  | ??                            |
| `ILLUMINATION_TEMPERATURE` | Temperature of the illumination module. | NA | ?? | ?? | ?? |
| `INTRINSIC_CALIB`               | The intrinsic parameters for the port currently targeted by the `FrameGrabber`. <br>Can also be retrieved with the `TOF_INFO` buffer.                                               | x   | ??  | ??  | Native                        |
| `INVERSE_INTRINSIC_CALIBRATION` | The inverse intrinsic parameters for the port currently targeted by the `FrameGrabber`. <br>Can also be retrieved with the `TOF_INFO` buffer.                                       | x   | ??  | ??  | Native                        |
| `JPEG_IMAGE`                    | The 2D RGB image.                                                                                                                                                                   | x   | NA  | NA  | Native                        |
| `JSON_DIAGNOSTIC`               | ??                                                                                                                                                                                  | ??  | ??  | ??  | ??                            |
| `JSON_MODEL`                    | ??                                                                                                                                                                                  | NA  | x   | NA  | ??                            |
| `MONOCHROM_2D`                  | ??                                                                                                                                                                                  | ??  | ??  | ??  | ??                            |
| `MONOCHROM_2D_12BIT`            | ??                                                                                                                                                                                  | ??  | ??  | ??  | ??                            |
| `NORM_AMPLITUDE_IMAGE`          | The normalized amplitude image                                                                                                                                                      | x   | x   | x   |
| `O3R_ODS_INFO`                  | The zones output of ODS.                                                                                                                                                            | x   | NA  | NA  | Native                        |
| `O3R_ODS_OCCUPANCY_GRID`        | The occupancy grid output of ODS.                                                                                                                                                   | x   | NA  | NA  | Native                        |
| `O3R_RESULT_ARRAY2D`            | Contains the result for O3R applications like PDS and MCC.                                                                                                                          | x   | NA  | NA  | Native                        |
| `O3R_RESULT_IMU`                | The IMU data for the OVP on-board IMU at port 6.                                                                                                                                    | x   | NA  | NA  | Native                        |
| `O3R_RESULT_JSON`               | Contains the result for O3R applications like PDS and MCC.                                                                                                                          | x   | NA  | NA  | Native                        |
| `RADIAL_DISTANCE_IMAGE`         | 2D image containing the radial distance value for each pixel.                                                                                                                       | x   | x   | ??  | Native                        |
| `RADIAL_DISTANCE_NOISE`         | 2D image containing the distance noise value for each pixel.                                                                                                                        | x   | NA  | NA  | Native                        |
| `REFLECTIVITY`                  | 2D image containing the estimated IR reflectivity (in percent).                                                                                                                     | x   | NA  | NA  | Native                        |
| `RGB_INFO`                      | Contains various information about the RGB camera and the latest RGB frame (see [`RGBInfoV1`](ADDLINK))                                                                             | x   | NA  | NA  | Native                        |
| `TOF_INFO`                      | Contains various information about the TOF camera and the latest TOF frame (see [`TOFInfoV4`](ADDLINK))                                                                             | x   | NA  | NA  | Native                        |
| `UNIT_VECTOR_ALL`               | Unit vectors that can be used to calculate the point cloud directly from the distance image.                                                                                        | NA  | x   | NA  | Native                        |
| `XYZ`                           | The point cloud in cartesian coordinates.                                                                                                                                           | x   | NA  | NA  | Calculated                    |

## Native device buffers

Retrieving the list of image buffers *natively* available from your device can be done through ifm3d. For example, for an O3R225 camera head connected to port 2 of an OVP800 platform:
```bash
$ ifm3d dump --ip 192.168.0.69 | jq .ports.port2.data.availablePCICOutput
[
  "TOF_INFO",
  "RADIAL_DISTANCE_NOISE",
  "RADIAL_DISTANCE_COMPRESSED",
  "REFLECTIVITY",
  "AMPLITUDE_COMPRESSED",
  "CONFIDENCE"
]
```
## ifm3d buffers

Additionally, ifm3d provides the following buffers:
```c++
ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE
ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE
ifm3d::buffer_id::XYZ
ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE
ifm3d::buffer_id::EXPOSURE_TIME
ifm3d::buffer_id::EXTRINSIC_CALIB
ifm3d::buffer_id::INTRINSIC_CALIBRATION
ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION
```
## Data formats

`ifm3d` supports multiple devices. These devices support multiple buffers and data types which can have different data formats on different devices.
The following table summarizes the buffers formats, along with their `ifm3d::buffer_id`, that the proper format can be used to unpack data received in a frame. 

| Data/Image            | ifm3d schema                      | O3D3XX        | O3X           | O3R           |
| --------------------- | --------------------------------- | ------------- | ------------- | ------------- |
| Radial distance image | ifm3d::buffer_id::RADIAL_DISTANCE | std::uint16_t | float         | float         |
| Amplitude image       | ifm3d::buffer_id::AMPLITUDE       | std::uint16_t | float         | float         |
| Raw amplitude image   | ifm3d::buffer_id::RAW_AMPLITUDE   | std::uint16_t | float         | NA            |
| Cartesian coordinate  | ifm3d::buffer_id::XYZ             | std::int16_t  | float         | float         |
| Unit vector           | ifm3d::buffer_id::UNIT_VECTOR_ALL | float         | float         | NA            |
| Image grayscale       | ifm3d::buffer_id::GRAY            | std::uint16_t | float         | NA            |
| Distance Noise Image  | ifm3d::buffer_id::DISTANCE_NOISE  | NA            | std::uint16_t | std::uint16_t |
| Confidence Image      | ifm3d::buffer_id::CONFIDENCE      | std::uint8_t  | std::uint8_t  | std::unit16_t |
| JPEG                  | ifm3d::buffer_id::IMG_JPEG        | NA            | NA            | std::unit8_t  |

Along with above data ifm3d also supports [ifm3d::buffer_id](../../../modules/framegrabber/include/ifm3d/fg#L22) values, which is used to obtain data that are obtained in standard STL containers or C++ default types.

| Data/Image                              | ifm3d schema                                    | O3D3XX                           | O3X                              | O3R                              |
| --------------------------------------- | ----------------------------------------------- | -------------------------------- | -------------------------------- | -------------------------------- |
| JSON Model Data                         | ifm3d::buffer_id::JSON_MODEL                    | std::string                      | NA                               | NA                               |
| JSON Model Data                         | ifm3d::buffer_id::JSON_MODEL                    | std::string                      | NA                               | NA                               |
| Intrinsic calibration parameter         | ifm3d::buffer_id::INTRINSIC_CALIBRATION         | std::vector<float>               | NA                               | std::vector<float>               |
| inverse Intrinsic calibration parameter | ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION | std::vector<float>               | NA                               | std::vector<float>               |
| Illumination temperature                | ifm3d::buffer_id::ILLUMINATION_TEMP             | float                            | NA                               | NA                               |
| Exposure time                           | ifm3d::buffer_id::EXPOSURE_TIME                 | std::vector\<ifm3d::TimePointT\> | std::vector\<ifm3d::TimePointT\> | std::vector\<ifm3d::TimePointT\> |


The format of a specific `buffer_id` can also be retrieve programmatically. 
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