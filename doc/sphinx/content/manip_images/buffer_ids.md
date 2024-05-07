# Buffer types

```ifm3d``` supports multiple devices (O3R, O3D, O3X). 

Each device provides different types of data natively, and the `ifm3d` library add extra buffers for convenience.

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

```ifm3d``` supports multiple devices. These devices support multiple buffers/data types which can have different data formats on different devices.
The following table summarizes the buffers formats, along with their [ifm3d::buffer_id](../../../modules/framegrabber/include/ifm3d/fg#L22), so a ```std::set<ifm3d::buffer_id>``` needs to be pass in ```FrameGrabber::Start``` method to enable the corresponding buffer for grabbing.
 

| Data/Image                              | ifm3d schema                                    | O3D3XX             | O3X           | O3R                |
|-----------------------------------------|-------------------------------------------------|--------------------|---------------|--------------------|
| Radial distance image                   | ifm3d::buffer_id::RADIAL_DISTANCE               | std::uint16_t      | float         | float              |
| Amplitude image                         | ifm3d::buffer_id::AMPLITUDE                     | std::uint16_t      | float         | float              |
| Raw amplitude image                     | ifm3d::buffer_id::RAW_AMPLITUDE                 | std::uint16_t      | float         | NA                 |
| Cartesian coordinate                    | ifm3d::buffer_id::XYZ                           | std::int16_t       | float         | float              |
| Unit vector                             | ifm3d::buffer_id::UNIT_VECTOR_ALL               | float              | float         | NA                 |
| Image grayscale                         | ifm3d::buffer_id::GRAY                          | std::uint16_t      | float         | NA                 |
| Distance Noise Image                    | ifm3d::buffer_id::DISTANCE_NOISE                | NA                 | std::uint16_t | std::uint16_t      |
| Confidence Image                        | ifm3d::buffer_id::CONFIDENCE                    | std::uint8_t       | std::uint8_t  | std::unit16_t      |
| JPEG                                    | ifm3d::buffer_id::IMG_JPEG                      | NA                 | NA            | std::unit8_t       |

Along with above data ifm3d also supports [ifm3d::buffer_id](../../../modules/framegrabber/include/ifm3d/fg#L22) values, which is used to obtain data that are obtained in standard STL containers or C++ default types.

| Data/Image                              | ifm3d schema                                    | O3D3XX                           | O3X                              | O3R                              |
|-----------------------------------------|-------------------------------------------------|----------------------------------|----------------------------------|----------------------------------|
| JSON Model Data                         | ifm3d::buffer_id::JSON_MODEL                    | std::string        | NA            | NA                 |
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