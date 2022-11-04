## Device, Schema and data types for ifm3d::Buffer

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
