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

These devices provide multiple buffers/data types which can have different data formats.

To retrieve the data type for a specific buffer, one can use the code below (adapt to the specific `buffer_id`):
:::::{tabs}
::::{group-tab}
:::cpp
ifm3d::Buffer xyz = frame->GetBuffer(ifm3d::buffer_id::XYZ);

// Query the data format and channel.
std::cout << xyz.nchannels() << std::endl;
std::cout << static_cast<int>(xyz.dataFormat()) << std::endl;
:::
::::
::::{group-tab}
:::python
print(type(frame.get_buffer(buffer_id.XYZ)))
print(np.shape(frame.get_buffer(buffer_id.XYZ)))
print(frame.get_buffer(buffer_id.RADIAL_DISTANCE_NOISE).dtype)
:::
:::: 
:::::

In C++, one has to refer to the pixel format correspondence defined in the API [here](https://api.ifm3d.com/html/cpp_api/device_2device_8h_source.html#L72). For instance, the format returned for `ifm3d::buffer_id::XYZ` is `2`, which corresponds to `FORMAT_32F` (32 bit float).