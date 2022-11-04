## Device, Schema and data types for ifm3d::Buffer

```ifm3d``` supports multiple devices. 
To see the list of available buffer types, refer to [this](https://ifm3d.com/sphinx-doc/build/html/ifm3d/doc/sphinx/cpp_api/frame_8h.html#a363ee802dc7953150dc23ba56d7e9c50) section of the API documentation.

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

In C++, one has to refer to the pixel format correspondence defined in the API [here](ADDLINK). For instance, the format returned for `ifm3d::buffer_id::XYZ` is `2`, which corresponds to `FORMAT_32F` (32 bit float).