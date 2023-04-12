# How to: deserialize O3R data

Some of the data provided by the O3R platform needs to be deserialized to be used. This is the case for:
- the intrinsic calibration parameters (`ifm3dpy.deserialize.Calibration`), which provides details like which optical model is used (Fisheye, pinhole) and the values for each of the model's parameters,
- the extrinsic calibration (optics to user) parameters (` ifm3dpy.deserialize.ExtrinsicOpticToUser`), which provides the transformations between the optical system and the reference point on the camera housing,
- the ODS zone information (`ifm3dpy.deserialize.ODSInfoV1`), which contains the zone id being used and the occupancy of the zones,
- the ODS occupancy grid information (`ifm3dpy.deserialize.ODSOccupancyGridV1`), which contains occupancy grid data and the transormation matrix,
- the RGB information (`ifm3dpy.deserialize.RGBInfoV1`), which provides exposure times and calibration parameters for the O3R RGB cameras.

For more information on the data structures of each buffer please refer to the [python API documentation](https://api.ifm3d.com/latest/_autosummary/ifm3dpy.deserialize.html) or the [c++ API documentation].

The usage of the deserializer is the same for all the buffers mentioned above: create the object, and call the deserlize function. Follow the example below for an example on deserialializing the `RGBInfoV1` buffer.

:::::{tabs}
:::: {group-tab} Python
:::{literalinclude} deserialize.py
:language: python
:::
::::
:::: {group-tab} C++
:::{literalinclude} deserialize.cpp
:language: cpp
:::
::::
:::::