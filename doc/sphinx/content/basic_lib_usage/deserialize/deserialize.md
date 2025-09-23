# How to: deserialize O3R data

When you acquire a frame from a FrameGrabber instance, the contents of each buffer depend on its `buffer_id`.

- Image buffers (e.g., `RADIAL_DISTANCE_IMAGE`, `NORM_AMPLITUDE_IMAGE`, `CONFIDENCE_IMAGE`, `JPEG_IMAGE`, `XYZ`) are delivered as numeric arrays (NumPy arrays in Python, std::vector/cv::Mat in C++). These can be used directly for computation and visualization.
- Metadata buffers (e.g., `TOF_INFO`, `RGB_INFO`, `O3R_ODS_INFO` etc.,) are delivered as serialized structs. To access their fields, you must deserialize them using the appropriate deserialize class.

## Buffer IDs with respective deserialize class

| **Buffer ID**                              | **Deserializer Class**                                                                                                                                                                              | **Description**                                                                |
| ------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| `TOF_INFO`                                 | [TofInfoV4](https://api.ifm3d.com/html/_autosummary/ifm3dpy.deserialize.TOFInfoV4.html#ifm3dpy.deserialize.TOFInfoV4)                                                                               | Metadata for ToF images: resolutions, exposures, timestamps, calibration, etc. |
| `RGB_INFO`                                 | [RGBInfoV1](https://api.ifm3d.com/html/_autosummary/ifm3dpy.deserialize.RGBInfoV1.html#ifm3dpy.deserialize.RGBInfoV1)                                                                               | Metadata for ToF images: resolutions, exposures, timestamps, calibration, etc. |
| `O3R_ODS_INFO`                             | [ODSInfoV1](https://api.ifm3d.com/html/_autosummary/ifm3dpy.deserialize.ODSInfoV1.html#ifm3dpy.deserialize.ODSInfoV1)                                                                               | Zone ID and zone information for Object Detection.                             |
| `O3R_ODS_OCCUPANCY_GRID`                   | [ODSOccupancyGridV1](https://api.ifm3d.com/html/_autosummary/ifm3dpy.deserialize.ODSOccupancyGridV1.html#ifm3dpy.deserialize.ODSOccupancyGridV1)                                                    | Occupancy grid data with transformation matrix.                                |
| `O3R_ODS_POLAR_OCC_GRID`                   | [ODSPolarOccupancyGridV1](https://api.ifm3d.com/html/_autosummary/ifm3dpy.deserialize.ODSPolarOccupancyGridV1.html#ifm3dpy.deserialize.ODSPolarOccupancyGridV1)                                     | A compressed version of the grid using polar coordinates                       |
| `O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION` | [ODSExtrinsicCalibrationCorrectionV1](https://api.ifm3d.com/html/_autosummary/ifm3dpy.deserialize.ODSExtrinsicCalibrationCorrectionV1.html#ifm3dpy.deserialize.ODSExtrinsicCalibrationCorrectionV1) | Extrinsic calibration correction parameters estimated by ODS application.      |

For more information on the available deserializer classes for managing different structs of other ifm devices like `O3D`, `O3X`, please refer to the [Python API documentation](https://api.ifm3d.com/latest/_autosummary/ifm3dpy.deserialize.html) or the [C++ API documentation](https://api.ifm3d.com/html/cpp_api/annotated.html).

The usage of the deserializer is the same for all the buffers mentioned above: create the object, and call the deserialize function. Follow the example below for an example on deserializing the `TOFInfoV4` buffer received from `buffer_id` - `TOF_INFO`.

:::::{tabs}
:::: {group-tab} Python
:::{literalinclude} ../../ifm3d-examples/ovp8xx/python/core/deserialize_tof_info.py
:language: python
:::
::::
:::: {group-tab} C++
:::{literalinclude} ../../ifm3d-examples/ovp8xx/cpp/core/deserialize/deserialize_tof_info.cpp
:language: cpp
:::
::::
:::::