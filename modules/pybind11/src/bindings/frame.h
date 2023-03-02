/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_FRAME
#define IFM3D_PYBIND_BINDING_FRAME

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

void
bind_frame(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::Frame, ifm3d::Frame::Ptr> frame(
    m,
    "Frame",
    R"(
      Represent a frame of data received from the the device.
    )"
  );

  py::enum_<ifm3d::buffer_id>(m, "buffer_id", "Enum: buffer_id available for use with the default Organizer.")
    .value("RADIAL_DISTANCE_IMAGE", ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE)    
    .value("NORM_AMPLITUDE_IMAGE", ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE)      
    .value("AMPLITUDE_IMAGE", ifm3d::buffer_id::AMPLITUDE_IMAGE)      
    .value("GRAYSCALE_IMAGE", ifm3d::buffer_id::GRAYSCALE_IMAGE)      
    .value("RADIAL_DISTANCE_NOISE", ifm3d::buffer_id::RADIAL_DISTANCE_NOISE)      
    .value("REFLECTIVITY", ifm3d::buffer_id::REFLECTIVITY)      
    .value("CARTESIAN_X_COMPONENT", ifm3d::buffer_id::CARTESIAN_X_COMPONENT)      
    .value("CARTESIAN_Y_COMPONENT", ifm3d::buffer_id::CARTESIAN_Y_COMPONENT)      
    .value("CARTESIAN_Z_COMPONENT", ifm3d::buffer_id::CARTESIAN_Z_COMPONENT)      
    .value("CARTESIAN_ALL", ifm3d::buffer_id::CARTESIAN_ALL)      
    .value("UNIT_VECTOR_ALL", ifm3d::buffer_id::UNIT_VECTOR_ALL)      
    .value("MONOCHROM_2D_12BIT", ifm3d::buffer_id::MONOCHROM_2D_12BIT)      
    .value("MONOCHROM_2D", ifm3d::buffer_id::MONOCHROM_2D)      
    .value("JPEG_IMAGE", ifm3d::buffer_id::JPEG_IMAGE)      
    .value("CONFIDENCE_IMAGE", ifm3d::buffer_id::CONFIDENCE_IMAGE)      
    .value("DIAGNOSTIC", ifm3d::buffer_id::DIAGNOSTIC)      
    .value("JSON_DIAGNOSTIC", ifm3d::buffer_id::JSON_DIAGNOSTIC)      
    .value("EXTRINSIC_CALIB", ifm3d::buffer_id::EXTRINSIC_CALIB)      
    .value("INTRINSIC_CALIB", ifm3d::buffer_id::INTRINSIC_CALIB)      
    .value("INVERSE_INTRINSIC_CALIBRATION", ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION)
    .value("TOF_INFO", ifm3d::buffer_id::TOF_INFO)
    .value("O3R_DISTANCE_IMAGE_INFO", ifm3d::buffer_id::O3R_DISTANCE_IMAGE_INFO)      
    .value("RGB_INFO", ifm3d::buffer_id::RGB_INFO)
    .value("O3R_RGB_IMAGE_INFO", ifm3d::buffer_id::O3R_RGB_IMAGE_INFO)      
    .value("JSON_MODEL", ifm3d::buffer_id::JSON_MODEL)      
    .value("ALGO_DEBUG", ifm3d::buffer_id::ALGO_DEBUG)      
    .value("O3R_ODS_OCCUPANCY_GRID", ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID)      
    .value("O3R_ODS_INFO", ifm3d::buffer_id::O3R_ODS_INFO)      
    .value("XYZ", ifm3d::buffer_id::XYZ)      
    .value("EXPOSURE_TIME", ifm3d::buffer_id::EXPOSURE_TIME)      
    .value("ILLUMINATION_TEMP", ifm3d::buffer_id::ILLUMINATION_TEMP);

  frame.def(
    "timestamps",
    &ifm3d::Frame::TimeStamps,
    R"(
      Get the timestamps of the frame
    )");

  frame.def(
    "frame_count",
    &ifm3d::Frame::FrameCount,
    R"(
      Get the frame count according to algorithm output
    )");

  frame.def(
    "has_buffer",
    &ifm3d::Frame::HasBuffer,
    py::arg("id"),
    R"(
      Check if a buffer with the given id is available in this frame
    )");

  frame.def(
    "get_buffer",
    [](const ifm3d::Frame::Ptr& frame, ifm3d::buffer_id id){
        return ifm3d::image_to_array(frame->GetBuffer(id));
    },
    py::arg("id"),
    R"(
      Get the buffer with the given id
    )");

  frame.def(
    "get_buffers",
    &ifm3d::Frame::GetBuffers,
    R"(
      Get the list of available buffers
    )");

  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_FRAME