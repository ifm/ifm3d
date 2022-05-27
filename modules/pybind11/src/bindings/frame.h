/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_FRAME
#define IFM3D_PYBIND_BINDING_FRAME

#include <pybind11/pybind11.h>

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

  py::enum_<ifm3d::buffer_id>(m, "image_id", "Enum: image_ids available for use with the default Organizer.")
    .value("RADIAL_DISTANCE", ifm3d::buffer_id::RADIAL_DISTANCE)
    .value("AMPLITUDE", ifm3d::buffer_id::AMPLITUDE)
    .value("RAW_AMPLITUDE", ifm3d::buffer_id::RAW_AMPLITUDE)
    .value("GRAY", ifm3d::buffer_id::GRAY)
    .value("CARTESIAN_X", ifm3d::buffer_id::CARTESIAN_X)
    .value("CARTESIAN_Y", ifm3d::buffer_id::CARTESIAN_Y)
    .value("CARTESIAN_Z", ifm3d::buffer_id::CARTESIAN_Z)
    .value("CARTESIAN_ALL", ifm3d::buffer_id::CARTESIAN_ALL)
    .value("UNIT_VECTOR_ALL", ifm3d::buffer_id::UNIT_VECTOR_ALL)
    .value("JPEG", ifm3d::buffer_id::JPEG)
    .value("CONFIDENCE", ifm3d::buffer_id::CONFIDENCE)
    .value("DIAGNOSTIC_DATA", ifm3d::buffer_id::DIAGNOSTIC_DATA)
    .value("EXTRINSIC_CALIBRATION", ifm3d::buffer_id::EXTRINSIC_CALIBRATION)
    .value("INTRINSIC_CALIBRATION", ifm3d::buffer_id::INTRINSIC_CALIBRATION)
    .value("INVERSE_INTRINSIC_CALIBRATION", ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION)
    .value("O3R_DISTANCE_IMAGE_INFORMATION", ifm3d::buffer_id::O3R_DISTANCE_IMAGE_INFORMATION)
    .value("JSON_MODEL", ifm3d::buffer_id::JSON_MODEL)
    .value("ALGO_DEBUG", ifm3d::buffer_id::ALGO_DEBUG)
    .value("XYZ", ifm3d::buffer_id::XYZ);
    

  frame.def(
    "timestamps",
    &ifm3d::Frame::TimeStamps,
    R"(
      Get the timestamps of the frame
    )");

  frame.def(
    "has_image",
    &ifm3d::Frame::HasBuffer,
    py::arg("id"),
    R"(
      Check if a image with the given id is available in this frame
    )");

  frame.def(
    "get_image",
    [](const ifm3d::Frame::Ptr& frame, ifm3d::buffer_id id){
        return ifm3d::image_to_array(frame->GetBuffer(id));
    },
    py::arg("id"),
    R"(
      Get the image with the given id
    )");

  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_FRAME