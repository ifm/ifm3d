/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_RGB_INFO_V1_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_RGB_INFO_V1_H

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ifm3d/deserialize/struct_rgb_info_v1.hpp>

void
bind_struct_rgbinfov1(pybind11::module_& m)
{
  // clang-format off
 
  py::class_<ifm3d::RGBInfoV1, ifm3d::RGBInfoV1::Ptr> rgb_info_v1(
    m,
    "RGBInfoV1",
    R"(
        Class for managing an instance of an struct RGBInfoV1
      )");

  rgb_info_v1.def(
    py::init<>(),
    R"(
        Constructor
      )");

  rgb_info_v1.def_readonly(
    "version",
    &ifm3d::RGBInfoV1::version,
    R"(
        Version of the RGB_INFO data
      )");

  rgb_info_v1.def_readonly(
    "frame_counter",
    &ifm3d::RGBInfoV1::frame_counter,
    R"(
        Frame count, The frame counter is initialized to 0 at the initialization
      )");
  rgb_info_v1.def_readonly(
    "timestamp_ns",
    &ifm3d::RGBInfoV1::timestamp_ns,
    R"(
        The timestamp of the 2D image, given in nano second
      )");
  rgb_info_v1.def_readonly(
    "exposure_time",
    &ifm3d::RGBInfoV1::exposure_time,
    R"(
        Actual exposure time of the 2D image
      )");
  rgb_info_v1.def_readonly(
    "extrisic_optic_to_user",
    &ifm3d::RGBInfoV1::extrisic_optic_to_user,
    R"(
        Extrinsic optic paramter of the 2D head
      )");
  rgb_info_v1.def_readonly(
    "intrinsic_calibration",
    &ifm3d::RGBInfoV1::intrinsic_calibration,
    R"(
        Intrinsic Calibration parameters
      )");
  rgb_info_v1.def_readonly(
    "inverse_intrinsic_calibration",
    &ifm3d::RGBInfoV1::inverse_intrinsic_calibration,
    R"(
        Inverse intrinsic Calibration parameters
      )");
 
  rgb_info_v1.def_static(
    "deserialize",
    [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> in) -> ifm3d::RGBInfoV1 {
    ifm3d::RGBInfoV1 val;
    val.Read(reinterpret_cast<const uint8_t*>(in.data(0)),in.nbytes());
    return val;
  },
    R"(
        Deserialize RGB_INFO Buffer to RGBInfoV1 struct
      )");
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_RGB_INFO_V1_H