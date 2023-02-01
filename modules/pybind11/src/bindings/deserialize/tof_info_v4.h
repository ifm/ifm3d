/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_TOF_INFO_V4_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_TOF_INFO_V4_H

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ifm3d/deserialize/struct_tof_info_v4.hpp>

void
bind_struct_tofinfov4(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::TOFInfoV4, ifm3d::TOFInfoV4::Ptr, ifm3d::TOFInfoV3> tof_info_v4(
    m,
    "TOFInfoV4",
    R"(
        Class for managing an instance of an struct TofInfoV4
      )");

  tof_info_v4.def(
    py::init<>(),
    R"(
        Constructor
      )");

  tof_info_v4.def_readonly(
    "measurement_block_index",
    &ifm3d::TOFInfoV4::measurement_block_index,
    R"(
         Current measurement block index (range 0 to N-1, where N is the number of sub-modes).
         This identifies the currently used sub-mode in cyclic modes.
         In non-cyclic modes this value is always 0.
      )");

  tof_info_v4.def_readonly(
    "measurement_range_min",
    &ifm3d::TOFInfoV4::measurement_range_min,
    R"(
        Current minimum measurement range [m].
        The value is based on the camera-individual ToF calibration.
        It is influenced by temperature.
      )");

  tof_info_v4.def_readonly(
    "measurement_range_max",
    &ifm3d::TOFInfoV4::measurement_range_max,
    R"(
         Current maximum measurement range [m].
         The value is based on the camera-individual ToF calibration.
         It is influenced by temperature.
      )");

  tof_info_v4.def_static(
    "deserialize",
    [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> in)
      -> ifm3d::TOFInfoV4 {
      ifm3d::TOFInfoV4 val;
      val.Read(reinterpret_cast<const uint8_t*>(in.data(0)), in.nbytes());
      return val;
    },
    R"(
        Deserialize TOF_INFO buffer to ToFInfoV4
      )");
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_TOF_INFO_V4_H