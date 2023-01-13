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
  py::class_<ifm3d::TofInfoV4, ifm3d::TofInfoV4::Ptr, ifm3d::TofInfoV3> tof_info_v4(
    m,
    "ToFInfoV4",
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
    &ifm3d::TofInfoV4::measurement_block_index,
    R"(
        index of the measurement box
      )");

  tof_info_v4.def_readonly(
    "measurement_range_min",
    &ifm3d::TofInfoV4::measurement_range_min,
    R"(
         minimum of the measurement range 
      )");

  tof_info_v4.def_readonly(
    "measurement_range_max",
    &ifm3d::TofInfoV4::measurement_range_max,
    R"(
         minimum of the measurement range 
      )");

  tof_info_v4.def_static(
    "Deserialize",
    [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> in)
      -> ifm3d::TofInfoV4 {
      ifm3d::TofInfoV4 val;
      val.Read(reinterpret_cast<const uint8_t*>(in.data(0)), in.nbytes());
      return val;
    },
    R"(
        Deserialize ToFInfoV3 Buffer
      )");
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_TOF_INFO_V4_H