/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_INFO_V1_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_INFO_V1_H

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ifm3d/deserialize/struct_o3r_ods_info_v1.hpp>

void
bind_struct_odsinfov1(pybind11::module_& m)
{

  // clang-format off
  py::class_<ifm3d::ODSInfoV1, ifm3d::ODSInfoV1::Ptr>ods_info_v1(
    m,
    "ODSInfoV1",
    R"(
        Class for managing an instance of an struct ODSInfoV1
      )");

  ods_info_v1.def(
    py::init<>(),
    R"(
        Constructor
      )");

  ods_info_v1.def_readonly(
    "timestamp_ns",
    &ifm3d::ODSInfoV1::timestamp_ns,
    R"(
        Timestamp of zone information [ns]
      )");

  ods_info_v1.def_readonly(
    "zone_occupied",
    &ifm3d::ODSInfoV1::zone_occupied,
    R"(
         Array with three value of uint8_t
         0 : zone is free 
         1 : zone is occupied
      )");

  ods_info_v1.def_readonly(
    "zone_config_id",
    &ifm3d::ODSInfoV1::zone_config_id,
    R"(
         User specific id to identify the zone configuration
      )");

  ods_info_v1.def_static(
    "deserialize",
    [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> in)
      -> ifm3d::ODSInfoV1 {
      ifm3d::ODSInfoV1 val;
      val.Read(reinterpret_cast<const uint8_t*>(in.data(0)), in.nbytes());
      return val;
    },
    R"(
        Deserialize ODS_INFO Buffer to ODSInfoV1 struct
      )");
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_INFO_V1_H