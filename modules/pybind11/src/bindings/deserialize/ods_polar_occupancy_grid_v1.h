/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_POLAR_OCCUPANCY_GRID_V1_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_POLAR_OCCUPANCY_GRID_V1_H

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ifm3d/deserialize/struct_o3r_ods_polar_occupancy_grid_v1.hpp>

void
bind_struct_odspolaroccupancygridv1(pybind11::module_& m)
{
  // clang-format off
    py::class_<ifm3d::ODSPolarOccupancyGridV1, ifm3d::ODSPolarOccupancyGridV1::Ptr>ods_polar_occupancy_grid_v1(
    m,
    "ODSPolarOccupancyGridV1",
    R"(
        Class for managing an instance of an struct ODSPolarOccupancyGridV1
      )");

    ods_polar_occupancy_grid_v1.def(
    py::init<>(),
    R"(
        Constructor
      )");

    ods_polar_occupancy_grid_v1.def_readonly(
    "version",
    &ifm3d::ODSPolarOccupancyGridV1::version,
    R"(
        version of the grid
      )");

    ods_polar_occupancy_grid_v1.def_readonly(
    "polarOccGrid",
    &ifm3d::ODSPolarOccupancyGridV1::polarOccGrid,
    R"(
        A compressed version of the grid using polar coordinates.
        The index corresponds to the angle
        (index i corresponds to the angle slice i*360/675 degree to (i+1)*360/675 degree),
        which is defined by atan2(ry, rx). The ray direction is given by (rx, ry).
        The value is the distance to nearest occupied cell on the ray from the vehicle origin,
        given in [mm]. In case there are no occupied cells on the ray, the value 65535 is set.
      )");

    ods_polar_occupancy_grid_v1.def_static(
    "deserialize",
    [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> in)
      -> ifm3d::ODSPolarOccupancyGridV1 {
      ifm3d::ODSPolarOccupancyGridV1 val;
      val.Read(reinterpret_cast<const uint8_t*>(in.data(0)), in.nbytes());
      return val;
    },
    R"(
        Deserialize ODSPolarOccupancyGridV1 Buffer
      )");
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_POLAR_OCCUPANCY_GRID_V1_H