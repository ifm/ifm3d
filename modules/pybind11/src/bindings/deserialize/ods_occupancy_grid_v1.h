/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_OCCUPANCY_GRID_V1_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_OCCUPANCY_GRID_V1_H

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ifm3d/deserialize/struct_o3r_ods_occupancy_grid_v1.hpp>

void
bind_struct_odsoccupancygridv1(pybind11::module_& m)
{

  // clang-format off
  py::class_<ifm3d::ODSOccupancyGridV1, ifm3d::ODSOccupancyGridV1::Ptr>ods_occupancy_grid_v1(
    m,
    "ODSOccupancyGridV1",
    R"(
        Class for managing an instance of an struct ODSOccupancyGridV1
      )");

  ods_occupancy_grid_v1.def(
    py::init<>(),
    R"(
        Constructor
      )");

  ods_occupancy_grid_v1.def_readonly(
    "timestamp_ns",
    &ifm3d::ODSOccupancyGridV1::timestamp_ns,
    R"(
        timestamp of the grid 
      )");

  ods_occupancy_grid_v1.def_readonly(
    "width",
    &ifm3d::ODSOccupancyGridV1::width,
    R"(
        number of grid cells
      )");

  ods_occupancy_grid_v1.def_readonly(
    "height",
    &ifm3d::ODSOccupancyGridV1::height,
    R"(
         number of grid cells
      )");

   ods_occupancy_grid_v1.def_readonly(
    "transfor_cell_center_to_user",
    &ifm3d::ODSOccupancyGridV1::transfor_cell_center_to_user,
    R"(
         values of matrix 2x3
         affine mapping between grid cell and user coordinate system
         e.g, multiplying the matrix with [0,0,1] gives the user coordinate
         of the center of upper left cell
      )");

  ods_occupancy_grid_v1.def_property_readonly(
    "image",
     [](const ifm3d::ODSOccupancyGridV1::Ptr& ods_occupancy_grid){
        return ifm3d::image_to_array(ods_occupancy_grid->image);
    },
    R"(
        array of width * height
      )");

  ods_occupancy_grid_v1.def_static(
    "deserialize",
    [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> in)
      -> ifm3d::ODSOccupancyGridV1 {
      ifm3d::ODSOccupancyGridV1 val;
      val.Read(reinterpret_cast<const uint8_t*>(in.data(0)), in.nbytes());
      return val;
    },
    R"(
        Deserialize ODSOccupancyGridV1 Buffer
      )");
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_OCCUPANCY_GRID_V1_H