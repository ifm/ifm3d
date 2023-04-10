/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_O3D_PARAM_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_O3D_PARAM_H

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ifm3d/deserialize/deserialize_o3d_buffers.hpp>

template <typename T>
void
bind_o3d_parameter(pybind11::module_& m, const std::string& name)
{
  // clang-format off
  py::class_<T, std::shared_ptr<T>> o3d_parameter(
    m,
    name.c_str(),
    (R"(
        Class for managing an o3d_parameter
      )" + name).c_str());

   o3d_parameter.def(
      py::init<>(),
    R"(
        Constructor
      )");

   o3d_parameter.def_readonly(
    "data",
    &T::data,
    R"(
        array of paramter values
      )");

    o3d_parameter.def_static(
    "deserialize",
    [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> in)
      -> T {
      T val;
      val.Read(reinterpret_cast<const uint8_t*>(in.data(0)), in.nbytes());
      return val;
    },
    R"(
        Deserialize  O3D Buffer  array of values
      )");
  // clang-format on
}

void
bind_struct_o3d_paramters(pybind11::module_& m)
{

  bind_o3d_parameter<ifm3d::O3DInstrinsicCalibration>(
    m,
    "O3DInstrinsicCalibration");

  m.attr("O3DInverseInstrinsicCalibration") =
    m.attr("O3DInstrinsicCalibration");

  bind_o3d_parameter<ifm3d::O3DExtrinsicCalibration>(
    m,
    "O3DExtrinsicCalibration");

  bind_o3d_parameter<ifm3d::O3DExposureTimes>(m, "O3DExposureTimes");

  bind_o3d_parameter<ifm3d::O3DILLUTemperature>(m, "O3DILLUTemperature");
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_O3D_PARAM_H