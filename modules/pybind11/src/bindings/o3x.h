/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_CAMERA_O3X
#define IFM3D_PYBIND_BINDING_CAMERA_O3X

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <fmt/core.h>

void
bind_o3x(pybind11::module_& m)
{

  // clang-format off
  py::class_<ifm3d::O3X, ifm3d::O3X::Ptr, ifm3d::LegacyDevice> o3x(
    m, "O3X",
    R"(
      Class for managing an instance of an O3X Camera

      Note that O3X support is currently experimental- Use at your own risk!.
    )");

  o3x.def(
    py::init([](std::string ip, std::uint16_t xmlrpc_port) {
      return std::make_shared<ifm3d::O3X>(ip, xmlrpc_port);
    }),
    R"(
      Constructor

      Parameters
      ----------
      ip : string, optional
          The ip address of the camera. Defaults to 192.168.0.69.

      xmlrpc_port : uint, optional
          The tcp port the sensor's XMLRPC server is listening on. Defaults to
          port 80.

      password : string, optional
          Password required for establishing an "edit session" with the sensor.
          Edit sessions allow for mutating camera parameters and persisting
          those changes. Defaults to '' (no password).
    )",
    py::arg("ip") = ifm3d::DEFAULT_IP,
    py::arg("xmlrpc_port") = ifm3d::DEFAULT_XMLRPC_PORT);
  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_CAMERA_O3X