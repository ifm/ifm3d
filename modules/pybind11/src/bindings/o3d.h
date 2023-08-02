/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_CAMERA_O3D
#define IFM3D_PYBIND_BINDING_CAMERA_O3D

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <fmt/core.h>
#include <ifm3d/device/o3d.h>

void
bind_o3d(pybind11::module_& m)
{

  // clang-format off
  py::class_<ifm3d::O3D, ifm3d::O3D::Ptr, ifm3d::LegacyDevice> o3d(
    m, "O3D",
    R"(
      Class for managing an instance of an O3D Camera

      Note that O3D support is currently experimental- Use at your own risk!.
    )");

  o3d.def(
    py::init([](std::string ip, std::uint16_t xmlrpc_port) {
      return std::make_shared<ifm3d::O3D>(ip, xmlrpc_port);
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