/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_CAMERA_O3C
#define IFM3D_PYBIND_BINDING_CAMERA_O3C

#include <fmt/format.h>
#include <ifm3d/device/o3c.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

inline void
bind_o3c(pybind11::module_& m)
{
  // Bind O3C class - inherits all O3R methods automatically
  py::class_<ifm3d::O3C, ifm3d::O3C::Ptr, ifm3d::O3R> o3c(m,
                                                          "O3C",
                                                          R"(
      Class for managing an instance of an O3C Camera.
    )");

  o3c.def(py::init([](std::string ip, std::uint16_t xmlrpc_port) {
            return std::make_shared<ifm3d::O3C>(ip, xmlrpc_port);
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
    )",
          py::arg("ip") = ifm3d::DEFAULT_IP,
          py::arg("xmlrpc_port") = ifm3d::DEFAULT_XMLRPC_PORT);

  // Create aliases to O3R's nested classes for O3C
  // This allows O3C.SetTemporaryApplicationParameter and O3C.Parameter
  // to reference the same classes defined in O3R
  o3c.attr("SetTemporaryApplicationParameter") =
    m.attr("O3R").attr("SetTemporaryApplicationParameter");
  o3c.attr("Parameter") = m.attr("O3R").attr("Parameter");
}

#endif // IFM3D_PYBIND_BINDING_CAMERA_O3C
