/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_CAMERA_O3R
#define IFM3D_PYBIND_BINDING_CAMERA_O3R

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

void
bind_camera_o3r(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::O3RCamera, ifm3d::O3RCamera::Ptr, ifm3d::CameraBase> camera_o3r(
    m, "O3RCamera",
    R"(
      Class for managing an instance of an O3R Camera
    )");

  camera_o3r.def(
    py::init([](std::string ip, std::uint16_t xmlrpc_port) {
      return std::make_shared<ifm3d::O3RCamera>(ip, xmlrpc_port);
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

  camera_o3r.def(
    "factory_reset",
    &ifm3d::O3RCamera::FactoryReset,
    py::arg("keep_network_settings"),
    R"(
      Sets the camera configuration back to the state in which it shipped from
      the ifm factory.

      Parameters
      ----------
      keep_network_settings : bool
          A bool indicating wether to keep the current network settings
    )");

  camera_o3r.def(
    "get",
    [](const ifm3d::O3RCamera::Ptr& c, const std::vector<std::string>& path)
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      return json_loads(c->Get(path).dump());
    },
    py::arg("path") = std::vector<std::string>(),
    R"(
      Returns the configuration formatted as JSON based on a path.
      If the path is empty, returns the whole configuration.

      Returns
      -------
      dict
          The JSON configuration for the list of object path fragments
    )");

  camera_o3r.def(
    "set",
    [](const ifm3d::O3RCamera::Ptr& c, const py::dict& json)
    {
      // Convert the input JSON to string and load it
      py::object json_dumps = py::module::import("json").attr("dumps");
      c->Set(json::parse(json_dumps(json).cast<std::string>()));
    },
    py::arg("json"),
    R"(
      Overwrites parts of the temporary JSON configuration which is achieved
      by merging the provided JSON fragment with the current temporary JSON.

      Parameters
      ----------
      json : dict
          The new temporay JSON configuration of the device.
    )");

  camera_o3r.def(
    "get_init",
    [](const ifm3d::O3RCamera::Ptr& c)
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      return json_loads(c->GetInit().dump());
    },
    R"(
      Return the initial JSON configuration.

      Returns
      -------
      dict
          The initial JSON configuration
    )");

  camera_o3r.def(
    "save_init",
    &ifm3d::O3RCamera::SaveInit,
    R"(
      Save to current temporary JSON configuration as initial JSON
      configuration
    )");

  camera_o3r.def(
    "get_init_status",
    &ifm3d::O3RCamera::GetInitStatus,
    R"(
      Returns the init status of the device

      Returns
      -------
      dict
          The init status of the device
    )");

  camera_o3r.def(
    "lock",
    &ifm3d::O3RCamera::Lock,
    py::arg("password"),
    R"(
      Release the lock from the Device

      Returns
      -------
      string
          The password used to unlock the device
    )");

  camera_o3r.def(
    "unlock",
    &ifm3d::O3RCamera::Unlock,
    py::arg("password"),
    R"(
      Locks the device until it is unlocked.
      If the device is unlocked and an empty password is provided the password
      protection is removed.

      Returns
      -------
      string
          The password used to lock the device
    )");

  camera_o3r.def(
    "get_schema",
    [](const ifm3d::O3RCamera::Ptr& c)
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      return json_loads(c->GetSchema().dump());
    },
    R"(
      Returns the current JSON schema configuration

      Returns
      -------
      dict
          The current json schema configuration
    )");
  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_CAMERA_O3R