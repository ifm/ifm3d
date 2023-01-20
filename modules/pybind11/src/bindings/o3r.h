/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_CAMERA_O3R
#define IFM3D_PYBIND_BINDING_CAMERA_O3R

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <fmt/core.h>

void
bind_o3r(pybind11::module_& m)
{

  // clang-format off
  py::class_<ifm3d::PortInfo> port_info(
      m, "PortInfo",
      R"(
        Provides information about a connected Port
      )"
    );

  port_info.def_readonly("port", &ifm3d::PortInfo::port, "The name of the port.");
  port_info.def_readonly("pcic_port", &ifm3d::PortInfo::pcic_port, "The assigned pcic port.");
  port_info.def_readonly("type", &ifm3d::PortInfo::type, "The type of the conntected sensor.");
  port_info.def(
    "__repr__", 
    [](ifm3d::PortInfo* self) {
      return fmt::format("PortInfo(port: \"{}\", pcic_port: {}, type: \"{}\")", 
        self->port, self->pcic_port, self->type);
    }
  );

  py::class_<ifm3d::O3R, ifm3d::O3R::Ptr, ifm3d::Device> o3r(
    m, "O3R",
    R"(
      Class for managing an instance of an O3R Camera
    )");

  o3r.def(
    py::init([](std::string ip, std::uint16_t xmlrpc_port) {
      return std::make_shared<ifm3d::O3R>(ip, xmlrpc_port);
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

  o3r.def(
    "factory_reset",
    &ifm3d::O3R::FactoryReset,
    py::arg("keep_network_settings"),
    R"(
      Sets the camera configuration back to the state in which it shipped from
      the ifm factory.

      Parameters
      ----------
      keep_network_settings : bool
          A bool indicating wether to keep the current network settings
    )");

  o3r.def(
    "get",
    [](const ifm3d::O3R::Ptr& c, const std::vector<std::string>& path)
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

  o3r.def(
    "set",
    [](const ifm3d::O3R::Ptr& c, const py::dict& json)
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

  o3r.def(
    "remove",
    &ifm3d::O3R::Remove,
    py::arg("json_pointer"),
    R"(
      Removes an object from the JSON. The scope of this method is limited to
      the following regular expressions

       * ^\/applications\/instances\/app\d+$
       * ^\/device\/log\/components\/[a-zA-Z0-9\-_]+$

      Parameters
      ----------
      json_pointer : string
          A JSON Pointer to the object to be removed.
    )");

  o3r.def(
    "reset",
    &ifm3d::O3R::Reset,
    py::arg("json_pointer"),
    R"(
      Sets the default value of an object inside the JSON. The object is
      addressed by a JSON Pointer. The object is resetted to the values
      defined in the JSON schema.

      Parameters
      ----------
      json_pointer : string
          A JSON Pointer to the object to be set to default.
    )");

  o3r.def(
    "get_init",
    [](const ifm3d::O3R::Ptr& c)
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

  o3r.def(
    "save_init",
    &ifm3d::O3R::SaveInit,
    R"(
      Save to current temporary JSON configuration as initial JSON
      configuration
    )");

  o3r.def(
    "get_init_status",
    &ifm3d::O3R::GetInitStatus,
    R"(
      Returns the init status of the device

      Returns
      -------
      dict
          The init status of the device
    )");

  o3r.def(
    "lock",
    &ifm3d::O3R::Lock,
    py::arg("password"),
    R"(
      Release the lock from the Device

      Returns
      -------
      string
          The password used to unlock the device
    )");

  o3r.def(
    "unlock",
    &ifm3d::O3R::Unlock,
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

  o3r.def(
    "get_schema",
    [](const ifm3d::O3R::Ptr& c)
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

  o3r.def(
    "get_diagnostic",
    [](const ifm3d::O3R::Ptr& c)
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      return json_loads(c->GetDiagnostic().dump());
    },
    R"(
      Returns the content of the diagnostic memory formatted in JSON

      Returns
      -------
      dict
    )");

  o3r.def(
    "get_diagnostic_filter_schema",
    [](const ifm3d::O3R::Ptr& c)
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      return json_loads(c->GetDiagnosticFilterSchema().dump());
    },
    R"(
      Returns the JSON schema for the filter expression provided to the 
      getFiltered() method

      Returns
      -------
      dict
          The JSON schema
    )");

  o3r.def(
    "get_diagnostic_filtered",
    [](const ifm3d::O3R::Ptr& c, const py::dict& filter)
    {
      py::object json_dumps = py::module::import("json").attr("dumps");
      py::object json_loads = py::module::import("json").attr("loads");
      return json_loads(c->GetDiagnosticFiltered(json::parse(json_dumps(filter).cast<std::string>())).dump());
    },
    py::arg("filter"),
    R"(
      Returns the content of the diagnostic memory formatted in JSON
      and filtered according to the JSON filter expression

      Parameters
      ----------
      filter : dict
          A filter expression in JSON format

      Returns
      -------
      dict 
    )");

  o3r.def(
    "ports",
    &ifm3d::O3R::Ports,
    R"(
      Returns a list containing information about all connected physical ports

      Returns
      -------
      List[PortInfo]
          the list of Ports
    )");

  o3r.def(
    "port",
    &ifm3d::O3R::Port,
    py::arg("port"),
    R"(
      Returns information about a given physical port

      Parameters
      ----------
      port : str
          the port for which to get the information

      Returns
      -------
      PortInfo
          the port information
    )");

  o3r.def(
    "reboot_to_recovery",
    &ifm3d::O3R::RebootToRecovery,
    R"(
      Reboot the device into Recovery Mode

      Raises
      ------
      RuntimeError
    )");
  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_CAMERA_O3R