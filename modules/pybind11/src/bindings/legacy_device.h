/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_CAMERA
#define IFM3D_PYBIND_BINDING_CAMERA

#include <pybind11/pybind11.h>

void
bind_legacy_device(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::LegacyDevice, ifm3d::LegacyDevice::Ptr, ifm3d::Device> legacy_device(
    m, "LegacyDevice",
    R"(
      Class for managing an instance of an O3D/O3X Camera
    )");

  legacy_device.def(
    py::init(&ifm3d::LegacyDevice::MakeShared),
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
    py::arg("xmlrpc_port") = ifm3d::DEFAULT_XMLRPC_PORT,
    py::arg("password") = ifm3d::DEFAULT_PASSWORD);

  legacy_device.def_property(
    "password",
    &ifm3d::LegacyDevice::Password,
    &ifm3d::LegacyDevice::SetPassword,
    R"(The password associated with this Camera instance)");

  legacy_device.def_property_readonly(
    "session_id",
    &ifm3d::LegacyDevice::SessionID,
    R"(Retrieves the active session ID)");

  // Member Functions

  legacy_device.def(
    "request_session",
    &ifm3d::LegacyDevice::RequestSession,
    py::call_guard<py::gil_scoped_release>(),
    R"(
      Requests an edit-mode session with the camera.

      In order to (permanently) mutate parameters on the camera, an edit
      session needs to be established. Only a single edit sesson may be
      established at any one time with the camera (think of it as a global
      mutex on the camera state -- except if you ask for the mutex and it is
      already taken, an exception will be thrown).

      Most typical use-cases for end-users will not involve establishing an
      edit-session with the camera. To mutate camera parameters, the
      `FromJSON` family of functions should be used, which, under-the-hood,
      on the user's behalf, will establish the edit session and gracefully
      close it. There is an exception. For users who plan to modulate imager
      parameters (temporary parameters) on the fly while running the
      framegrabber, managing the session manually is necessary. For this
      reason, we expose this method in the public `Camera` interface.

      NOTE: The session timeout is implicitly set to `ifm3d::MAX_HEARTBEAT`
      after the session has been successfully established.

      Returns
      -------
      str
          The session id issued or accepted by the camera (see
          IFM3D_SESSION_ID environment variable)

      Raises
      ------
      RuntimeError

      @throws ifm3d::error_t if an error is encountered.
    )");

  legacy_device.def(
    "cancel_session",
    (bool (ifm3d::LegacyDevice::*)(void)) &ifm3d::LegacyDevice::CancelSession,
     py::call_guard<py::gil_scoped_release>(),
    R"(
      Explictly stops the current session with the sensor.

      Returns
      -------
      bool
          Indicates success or failure. On failure, check the ifm3d system log
          for details.
    )");

  legacy_device.def(
    "cancel_session",
    (bool (ifm3d::LegacyDevice::*)(const std::string&))&ifm3d::LegacyDevice::CancelSession,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("sid"),
    R"(
      Attempts to cancel a session with a particular session id.

      Parameters
      ----------
      sid : str
          Session ID to cancel.

      Returns
      -------
      bool
          Indicates success or failure. On failure, check the ifm3d system log
          for details.
    )");

  legacy_device.def(
    "heartbeat",
    &ifm3d::LegacyDevice::Heartbeat,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("hb"),
    R"(
      Sends a heartbeat message and sets the next heartbeat interval

      Heartbeat messages are used to keep a session with the sensor
      alive. This function sends a heartbeat message to the sensor and sets
      when the next heartbeat message is required.

      Parameters
      ----------
      hb : int
          The time (seconds) of when the next heartbeat message will
          be required.

      Returns
      -------
      int
          The current timeout interval in seconds for heartbeat messages

      Raises
      ------
      RuntimeError
    )");

  legacy_device.def(
    "set_temporary_application_parameters",
    &ifm3d::LegacyDevice::SetTemporaryApplicationParameters,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("params"),
    R"(
      Sets temporary application parameters in run mode.

      The changes are not persistent and are lost when entering edit mode or
      turning the device off. The parameters "ExposureTime" and
      "ExposureTimeRatio" of the imager configuration are supported. All
      additional parameters are ignored (for now). Exposure times are clamped
      to their allowed range, depending on the exposure mode. The user must
      provide the complete set of parameters depending on the exposure mode,
      i.e., "ExposureTime" only for single exposure modes and both
      "ExposureTime" and "ExposureTimeRatio" for double exposure
      modes. Otherwise, behavior is undefined.

      Parameters
      ----------
      params : dict[str, str]
          The parameters to set on the camera.

      Raises
      ------
      RuntimeError
    )");

  legacy_device.def(
    "active_application",
    &ifm3d::LegacyDevice::ActiveApplication,
    py::call_guard<py::gil_scoped_release>(),
    R"(
      Returns the index of the active application.

      A negative number indicates no application is marked as active on the
      sensor.
    )");

  legacy_device.def(
    "application_list",
    [](const ifm3d::LegacyDevice::Ptr& c) -> py::dict
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      py::gil_scoped_release release;
      auto json_string = c->ApplicationList().dump(2);
      py::gil_scoped_acquire acquire;
      return json_loads(json_string);
      },
      R"(
      Delivers basic information about all applications stored on the device.
      A call to this function does not require establishing a session with the
      camera.

      The returned information is encoded as an array of JSON objects.
      Each object in the array is basically a dictionary with the following
      keys: 'index', 'id', 'name', 'description', 'active'

      Returns
      -------
      dict
          A JSON encoding of the application information

      Raises
      ------
      RuntimeError
      )");

  legacy_device.def(
    "application_types",
    &ifm3d::LegacyDevice::ApplicationTypes,
    py::call_guard<py::gil_scoped_release>(),
    R"(
      Lists the valid application types supported by the sensor.

      Returns
      -------
      list[str]
          List of strings of the available types of applications supported by
          the sensor. Each element in the list is a string suitable to passing
          to 'CreateApplication'.

      Raises
      ------
      RuntimeError
    )");

  legacy_device.def(
    "imager_types",
    &ifm3d::LegacyDevice::ImagerTypes,
    py::call_guard<py::gil_scoped_release>(),
    R"(
      Lists the valid imager types supported by the sensor.

      Returns
      -------
      list[str]
          List of strings of the available imager types supported by the sensor

      Raises
      ------
      RuntimeError
    )");

  legacy_device.def(
    "copy_application",
    &ifm3d::LegacyDevice::CopyApplication,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("idx"),
    R"(
      Creates a new application by copying the configuration of another
      application. The device will generate an ID for the new application and
      put it on a free index.

      Parameters
      ----------
      idx : int
          The index of the application to copy

      Returns
      -------
      int
          Index of the new application

      Raises
      ------
      RuntimeError
    )");

  legacy_device.def(
    "create_application",
    &ifm3d::LegacyDevice::CreateApplication,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("type") = ifm3d::DEFAULT_APPLICATION_TYPE,
    R"(
      Creates a new application on the camera of the given type.

      To figure out valid `type`s, you should call the
      AvailableApplicationTypes()` method.

      Upon creation of the application, the embedded device will initialize
      all parameters as necessary based on the type. However, based on the
      type, the application may not be in an _activatable_ state. That is, it
      can be created and saved on the device, but it cannot be marked as
      active.

      Parameters
      ----------
      type : str, optional
          The (optional) application type to create. By default,
          it will create a new "Camera" application.

      Returns
      -------
      int
          The index of the new application.
    )");

  legacy_device.def(
    "delete_application",
    &ifm3d::LegacyDevice::DeleteApplication,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("idx"),
    R"(
      Deletes the application at the specified index from the sensor.

      Parameters
      ----------
      idx : int
          The index of the application to delete

      Raises
      ------
      RuntimeError
    )");

  legacy_device.def(
    "set_current_time",
    &ifm3d::LegacyDevice::SetCurrentTime,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("epoch_secs") = -1,
    R"(
      Sets the current time on the camera

      Parameters
      ----------
      epoch_secs : int, optional
          Time since the Unix epoch in seconds. A value less than 0 will
          implicity set the time to the current system time.
    )");

  legacy_device.def(
    "factory_reset",
    &ifm3d::LegacyDevice::FactoryReset,
    py::call_guard<py::gil_scoped_release>(),
    R"(
      Sets the camera configuration back to the state in which it shipped from
      the ifm factory.
    )");

  //
  // @TODO: Not tested; not supported on my o3d303?
  // how to expose this info anyway? numpy?
  //
  legacy_device.def(
    "unit_vectors",
    &ifm3d::LegacyDevice::UnitVectors,
    py::call_guard<py::gil_scoped_release>(),
    R"(
      For cameras that support fetching the Unit Vectors over XML-RPC, this
      function will return those data as a binary blob.

      Returns
      -------
      list[int]
    )");

  legacy_device.def(
    "export_ifm_config",
    &ifm3d::LegacyDevice::ExportIFMConfig,
    py::call_guard<py::gil_scoped_release>(),
    R"(
      Exports the entire camera configuration in a format compatible with
      Vision Assistant.

      Returns
      -------
      list[int]
    )");

  legacy_device.def(
    "export_ifm_app",
    &ifm3d::LegacyDevice::ExportIFMApp,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("idx"),
    R"(
      Export the application at the specified index into a byte array suitable
      for writing to a file. The exported bytes represent the ifm serialization
      of an application.

      This function provides compatibility with tools like IFM's Vision
      Assistant.

      Parameters
      ----------
      idx : int
          The index of the application to export.

      Returns
      -------
      list[int]
          A list of bytes representing the IFM serialization of the
          exported application.

      Raises
      ------
      RuntimeError
    )");

  legacy_device.def(
    "import_ifm_config",
    &ifm3d::LegacyDevice::ImportIFMConfig,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("bytes"),
    py::arg("flags") = 0,
    R"(
      Imports the entire camera configuration in a format compatible with
      Vision Assistant.

      Parameters
      ----------
      bytes : list[int]
          The camera configuration, serialized in the ifm format

      flags : int
    )");

  legacy_device.def(
    "import_ifm_app",
    &ifm3d::LegacyDevice::ImportIFMApp,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("bytes"),
    R"(
      Import the IFM-encoded application.

      This function provides compatibility with tools like IFM's Vision
      Assistant. An application configuration exported from VA, can be
      imported using this function.

      Parameters
      ----------
      bytes : list[int]
          The raw bytes from the zip'd JSON file. NOTE: This
          function will base64 encode the data for tranmission
          over XML-RPC.

      Returns
      -------
      int
          The index of the imported application.
    )");
  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_CAMERA