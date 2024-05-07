/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_CAMERA_BASE
#define IFM3D_PYBIND_BINDING_CAMERA_BASE

#include <pybind11/pybind11.h>

void
bind_device(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::Device, ifm3d::Device::Ptr> device(
    m, "Device",
    R"(
      Base class for managing an instance of an all cameras
    )");

  // Types

  py::enum_<ifm3d::Device::boot_mode>(device, "boot_mode", "Enum: Camera boot up modes.")
    .value("PRODUCTIVE", ifm3d::Device::boot_mode::PRODUCTIVE, "the normal runtime firmware comes up")
    .value("RECOVERY", ifm3d::Device::boot_mode::RECOVERY, "allows you to flash new firmware");

  py::enum_<ifm3d::Device::operating_mode>(device, "operating_mode", "Enum: Camera operating modes")
    .value("RUN", ifm3d::Device::operating_mode::RUN, "streaming pixel data")
    .value("EDIT", ifm3d::Device::operating_mode::EDIT, "configuring the device/applications");

  py::enum_<ifm3d::Device::trigger_mode>(device, "trigger_mode", "Enum: Image acquisition trigger modes")
    .value("FREE_RUN", ifm3d::Device::trigger_mode::FREE_RUN)
    .value("SW", ifm3d::Device::trigger_mode::SW);

  py::enum_<ifm3d::Device::import_flags>(device, "import_flags", "Enum: Import flags used when importing a Vision Assistant configuration")
    .value("GLOBAL", ifm3d::Device::import_flags::GLOBAL)
    .value("NET", ifm3d::Device::import_flags::NET)
    .value("APPS", ifm3d::Device::import_flags::APPS);

  py::enum_<ifm3d::Device::spatial_filter>(device, "spatial_filter", "Enum: Convenience constants for spatial filter types")
    .value("OFF", ifm3d::Device::spatial_filter::OFF)
    .value("MEDIAN", ifm3d::Device::spatial_filter::MEDIAN)
    .value("MEAN", ifm3d::Device::spatial_filter::MEAN)
    .value("BILATERAL", ifm3d::Device::spatial_filter::BILATERAL);

  py::enum_<ifm3d::Device::temporal_filter>(device, "temporal_filter", "Enum: Convenience constants for temporal filter types")
    .value("OFF", ifm3d::Device::temporal_filter::OFF)
    .value("MEAN", ifm3d::Device::temporal_filter::MEAN)
    .value("ADAPTIVE_EXP", ifm3d::Device::temporal_filter::ADAPTIVE_EXP);

  py::enum_<ifm3d::Device::mfilt_mask_size>(device, "mfilt_mask_size", "Enum: Convenient constants for median filter mask sizes")
    .value("_3x3", ifm3d::Device::mfilt_mask_size::_3x3)
    .value("_5x5", ifm3d::Device::mfilt_mask_size::_5x5);

  py::enum_<ifm3d::Device::device_family>(device, "device_family", "Enum: The family of the device")
    .value("UNKNOWN", ifm3d::Device::device_family::UNKNOWN)
    .value("O3D", ifm3d::Device::device_family::O3D)
    .value("O3X", ifm3d::Device::device_family::O3X)
    .value("O3R", ifm3d::Device::device_family::O3R);

  // Ctor
  device.def(
    py::init(&ifm3d::Device::MakeShared),
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
    py::arg("password") = ifm3d::DEFAULT_PASSWORD,
    py::arg("throw_if_unavailable") = true
  );

  // Accessors/Mutators

  device.def_property_readonly(
    "ip",
    &ifm3d::Device::IP,
    R"(The IP address associated with this Camera instance)");

  device.def_property_readonly(
    "xmlrpc_port",
    &ifm3d::Device::XMLRPCPort,
    R"(The XMLRPC port associated with this Camera instance)");

  device.def(
    "force_trigger",
    &ifm3d::Device::ForceTrigger,
    py::call_guard<py::gil_scoped_release>(),
    R"(
      Sends a S/W trigger to the camera over XMLRPC.

      The O3X does not S/W trigger over PCIC, so, this function
      has been developed specficially for it. For the O3D, this is a NOOP.
    )");

  device.def(
    "reboot",
    &ifm3d::Device::Reboot,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("mode") = ifm3d::Device::boot_mode::PRODUCTIVE,
    R"(
      Reboot the sensor

      Parameters
      ----------
      mode : CameraBase.boot_mode
          The system mode to boot into upon restart of the sensor

      Raises
      ------
      RuntimeError
    )");

  device.def(
    "device_type",
    &ifm3d::Device::DeviceType,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("use_cached") = true,
    R"(
      Obtains the device type of the connected camera.

      This is a convenience function for extracting out the device type of the
      connected camera. The primary intention of this function is for internal
      usage (i.e., to trigger conditional logic based on the model hardware
      we are talking to) however, it will likely be useful in
      application-level logic as well, so, it is available in the public
      interface.

      Parameters
      ----------
      use_cached : bool
          If set to true, a cached lookup of the device type will be used as
          the return value. If false, it will make a network call to the camera
          to get the "real" device type. The only reason for setting this to
          `false` would be if you expect over the lifetime of your camera
          instance that you will swap out (for example) an O3D for an O3X (or
          vice versa) -- literally, swapping out the network cables while an
          object instance is still alive. If that is not something you are
          worried about, leaving this set to true should result in a signficant
          performance increase.

      Returns
      -------
      str
          Type of device connected
    )");

  device.def(
    "who_am_i",
    &ifm3d::Device::WhoAmI,
    R"(
      Retrieve the device family of the connected device

      Returns
      -------
      CameraBase.device_family
          The device family
    )");

  device.def(
    "am_i",
    &ifm3d::Device::AmI,
    py::arg("family"),
    R"(
      Checking whether a device is one of the specified device family

      Parameters
      ----------
      family : CameraBase.device_family
          The family to check for

      Returns
      -------
      bool
          True if the device is of the specified family
    )");

  device.def(
    "device_parameter",
    &ifm3d::Device::DeviceParameter,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("key"),
    R"(
      Convenience accessor for extracting a device parameter

      No edit session is created on the camera

      Parameters
      ----------
      key : str
          Name of the parameter to extract

      Returns
      -------
      str
          Value of the requested parameter

      Raises
      ------
      RuntimeError
    )");

  device.def(
    "trace_logs",
    &ifm3d::Device::TraceLogs,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("count"),
    R"(
      Delivers the trace log from the camera

      A session is not required to call this function.

      Parameters
      ----------
      count : int
          Number of entries to retrieve

      Returns
      -------
      list[str]
          List of strings for each entry in the tracelog
    )");

  device.def(
    "check_minimum_firmware_version",
    &ifm3d::Device::CheckMinimumFirmwareVersion,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("major"),
    py::arg("minor"),
    py::arg("patch"),
    R"(
      Checks for a minimum ifm camera software version

      Parameters
      ----------
      major : int
          Major version of software

      minor : int
          Minor Version of software

      patch : int
          Patch Number of software

      Returns
      -------
      bool
          True if current software version is greater
          or equal to the value passed
    )");

  device.def(
    "firmware_version",
    &ifm3d::Device::FirmwareVersion,
    R"(
      Version of firmware installed on device

      Returns
      -------
      SemVer
          Firmware version 
    )");

  device.def(
    "to_json",
    [](const ifm3d::Device::Ptr& c) -> py::dict
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      py::gil_scoped_release release;
      auto json_string = c->ToJSONStr();
      py::gil_scoped_acquire acquire;
      return json_loads(json_string);
    },
    R"(
      A JSON object containing the state of the camera

      Returns
      -------
      dict
          Camera JSON, compatible with python's json module

      Raises
      ------
      RuntimeError
    )");

  device.def(
    "from_json",
    [](const ifm3d::Device::Ptr& c, const py::dict& json)
    {
      // Convert the input JSON to string and load it
      py::object json_dumps = py::module::import("json").attr("dumps");
      auto json_string = json_dumps(json).cast<std::string>();
      py::gil_scoped_release release;
      c->FromJSONStr(json_string);
    },
    py::arg("json"),
    R"(
      Configures the camera based on the parameter values of the passed in
      JSON. This function is _the_ way to tune the
      camera/application/imager/etc. parameters.

      Parameters
      ----------
      json : dict
          A json object encoding a camera configuration to apply
          to the hardware.

      Raises
      ------
      RuntimeError
          If this raises an exception, you are
          encouraged to check the log file as a best effort is made to be
          as descriptive as possible as to the specific error that has
          occured.
    )");
  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_CAMERA_BASE