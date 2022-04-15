/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_CAMERA_BASE
#define IFM3D_PYBIND_BINDING_CAMERA_BASE

#include <pybind11/pybind11.h>

void
bind_camera_base(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::CameraBase, ifm3d::CameraBase::Ptr> camera_base(
    m, "CameraBase",
    R"(
      Base class for managing an instance of an all cameras
    )");

  // Types

  py::enum_<ifm3d::CameraBase::boot_mode>(camera_base, "boot_mode", "Enum: Camera boot up modes.")
    .value("PRODUCTIVE", ifm3d::CameraBase::boot_mode::PRODUCTIVE, "the normal runtime firmware comes up")
    .value("RECOVERY", ifm3d::CameraBase::boot_mode::RECOVERY, "allows you to flash new firmware");

  py::enum_<ifm3d::CameraBase::operating_mode>(camera_base, "operating_mode", "Enum: Camera operating modes")
    .value("RUN", ifm3d::CameraBase::operating_mode::RUN, "streaming pixel data")
    .value("EDIT", ifm3d::CameraBase::operating_mode::EDIT, "configuring the device/applications");

  py::enum_<ifm3d::CameraBase::trigger_mode>(camera_base, "trigger_mode", "Enum: Image acquisition trigger modes")
    .value("FREE_RUN", ifm3d::CameraBase::trigger_mode::FREE_RUN)
    .value("SW", ifm3d::CameraBase::trigger_mode::SW);

  py::enum_<ifm3d::CameraBase::import_flags>(camera_base, "import_flags", "Enum: Import flags used when importing a Vision Assistant configuration")
    .value("GLOBAL", ifm3d::CameraBase::import_flags::GLOBAL)
    .value("NET", ifm3d::CameraBase::import_flags::NET)
    .value("APPS", ifm3d::CameraBase::import_flags::APPS);

  py::enum_<ifm3d::CameraBase::spatial_filter>(camera_base, "spatial_filter", "Enum: Convenience constants for spatial filter types")
    .value("OFF", ifm3d::CameraBase::spatial_filter::OFF)
    .value("MEDIAN", ifm3d::CameraBase::spatial_filter::MEDIAN)
    .value("MEAN", ifm3d::CameraBase::spatial_filter::MEAN)
    .value("BILATERAL", ifm3d::CameraBase::spatial_filter::BILATERAL);

  py::enum_<ifm3d::CameraBase::temporal_filter>(camera_base, "temporal_filter", "Enum: Convenience constants for temporal filter types")
    .value("OFF", ifm3d::CameraBase::temporal_filter::OFF)
    .value("MEAN", ifm3d::CameraBase::temporal_filter::MEAN)
    .value("ADAPTIVE_EXP", ifm3d::CameraBase::temporal_filter::ADAPTIVE_EXP);

  py::enum_<ifm3d::CameraBase::mfilt_mask_size>(camera_base, "mfilt_mask_size", "Enum: Convenient constants for median filter mask sizes")
    .value("_3x3", ifm3d::CameraBase::mfilt_mask_size::_3x3)
    .value("_5x5", ifm3d::CameraBase::mfilt_mask_size::_5x5);

  py::enum_<ifm3d::CameraBase::device_family>(camera_base, "device_family", "Enum: The family of the device")
    .value("UNKNOWN", ifm3d::CameraBase::device_family::UNKNOWN)
    .value("O3D", ifm3d::CameraBase::device_family::O3D)
    .value("O3X", ifm3d::CameraBase::device_family::O3X)
    .value("O3R", ifm3d::CameraBase::device_family::O3R);

  // Ctor

  camera_base.def(
    py::init(&ifm3d::CameraBase::MakeShared),
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

  // Accessors/Mutators

  camera_base.def_property_readonly(
    "ip",
    &ifm3d::CameraBase::IP,
    R"(The IP address associated with this Camera instance)");

  camera_base.def_property_readonly(
    "xmlrpc_port",
    &ifm3d::CameraBase::XMLRPCPort,
    R"(The XMLRPC port associated with this Camera instance)");

  camera_base.def(
    "force_trigger",
    &ifm3d::CameraBase::ForceTrigger,
    R"(
      Sends a S/W trigger to the camera over XMLRPC.

      The O3X does not S/W trigger over PCIC, so, this function
      has been developed specficially for it. For the O3D, this is a NOOP.
    )");

  camera_base.def(
    "reboot",
    &ifm3d::CameraBase::Reboot,
    py::arg("mode") = ifm3d::CameraBase::boot_mode::PRODUCTIVE,
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

  camera_base.def(
    "device_type",
    &ifm3d::CameraBase::DeviceType,
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

  camera_base.def(
    "who_am_i",
    &ifm3d::CameraBase::WhoAmI,
    R"(
      Retrieve the device family of the connected device

      Returns
      -------
      CameraBase.device_family
          The device family
    )");

  camera_base.def(
    "am_i",
    &ifm3d::CameraBase::AmI,
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

  camera_base.def(
    "device_parameter",
    &ifm3d::CameraBase::DeviceParameter,
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

  camera_base.def(
    "trace_logs",
    &ifm3d::CameraBase::TraceLogs,
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

  camera_base.def(
    "check_minimum_firmware_version",
    &ifm3d::CameraBase::CheckMinimumFirmwareVersion,
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

  camera_base.def(
    "to_json",
    [](const ifm3d::CameraBase::Ptr& c)
    {
      // Convert the JSON to a python JSON object using the json module
      py::object json_loads = py::module::import("json").attr("loads");
      return json_loads(c->ToJSONStr());
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

  camera_base.def(
    "from_json",
    [](const ifm3d::CameraBase::Ptr& c, const py::dict& json)
    {
      // Convert the input JSON to string and load it
      py::object json_dumps = py::module::import("json").attr("dumps");
      c->FromJSONStr(json_dumps(json).cast<std::string>());
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