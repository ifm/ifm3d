/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_SWUPDATER
#define IFM3D_PYBIND_BINDING_SWUPDATER

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <ifm3d/swupdater/swupdater.h>

void
bind_swupdater(pybind11::module_& m)
{
  // clang-format off

  py::class_<ifm3d::SWUpdater, ifm3d::SWUpdater::Ptr> swupdater(
    m, 
    "SWUpdater",
    R"(
      Class for managing an instance of an swupdater
    )"
  );

  swupdater.def(
    py::init([](ifm3d::Device::Ptr cam, const std::optional<ifm3d::SWUpdater::FlashStatusCb>& cb, const std::uint16_t swupdate_recovery_port) {
      return std::make_shared<ifm3d::SWUpdater>(cam, cb ? cb.value() : [](float progress,const std::string& message)->void {}, swupdate_recovery_port);
    }),
    py::arg("cam"),
    py::arg("cb") = std::nullopt,
    py::arg("swupdate_recovery_port") = ifm3d::SWUPDATER_RECOVERY_PORT,
    R"(
      Constructor

      Parameters
      ----------
      cam : ifm3dpy.Camera
          The camera instance to grab frames from

      cb : callback function  with parameters (float, string)
          A  function for progress of the update

      swupdate_recovery_port : uint16_t
          The Swupdate communication port
      )"
    );

  swupdater.def(
    "reboot_to_recovery",
    &ifm3d::SWUpdater::RebootToRecovery,
    R"(
      Reboot the device in Recovery Modes
    )"
  );

  swupdater.def(
    "wait_for_recovery",
    &ifm3d::SWUpdater::WaitForRecovery,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("timeout_millis") = 0,
    R"(
      Waits and check for device in recovery mode
      till timeout

      Parameter
      ---------

      timeout_millis : long
          A timeout value in milliseconds

      Returns
      -------
      bool
          True if device in the recovery mode
    )"
  );

  swupdater.def(
    "reboot_to_productive",
    &ifm3d::SWUpdater::RebootToProductive,
    R"(
      Reboot the device in productive mode
    )"
  );

  swupdater.def(
    "wait_for_productive",
    &ifm3d::SWUpdater::WaitForProductive,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("timeout_millis") = 0,
    R"(
      Waits and check for device in productive mode
      till timeout

      Parameter
      ---------

      timeout_millis : long
          A timeout value in milliseconds

      Returns
      -------
      bool
          True if device in the productive mode
    )"
  );

  swupdater.def(
    "flash_firmware",
    &ifm3d::SWUpdater::FlashFirmware,
    py::call_guard<py::gil_scoped_release>(),
    py::arg("swu_file"),
    py::arg("timeout_millis") = 0,
    R"(
      Flash the firmware .swu file to the device

      Parameter
      ---------

      swu_file : string
         A .swu file to flash on the device

      timeout_millis : long
         A timeout value in milliseconds

      Returns
      -------
      bool
          True if firmware upload is successful
    )"
  );

  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_SWUPDATER