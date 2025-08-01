/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

// Pybind must come after ifm3d because the pybind includes
// set some macros that conflict with the nlohmann json.hpp
// implementation on Windows builds
#include <string>

#include "ifm3d/device/version.h"
#include "ifm3d/device/device.h"
#include "bindings/future.h"
#include "ifm3d/fg/frame.h"
#include "util.hpp"

#include "bindings/device.h"
#include "bindings/legacy_device.h"
#include "bindings/logging.h"
#include "bindings/o3r.h"
#include "bindings/o3d.h"
#include "bindings/o3x.h"
#include "bindings/error.h"
#include "bindings/frame.h"
#include "bindings/framegrabber.h"
#if defined(BUILD_MODULE_SWUPDATER)
#  include "bindings/swupdater.h"
#endif
#include "bindings/semver.h"
#include "bindings/deserialize/deserialize.h"
#include <pybind11/detail/common.h>
#include <pybind11/pybind11.h>

PYBIND11_MODULE(ifm3dpy, m)
{
  m.doc() = R"(
    Bindings for the ifm3d Library

    **Variables**

    .. csv-table::

  )";

  // Module metadata
  ifm3d::add_attr(m,
                  "__version__",
                  std::to_string(IFM3D_VERSION_MAJOR) + "." +
                    std::to_string(IFM3D_VERSION_MINOR) + "." +
                    std::to_string(IFM3D_VERSION_PATCH) +
                    std::string(IFM3D_VERSION_TWEAK) +
                    std::string(IFM3D_VERSION_META),

                  "The ifm3d version.");

  ifm3d::add_attr(m, "__package__", "ifm3dpy", "The ifm3d package.");

  // Camera defaults
  ifm3d::add_attr(m,
                  "DEFAULT_IP",
                  ifm3d::DEFAULT_IP,
                  "The default IP to connect to.");
  ifm3d::add_attr(m,
                  "DEFAULT_XMLRPC_PORT",
                  ifm3d::DEFAULT_XMLRPC_PORT,
                  "The default XMLRPC port.");
  ifm3d::add_attr(m,
                  "DEFAULT_PASSWORD",
                  ifm3d::DEFAULT_PASSWORD,
                  "The default password.");

  // Constants to use for querying supported firmware versions
  ifm3d::add_attr(m,
                  "O3D_TIME_SUPPORT_MAJOR",
                  ifm3d::O3D_TIME_SUPPORT_MAJOR,
                  "Constant for querying for O3D time support.");
  ifm3d::add_attr(m,
                  "O3D_TIME_SUPPORT_MINOR",
                  ifm3d::O3D_TIME_SUPPORT_MINOR,
                  "Constant for querying for O3D time support.");
  ifm3d::add_attr(m,
                  "O3D_TIME_SUPPORT_PATCH",
                  ifm3d::O3D_TIME_SUPPORT_PATCH,
                  "Constant for querying for O3D time support.");

  ifm3d::add_attr(
    m,
    "O3D_TMP_PARAMS_SUPPORT_MAJOR",
    ifm3d::O3D_TMP_PARAMS_SUPPORT_MAJOR,
    "Constant for querying for O3D temporary parameter support.");
  ifm3d::add_attr(
    m,
    "O3D_TMP_PARAMS_SUPPORT_MINOR",
    ifm3d::O3D_TMP_PARAMS_SUPPORT_MINOR,
    "Constant for querying for O3D temporary parameter support.");
  ifm3d::add_attr(
    m,
    "O3D_TMP_PARAMS_SUPPORT_PATCH",
    ifm3d::O3D_TMP_PARAMS_SUPPORT_PATCH,
    "Constant for querying for O3D temporary parameter support.");

  ifm3d::add_attr(
    m,
    "O3D_INTRINSIC_PARAM_SUPPORT_MAJOR",
    ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MAJOR,
    "Constant for querying for O3D intrinsic parameter support.");
  ifm3d::add_attr(
    m,
    "O3D_INTRINSIC_PARAM_SUPPORT_MINOR",
    ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MINOR,
    "Constant for querying for O3D intrinsic parameter support.");
  ifm3d::add_attr(
    m,
    "O3D_INTRINSIC_PARAM_SUPPORT_PATCH",
    ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_PATCH,
    "Constant for querying for O3D intrinsic parameter support.");

  ifm3d::add_attr(
    m,
    "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR",
    ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR,
    "Constant for querying for O3D inverse intrinsic parameter support.");
  ifm3d::add_attr(
    m,
    "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR",
    ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR,
    "Constant for querying for O3D inverse intrinsic parameter support.");
  ifm3d::add_attr(
    m,
    "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH",
    ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH,
    "Constant for querying for O3D inverse intrinsic parameter support.");

  bind_future<void>(m,
                    "Awaitable",
                    "Provides a mechanism to wait for completion of a task",
                    "None");

  auto logging_module = m.def_submodule(
    "logging",
    R"(Provides access for configuring the logging facilities of ifm3d.)");
  bind_logging(logging_module);
  bind_numpy(m);
  auto device_module = m.def_submodule(
    "device",
    R"(Provides an implementation of the XMLRPC protocol for configuring the camera and pmd imager settings.)");
  bind_semver(device_module);
  bind_error(device_module);
  bind_device(device_module);
  bind_o3r(device_module);
  bind_legacy_device(device_module);
  bind_o3d(device_module);
  bind_o3x(device_module);

  auto framegrabber_module = m.def_submodule(
    "framegrabber",
    R"(Provides an implementation of the PCIC protocol for streaming pixel data and triggered image acquisition.)");
  bind_frame(framegrabber_module, m);
  bind_future<ifm3d::Frame::Ptr>(
    m,
    "FrameAwaitable",
    "Provides a mechanism to access the frame object",
    "Frame");
  bind_framegrabber(framegrabber_module);

#if defined(BUILD_MODULE_SWUPDATER)
  auto swupdater_module = m.def_submodule(
    "swupdater",
    R"(Provides utilities for managing the SWUpdate subsystem of the camera.)");
  bind_swupdater(swupdater_module);
#endif

  auto deserializer_module = m.def_submodule(
    "deserialize",
    R"(Provides definitions and functions for deserializing structs sent over PCIC)");
  bind_deserialize_struct(deserializer_module);

  // deprecated aliases for backwards compatibility, will removed at some point
  // in the future
  m.attr("SemVer") = device_module.attr("SemVer");
  m.attr("Error") = device_module.attr("Error");
  m.attr("Device") = device_module.attr("Device");
  m.attr("LegacyDevice") = device_module.attr("LegacyDevice");
  m.attr("O3R") = device_module.attr("O3R");
  m.attr("FrameGrabber") = framegrabber_module.attr("FrameGrabber");
  m.attr("Frame") = framegrabber_module.attr("Frame");
  m.attr("buffer_id") = framegrabber_module.attr("buffer_id");
#if defined(BUILD_MODULE_SWUPDATER)
  m.attr("SWUpdater") = swupdater_module.attr("SWUpdater");
#endif
}
