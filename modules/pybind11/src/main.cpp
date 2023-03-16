/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/device.h>
#include <ifm3d/fg.h>
#include <ifm3d/tools/cmdline_app.h>

// Pybind must come after ifm3d because the pybind includes
// set some macros that conflict with the nlohmann json.hpp
// implementation on Windows builds
#include <pybind11/pybind11.h>
#include <pybind11/iostream.h>
#include <pybind11/functional.h>

#include "util.hpp"

#include "bindings/device.h"
#include "bindings/legacy_device.h"
#include "bindings/o3r.h"
#include "bindings/error.h"
#include "bindings/frame.h"
#include "bindings/framegrabber.h"
#include "bindings/swupdater.h"
#include "bindings/semver.h"
#include "bindings/deserialize/deserialize.h"

namespace py = pybind11;

/* run the command of ifm3d tools */
std::tuple<int, std::string>
run(py::list inlist, const bool& std_out = false)
{
  const size_t argc = static_cast<size_t>(inlist.size());
  auto argv = std::make_unique<const char*[]>(argc);

  for (size_t i = 0; i < argc; ++i)
    {
      argv.get()[i] = PyUnicode_AsUTF8(inlist[i].ptr());
    }
  if (std_out)
    {
      py::scoped_ostream_redirect stream(
        std::cout,                               // std::ostream&
        py::module::import("sys").attr("stdout") // Python output

      );
      const int ret = ifm3d::CmdLineApp::Execute(argc, argv.get());
      return std::tuple<int, std::string>(ret, "");
    }
  else
    {
      std::stringstream buffer;
      std::cout.rdbuf(buffer.rdbuf());
      const int ret = ifm3d::CmdLineApp::Execute(argc, argv.get());
      return std::tuple<int, std::string>(ret, buffer.str());
    }
}

PYBIND11_MODULE(ifm3dpy, m)
{
  m.def(
    "_run_cmdtool",
    []() {
      py::list argv = py::module::import("sys").attr("argv");
      run(argv, true);
    },
    "Entry point for the ifm3dpy console application");

  m.def(
    "run",
    [](py::list argv) -> std::tuple<int, std::string> {
      /* check if ifm3dpy is first element if not insert */
      if (argv[0].cast<std::string>() != "ifm3dpy")
        {
          argv.insert(0, "ifm3dpy");
        }
      return run(argv);
    },
    R"(
        This function provides python application interface to run command line tool

        Parameters
        ----------
        argv : py::list
            command line parameter in the list. e.g. to call a 'ls' command
            ['ls', '--ip=192.168.0.69']

        Returns
        -------
        Tuple(int,string): py::tuple(int,string)
            execution state and output string.
      )");

  m.doc() = R"(
    Bindings for the ifm3d Library

    **Variables**

    .. csv-table::

  )";

  auto add_attr = [&m](const std::string& name,
                       const auto& value,
                       const std::string& doc = "") {
    m.attr(name.c_str()) = value;
    m.doc() =
      m.doc().cast<std::string>() + "     \"" + name + "\", \"" + doc + "\"\n";
  };

  // Module metadata
  add_attr("__version__",
           std::to_string(IFM3D_VERSION_MAJOR) + "." +
             std::to_string(IFM3D_VERSION_MINOR) + "." +
             std::to_string(IFM3D_VERSION_PATCH) +
             std::string(IFM3D_VERSION_TWEAK) +
             std::string(IFM3D_VERSION_META),

           "The ifm3d version.");

  add_attr("__package__", "ifm3dpy", "The ifm3d package.");

  // Camera defaults
  add_attr("DEFAULT_IP", ifm3d::DEFAULT_IP, "The default IP to connect to.");
  add_attr("DEFAULT_XMLRPC_PORT",
           ifm3d::DEFAULT_XMLRPC_PORT,
           "The default XMLRPC port.");
  add_attr("DEFAULT_PASSWORD",
           ifm3d::DEFAULT_PASSWORD,
           "The default password.");

  // Constants to use for querying supported firmware versions
  add_attr("O3D_TIME_SUPPORT_MAJOR",
           ifm3d::O3D_TIME_SUPPORT_MAJOR,
           "Constant for querying for O3D time support.");
  add_attr("O3D_TIME_SUPPORT_MINOR",
           ifm3d::O3D_TIME_SUPPORT_MINOR,
           "Constant for querying for O3D time support.");
  add_attr("O3D_TIME_SUPPORT_PATCH",
           ifm3d::O3D_TIME_SUPPORT_PATCH,
           "Constant for querying for O3D time support.");

  add_attr("O3D_TMP_PARAMS_SUPPORT_MAJOR",
           ifm3d::O3D_TMP_PARAMS_SUPPORT_MAJOR,
           "Constant for querying for O3D temporary parameter support.");
  add_attr("O3D_TMP_PARAMS_SUPPORT_MINOR",
           ifm3d::O3D_TMP_PARAMS_SUPPORT_MINOR,
           "Constant for querying for O3D temporary parameter support.");
  add_attr("O3D_TMP_PARAMS_SUPPORT_PATCH",
           ifm3d::O3D_TMP_PARAMS_SUPPORT_PATCH,
           "Constant for querying for O3D temporary parameter support.");

  add_attr("O3D_INTRINSIC_PARAM_SUPPORT_MAJOR",
           ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MAJOR,
           "Constant for querying for O3D intrinsic parameter support.");
  add_attr("O3D_INTRINSIC_PARAM_SUPPORT_MINOR",
           ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MINOR,
           "Constant for querying for O3D intrinsic parameter support.");
  add_attr("O3D_INTRINSIC_PARAM_SUPPORT_PATCH",
           ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_PATCH,
           "Constant for querying for O3D intrinsic parameter support.");

  add_attr(
    "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR",
    ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR,
    "Constant for querying for O3D inverse intrinsic parameter support.");
  add_attr(
    "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR",
    ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR,
    "Constant for querying for O3D inverse intrinsic parameter support.");
  add_attr(
    "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH",
    ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH,
    "Constant for querying for O3D inverse intrinsic parameter support.");

  bind_future<ifm3d::Frame::Ptr>(
    m,
    "FrameAwaitable",
    "Provides a mechanism to access the frame object");
  bind_future<void>(m,
                    "Awaitable",
                    "Provides a mechanism to wait for completion of a task");

  auto device_module = m.def_submodule(
    "device",
    R"(Provides an implementation of the XMLRPC protocol for configuring the camera and pmd imager settings.)");
  bind_semver(device_module);
  bind_error(device_module);
  bind_device(device_module);
  bind_legacy_device(device_module);
  bind_o3r(device_module);

  auto framegrabber_module = m.def_submodule(
    "framegrabber",
    R"(Provides an implementation of the PCIC protocol for streaming pixel data and triggered image acquisition.)");
  bind_frame(framegrabber_module);
  bind_framegrabber(framegrabber_module);

  auto swupdater_module = m.def_submodule(
    "swupdater",
    R"(Provides utilities for managing the SWUpdate subsystem of the camera.)");
  bind_swupdater(swupdater_module);

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
  m.attr("SWUpdater") = swupdater_module.attr("SWUpdater");
}
