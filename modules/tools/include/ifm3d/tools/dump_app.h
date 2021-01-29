// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_TOOLS_DUMP_APP_H__
#define __IFM3D_TOOLS_DUMP_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `dump` subcommand to the `ifm3d`
   * command-line utility.
   *
   * `dump` serializes the camera state to JSON
   */
  class DumpApp : public ifm3d::CmdLineApp
  {
  public:
    DumpApp(int argc, const char** argv, const std::string& name = "dump");
    int Run();
  }; // end: class DumpApp

} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_DUMP_APP_H__
