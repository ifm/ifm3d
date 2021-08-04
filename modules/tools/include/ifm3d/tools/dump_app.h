// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_DUMP_APP_H
#define IFM3D_TOOLS_DUMP_APP_H

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
    int Run() override;
  }; // end: class DumpApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_DUMP_APP_H
