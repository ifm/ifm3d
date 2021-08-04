// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_CP_APP_H
#define IFM3D_TOOLS_CP_APP_H

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `cp` subcommand to the `ifm3d` command-line
   * utility.
   */
  class CpApp : public ifm3d::CmdLineApp
  {
  public:
    CpApp(int argc, const char** argv, const std::string& name = "cp");
    int Run() override;
    bool CheckCompatibility() override;
  }; // end: class CpApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_CP_APP_H
