// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_RESET_APP_H
#define IFM3D_TOOLS_RESET_APP_H

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `reset` subcommand to the `ifm3d`
   * command-line utility.
   */
  class ResetApp : public ifm3d::CmdLineApp
  {
  public:
    ResetApp(int argc, const char** argv, const std::string& name = "reset");
    int Run() override;
    bool CheckCompatibility() override;
  }; // end: class ResetApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_RESET_APP_H
