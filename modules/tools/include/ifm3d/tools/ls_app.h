// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_TOOLS_LS_APP_H__
#define __IFM3D_TOOLS_LS_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `ls` subcommand to the `ifm3d` command-line
   * utility.
   */
  class LsApp : public ifm3d::CmdLineApp
  {
  public:
    LsApp(int argc, const char** argv, const std::string& name = "ls");
    int Run();
  }; // end: class LsApp
} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_LS_APP_H__
