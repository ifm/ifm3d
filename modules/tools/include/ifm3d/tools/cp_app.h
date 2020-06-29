// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_TOOLS_CP_APP_H__
#define __IFM3D_TOOLS_CP_APP_H__

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
    int Run();
  }; // end: class CpApp

} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_CP_APP_H__
