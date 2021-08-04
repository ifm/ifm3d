// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_FG_HZ_APP_H
#define IFM3D_TOOLS_FG_HZ_APP_H

#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/tools/fg/fg_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `hz` subcommand to the
   * `ifm3d` command-line utility.
   */
  class HzApp : public ifm3d::FgApp
  {
  public:
    HzApp(int argc, const char** argv, const std::string& name = "hz");
    int Run();
  };

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_FG_HZ_APP_H
