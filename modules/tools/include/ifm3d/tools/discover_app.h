// -*- c++ -*-
/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_TOOLS_DISCOVER_APP_H__
#define __IFM3D_TOOLS_DISCOVER_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `discover` subcommand to the `ifm3d` command-line
   * utility.
   */
  class DiscoverApp : public ifm3d::CmdLineApp
  {
  public:
    DiscoverApp(int argc, const char **argv, const std::string& name = "discover");
    int Run();
  }; // end: class DiscoverApp
} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_DISCOVER_APP_H__
