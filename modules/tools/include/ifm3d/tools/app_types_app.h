// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_TOOLS_APP_TYPES_APP_H__
#define __IFM3D_TOOLS_APP_TYPES_APP_H__

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `app-types` subcommand to the `ifm3d`
   * command-line utility.
   */
  class AppTypesApp : public ifm3d::CmdLineApp
  {
  public:
    AppTypesApp(int argc,
                const char** argv,
                const std::string& name = "app-types");
    int Run();
  }; // end: class AppTypeApp
} // end: namespace ifm3d

#endif // __IFM3D_TOOLS_APP_TYPES_APP_H__
