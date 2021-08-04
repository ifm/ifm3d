// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_REBOOT_APP_H
#define IFM3D_TOOLS_REBOOT_APP_H

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `reboot` subcommand to the `ifm3d`
   * command-line utility.
   */
  class RebootApp : public ifm3d::CmdLineApp
  {
  public:
    RebootApp(int argc, const char** argv, const std::string& name = "reboot");
    int Run() override;
  }; // end: class RebootApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_REBOOT_APP_H
