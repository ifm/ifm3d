// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_CONFIG_APP_H
#define IFM3D_TOOLS_CONFIG_APP_H

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `config` subcommand to the `ifm3d`
   * command-line utility.
   *
   * `config` reads in a JSON description of the desired camera state and makes
   * a best effort attempt at having the hardware reflect the described state
   * of the JSON.
   */
  class ConfigApp : public ifm3d::CmdLineApp
  {
  public:
    ConfigApp(int argc, const char** argv, const std::string& name = "config");
    int Run() override;
  }; // end: class ConfigApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_CONFIG_APP_H
