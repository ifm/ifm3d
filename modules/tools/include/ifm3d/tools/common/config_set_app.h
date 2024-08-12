// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_CONFIG_SET_APP_H
#define IFM3D_TOOLS_CONFIG_SET_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `config (O3D/O3X) or set (ovp8xx)`
   * subcommand to the `ifm3d` command-line utility.
   *
   * `config` reads in a JSON description of the desired camera state and makes
   * a best effort attempt at having the hardware reflect the described state
   * of the JSON.
   */

  class ConfigSetApp : public Command
  {
  public:
    ~ConfigSetApp();

    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string config_file{"-"};
    std::string path = "";

  }; // end: ConfigSetApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_CONFIG_SET_APP_H
