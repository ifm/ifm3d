// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_CONFIG_OVP8XX_APP_H
#define IFM3D_TOOLS_CONFIG_OVP8XX_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `config` group subcommand to the `ifm3d`
   * command-line utility.
   *
   * `config` reads in a JSON description of the desired camera state and makes
   * a best effort attempt at having the hardware reflect the described state
   * of the JSON.
   */

  class ConfigOvp8xxApp : public Command
  {
  public:
    ConfigOvp8xxApp() = default;
    ConfigOvp8xxApp(const ConfigOvp8xxApp&) = default;
    ConfigOvp8xxApp(ConfigOvp8xxApp&&) = delete;
    ConfigOvp8xxApp& operator=(const ConfigOvp8xxApp&) = default;
    ConfigOvp8xxApp& operator=(ConfigOvp8xxApp&&) = delete;
    ~ConfigOvp8xxApp() override;

    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: ConfigOvp8xxApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_CONFIG_OVP8XX_APP_H
