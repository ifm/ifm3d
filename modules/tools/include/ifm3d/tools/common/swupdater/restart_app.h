/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_RESTART_APP_H
#define IFM3D_TOOLS_SWUPDATER_RESTART_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `swupdate restart` subcommand to the
   * `ifm3d` command-line utility.
   */
  class RestartApp : public Command
  {
  public:
    RestartApp() = default;
    RestartApp(const RestartApp&) = default;
    RestartApp(RestartApp&&) = delete;
    RestartApp& operator=(const RestartApp&) = default;
    RestartApp& operator=(RestartApp&&) = delete;
    ~RestartApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;
    bool CheckCompatibility() override;
  }; // end: class RestartApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SWUPDATER_RESTART_APP_H
