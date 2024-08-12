/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_RESTART_APP_H
#define IFM3D_TOOLS_SWUPDATER_RESTART_APP_H

#include <string>
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
    ~RestartApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;
    virtual bool CheckCompatibility() override;
  }; // end: class RestartApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SWUPDATER_RESTART_APP_H
