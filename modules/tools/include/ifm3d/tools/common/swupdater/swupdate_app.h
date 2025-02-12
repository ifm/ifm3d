/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
#define IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `swupdate` subcommand to the
   * `ifm3d` command-line utility.
   */
  class SWUpdateApp : public Command
  {
  public:
    ~SWUpdateApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;
    virtual bool CheckCompatibility() override;

    bool detect{false};
    CLI::App* subcmd_flash;
    CLI::App* subcmd_restart;

  }; // end: class SWUpdateApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
