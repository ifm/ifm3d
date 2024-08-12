/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_FLASH_SW_APP_H
#define IFM3D_TOOLS_SWUPDATER_FLASH_SW_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `swupdate flash` subcommand to the
   * `ifm3d` command-line utility.
   */
  class FlashSWApp : public Command
  {
  public:
    ~FlashSWApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string swu_file{"-"};
    bool quiet{false};
    unsigned int timeout{300};

  }; // end: class FlashSWApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SWUPDATER_FLASH_SW_APP_H
