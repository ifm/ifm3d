/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_FLASH_SW_APP_H
#define IFM3D_TOOLS_SWUPDATER_FLASH_SW_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `swupdate flash` subcommand to the
   * `ifm3d` command-line utility.
   */
  class FlashSWApp : public Command
  {
  public:
    FlashSWApp() = default;
    FlashSWApp(const FlashSWApp&) = default;
    FlashSWApp(FlashSWApp&&) = delete;
    FlashSWApp& operator=(const FlashSWApp&) = default;
    FlashSWApp& operator=(FlashSWApp&&) = delete;
    ~FlashSWApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    std::string swu_file{"-"};
    bool quiet{false};
    unsigned int timeout{300};

  }; // end: class FlashSWApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SWUPDATER_FLASH_SW_APP_H
