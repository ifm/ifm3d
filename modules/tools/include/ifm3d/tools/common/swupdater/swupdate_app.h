/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
#define IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <ifm3d/swupdater.h>

namespace ifm3d
{
  class SWUpdater;

  /**
   * Concrete implementation of the `swupdate` subcommand to the
   * `ifm3d` command-line utility.
   */
  class SWUpdateApp : public Command
  {
  public:
    SWUpdateApp(std::optional<ifm3d::Device::swu_version> force_swu_version =
                  std::nullopt);
    ~SWUpdateApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;
    virtual bool CheckCompatibility() override;

    std::shared_ptr<ifm3d::SWUpdater> CreateSWUpdater(
      bool quiet = false,
      std::uint16_t swupdate_recovery_port = ifm3d::SWUPDATER_RECOVERY_PORT);

    bool detect{false};
    CLI::App* subcmd_flash{};
    CLI::App* subcmd_restart{};

  private:
    std::optional<ifm3d::Device::swu_version> force_swu_version;

  }; // end: class SWUpdateApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
