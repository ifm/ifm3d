/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
#define IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
#pragma once

#include <ifm3d/common/features.h>
#if defined(BUILD_MODULE_SWUPDATER)

#  include <ifm3d/swupdater.h>
#  include <ifm3d/tools/command.hpp>
#  include <ifm3d/tools/main_command.hpp>

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
    SWUpdateApp(std::optional<ifm3d::Device::SWUVersion> force_swu_version);
    SWUpdateApp(const SWUpdateApp&) = default;
    SWUpdateApp(SWUpdateApp&&) = delete;
    SWUpdateApp& operator=(const SWUpdateApp&) = default;
    SWUpdateApp& operator=(SWUpdateApp&&) = delete;
    ~SWUpdateApp() override = default;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;
    bool CheckCompatibility() override;

    std::shared_ptr<ifm3d::SWUpdater> CreateSWUpdater(
      bool quiet = false,
      std::uint16_t swupdate_recovery_port = ifm3d::SWUPDATER_RECOVERY_PORT);

    bool detect{false};
    CLI::App* subcmd_flash{};
    CLI::App* subcmd_restart{};

  private:
    std::optional<ifm3d::Device::SWUVersion> _force_swu_version;

  }; // end: class SWUpdateApp

} // end: namespace ifm3d

#endif
#endif // IFM3D_TOOLS_SWUPDATER_SWUPDATE_APP_H
