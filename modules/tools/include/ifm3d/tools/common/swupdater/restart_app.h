/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SWUPDATER_RESTART_APP_H
#define IFM3D_TOOLS_SWUPDATER_RESTART_APP_H
#include <ifm3d/device/device.h>
#pragma once

#include <ifm3d/common/features.h>
#if defined(BUILD_MODULE_SWUPDATER)

#  include <ifm3d/swupdater/swupdater.h>
#  include <ifm3d/tools/command.hpp>
#  include <ifm3d/tools/main_command.hpp>

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

  private:
    bool _wait{false};
  }; // end: class RestartApp

  void reboot_device(ifm3d::Device::Ptr device,
                     ifm3d::SWUpdater::Ptr swupdater,
                     ifm3d::Device::BootMode mode,
                     bool wait);

} // end: namespace ifm3d

#endif
#endif // IFM3D_TOOLS_SWUPDATER_RESTART_APP_H
