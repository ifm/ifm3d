// -*- c++ -*-
/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_DISCOVER_APP_H
#define IFM3D_TOOLS_DISCOVER_APP_H
#pragma once

#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `discover` subcommand to the `ifm3d`
   * command-line utility.
   */
  class DiscoverApp : public Command
  {
  public:
    DiscoverApp() = default;
    DiscoverApp(const DiscoverApp&) = default;
    DiscoverApp(DiscoverApp&&) = delete;
    DiscoverApp& operator=(const DiscoverApp&) = default;
    DiscoverApp& operator=(DiscoverApp&&) = delete;
    ~DiscoverApp() override;
    void Execute(CLI::App* app) override;
    std::string GetDeviceType(const ifm3d::Device::Ptr& cam);
    CLI::App* CreateCommand(CLI::App* parent) override;
    bool CheckCompatibility() override;

    CLI::App* subcmd_set_temp_ip{};
  }; // end: class

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_DISCOVER_APP_H
