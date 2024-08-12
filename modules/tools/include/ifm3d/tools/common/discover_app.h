// -*- c++ -*-
/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_DISCOVER_APP_H
#define IFM3D_TOOLS_DISCOVER_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `discover` subcommand to the `ifm3d`
   * command-line utility.
   */
  class DiscoverApp : public Command
  {
  public:
    ~DiscoverApp();
    virtual void Execute(CLI::App* app) override;
    std::string GetDeviceType(const ifm3d::Device::Ptr& cam);
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: class

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_DISCOVER_APP_H
