// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_IMPORT_DEVICE_APP_H
#define IFM3D_TOOLS_IMPORT_DEVICE_APP_H
#pragma once

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `import device` subcommand to the `ifm3d`
   * command-line utility.
   */
  class ImportDeviceApp : public Command
  {
  public:
    ~ImportDeviceApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string input_file{"-"};
    bool no_global_config;
    bool no_network_config;
    bool no_app_config;

  }; // end: class ImportDeviceApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_IMPORT_DEVICE_APP_H
