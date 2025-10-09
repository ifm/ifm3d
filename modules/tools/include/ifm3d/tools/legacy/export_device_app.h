// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_EXPORT_DEVICE_APP_H
#define IFM3D_TOOLS_EXPORT_DEVICE_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `export device` subcommand to the `ifm3d`
   * command-line utility.
   *
   * `export` provides compatibility with Vision Assistant - it can produce
   * exported applications/configurations that can be consumed by Vision
   * Assistant.
   */
  class ExportDeviceApp : public Command
  {
  public:
    ExportDeviceApp() = default;
    ExportDeviceApp(const ExportDeviceApp&) = default;
    ExportDeviceApp(ExportDeviceApp&&) = delete;
    ExportDeviceApp& operator=(const ExportDeviceApp&) = default;
    ExportDeviceApp& operator=(ExportDeviceApp&&) = delete;
    ~ExportDeviceApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    std::string output_file{"-"};

  }; // end: class ExportDeviceApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_EXPORT_DEVICE_APP_H
