// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_EXPORT_APPLICATION_APP_H
#define IFM3D_TOOLS_EXPORT_APPLICATION_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `export app` subcommand to the `ifm3d`
   * command-line utility.
   */
  class ExportApplicationApp : public Command
  {
  public:
    ExportApplicationApp() = default;
    ExportApplicationApp(const ExportApplicationApp&) = default;
    ExportApplicationApp(ExportApplicationApp&&) = delete;
    ExportApplicationApp& operator=(const ExportApplicationApp&) = default;
    ExportApplicationApp& operator=(ExportApplicationApp&&) = delete;
    ~ExportApplicationApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    int application_index{0};
    std::string output_file{"-"};

  }; // end: class ExportApplicationApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_EXPORT_APPLICATION_APP_H
