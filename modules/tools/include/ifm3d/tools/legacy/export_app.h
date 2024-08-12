// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_EXPORT_APP_H
#define IFM3D_TOOLS_EXPORT_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `export` subcommand to the `ifm3d`
   * command-line utility.
   *
   * `export` provides compatibility with Vision Assistant - it can produce
   * exported applications/configurations that can be consumed by Vision
   * Assistant.
   */
  class ExportApp : public Command
  {
  public:
    ~ExportApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string output_file{"-"};
    int application_index{-1};

  }; // end: class ExportApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_EXPORT_APP_H
