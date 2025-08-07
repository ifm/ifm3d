// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_IMPORT_APPLICATION_APP_H
#define IFM3D_TOOLS_IMPORT_APPLICATION_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `import app` subcommand to the `ifm3d`
   * command-line utility.
   */
  class ImportApplicationApp : public Command
  {
  public:
    ImportApplicationApp() = default;
    ImportApplicationApp(const ImportApplicationApp&) = default;
    ImportApplicationApp(ImportApplicationApp&&) = delete;
    ImportApplicationApp& operator=(const ImportApplicationApp&) = default;
    ImportApplicationApp& operator=(ImportApplicationApp&&) = delete;
    ~ImportApplicationApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    std::string input_file{"-"};

  }; // end: class ImportApplicationApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_IMPORT_APPLICATION_APP_H
