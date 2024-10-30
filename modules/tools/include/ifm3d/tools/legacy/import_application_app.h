// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_IMPORT_APPLICATION_APP_H
#define IFM3D_TOOLS_IMPORT_APPLICATION_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `import app` subcommand to the `ifm3d`
   * command-line utility.
   */
  class ImportApplicationApp : public Command
  {
  public:
    ~ImportApplicationApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string input_file{"-"};

  }; // end: class ImportApplicationApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_IMPORT_APPLICATION_APP_H