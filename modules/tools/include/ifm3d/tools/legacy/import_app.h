// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_IMPORT_APP_H
#define IFM3D_TOOLS_IMPORT_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `import` subcommand to the `ifm3d`
   * command-line utility.
   *
   * `import` provides compatibility with Vision Assistant - it can consume
   * application or configurations exported by Vision Assistant.
   */
  class ImportApp : public Command
  {
  public:
    ~ImportApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string input_file{"-"};
    CLI::Option* config;
    bool global_config{false};
    bool network_config{false};
    bool app_config{false};

  }; // end: class ImportApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_IMPORT_APP_H
