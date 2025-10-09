// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_IMPORT_APP_H
#define IFM3D_TOOLS_IMPORT_APP_H
#pragma once
#pragma once

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
    ImportApp() = default;
    ImportApp(const ImportApp&) = default;
    ImportApp(ImportApp&&) = delete;
    ImportApp& operator=(const ImportApp&) = default;
    ImportApp& operator=(ImportApp&&) = delete;
    ~ImportApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: class ImportApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_IMPORT_APP_H
