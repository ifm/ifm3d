// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SAVE_INIT_APP_H
#define IFM3D_TOOLS_SAVE_INIT_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `saveInit` subcommand to the `ifm3d`
   * command-line utility.
   */

  class SaveInitApp : public Command
  {
  public:
    SaveInitApp() = default;
    SaveInitApp(const SaveInitApp&) = default;
    SaveInitApp(SaveInitApp&&) = delete;
    SaveInitApp& operator=(const SaveInitApp&) = default;
    SaveInitApp& operator=(SaveInitApp&&) = delete;
    ~SaveInitApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    std::vector<std::string> paths;

  }; // end: class SaveInitApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SAVE_INIT_APP_H
