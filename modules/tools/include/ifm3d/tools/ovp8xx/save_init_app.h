// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SAVE_INIT_APP_H
#define IFM3D_TOOLS_SAVE_INIT_APP_H
#pragma once

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `saveInit` subcommand to the `ifm3d`
   * command-line utility.
   */

  class SaveInitApp : public Command
  {
  public:
    ~SaveInitApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::vector<std::string> paths = {};

  }; // end: class SaveInitApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SAVE_INIT_APP_H
