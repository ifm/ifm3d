// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_REMOVE_APP_H
#define IFM3D_TOOLS_REMOVE_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `remove` subcommand to the `ifm3d`
   * command-line utility.
   */

  class RemoveApp : public Command
  {
  public:
    ~RemoveApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string path;

  }; // end: class RemoveApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_REMOVE_APP_H
