// -*- c++ -*-
/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_STAT_APP_H
#define IFM3D_TOOLS_STAT_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `stat` subcommand to the `ifm3d`
   * command-line utility.
   */

  class StatApp : public Command
  {
  public:
    ~StatApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: class StatApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_STAT_APP_H
