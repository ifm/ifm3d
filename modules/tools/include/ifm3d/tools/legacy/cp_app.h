// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_CP_APP_H
#define IFM3D_TOOLS_CP_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `cp` subcommand to the `ifm3d` command-line
   * utility.
   */

  class CpApp : public Command
  {
  public:
    ~CpApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    int index{-1};

  }; // end: class CpApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_CP_APP_H
