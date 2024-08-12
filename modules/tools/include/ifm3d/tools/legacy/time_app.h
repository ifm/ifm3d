// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_TIME_APP_H
#define IFM3D_TOOLS_TIME_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementatoin of the `time` subcommand to the `ifm3d`
   * command-line utility.
   */
  class TimeApp : public Command
  {
  public:
    ~TimeApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    int epoch{-1};
    CLI::Option* opt_epoch;

  }; // end: class TimeApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_TIME_APP_H
