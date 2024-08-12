// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_DUMP_APP_H
#define IFM3D_TOOLS_DUMP_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `dump` subcommand to the `ifm3d`
   * command-line utility.
   *
   * `dump` serializes the camera state to JSON
   */
  class DumpApp : public Command
  {
  public:
    ~DumpApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::vector<std::string> paths;

  }; // end: class DumpApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_DUMP_APP_H
