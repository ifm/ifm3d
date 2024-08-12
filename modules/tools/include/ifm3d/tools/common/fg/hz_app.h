// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_FG_HZ_APP_H
#define IFM3D_TOOLS_FG_HZ_APP_H

#include <string>
#include <CLI/CLI.hpp>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <ifm3d/tools/tools_export.h>

namespace ifm3d
{
  extern IFM3D_TOOLS_EXPORT const int FG_TIMEOUT;
  /**
   * Concrete implementation of the `hz` subcommand to the
   * `ifm3d` command-line utility.
   */

  class HzApp : public Command
  {
  public:
    ~HzApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    unsigned short pcic_port{(unsigned short)ifm3d::DEFAULT_PCIC_PORT};
    int nframes{10};
    int nruns{1};
    bool sw_trigger{false};

  }; // end: class HzApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_FG_HZ_APP_H
