// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_LS_APP_H
#define IFM3D_TOOLS_LS_APP_H
#pragma once

#include <CLI/CLI.hpp>
#include <ifm3d/device.h>
#include <ifm3d/device/device.h>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `ls` subcommand to the `ifm3d` command-line
   * utility.
   */
  class LsApp : public Command
  {
  public:
    LsApp() = default;
    LsApp(const LsApp&) = default;
    LsApp(LsApp&&) = delete;
    LsApp& operator=(const LsApp&) = default;
    LsApp& operator=(LsApp&&) = delete;
    ~LsApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: class LsApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_LS_APP_H
