// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_RESET_OVP8XX_APP_H
#define IFM3D_TOOLS_RESET_OVP8XX_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `ovp8xx reset` subcommand to the `ifm3d`
   * command-line utility.
   */

  class ResetOvp8xxApp : public Command
  {
  public:
    ResetOvp8xxApp() = default;
    ResetOvp8xxApp(const ResetOvp8xxApp&) = default;
    ResetOvp8xxApp(ResetOvp8xxApp&&) = delete;
    ResetOvp8xxApp& operator=(const ResetOvp8xxApp&) = default;
    ResetOvp8xxApp& operator=(ResetOvp8xxApp&&) = delete;
    ~ResetOvp8xxApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    std::string path;

  }; // end: class ResetOvp8xxApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_RESET_OVP8XX_APP_H
