// -*- c++ -*-
/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_STAT_APP_H
#define IFM3D_TOOLS_STAT_APP_H
#pragma once

#include <ifm3d/common/features.h>
#if defined(BUILD_MODULE_FRAMEGRABBER)

#  include <ifm3d/tools/command.hpp>
#  include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `stat` subcommand to the `ifm3d`
   * command-line utility.
   */

  class StatApp : public Command
  {
  public:
    StatApp() = default;
    StatApp(const StatApp&) = default;
    StatApp(StatApp&&) = delete;
    StatApp& operator=(const StatApp&) = default;
    StatApp& operator=(StatApp&&) = delete;
    ~StatApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: class StatApp
} // end: namespace ifm3d

#endif
#endif // IFM3D_TOOLS_STAT_APP_H
