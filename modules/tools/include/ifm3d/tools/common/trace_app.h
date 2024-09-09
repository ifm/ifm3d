// -*- c++ -*-
/*
 * Copyright (C) 2018 ifm syntron gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_TRACE_APP_H
#define IFM3D_TOOLS_TRACE_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `trace` subcommand to the `ifm3d`
   * command-line utility.
   */
  class TraceApp : public Command
  {
  public:
    ~TraceApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    int limit{0};

  }; // end: class TraceApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_TRACE_APP_H
