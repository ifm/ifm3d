// -*- c++ -*-
/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_DIAGNOSTIC_APP_H
#define IFM3D_TOOLS_DIAGNOSTIC_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `diagnostic` subcommand to the `ifm3d`
   * command-line utility.
   */

  class DiagnosticApp : public Command
  {
  public:
    ~DiagnosticApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: class DiagnosticApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_DIAGNOSTIC_APP_H
