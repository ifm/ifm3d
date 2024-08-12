// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_GET_DIAGNOSTIC_APP_H
#define IFM3D_TOOLS_GET_DIAGNOSTIC_APP_H

#include <string>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `get` subcommand to the `ifm3d`
   * command-line utility.
   */

  class GetDiagnosticApp : public Command
  {
  public:
    ~GetDiagnosticApp();
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: class GetDiagnosticApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_GET_DIAGNOSTIC_APP_H
