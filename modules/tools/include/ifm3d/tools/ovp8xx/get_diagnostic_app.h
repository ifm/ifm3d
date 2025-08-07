// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_GET_DIAGNOSTIC_APP_H
#define IFM3D_TOOLS_GET_DIAGNOSTIC_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `get` subcommand to the `ifm3d`
   * command-line utility.
   */

  class GetDiagnosticApp : public Command
  {
  public:
    GetDiagnosticApp() = default;
    GetDiagnosticApp(const GetDiagnosticApp&) = default;
    GetDiagnosticApp(GetDiagnosticApp&&) = delete;
    GetDiagnosticApp& operator=(const GetDiagnosticApp&) = default;
    GetDiagnosticApp& operator=(GetDiagnosticApp&&) = delete;
    ~GetDiagnosticApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    std::string filter_expression;
  }; // end: class GetDiagnosticApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_GET_DIAGNOSTIC_APP_H
