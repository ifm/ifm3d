// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_GET_FILTER_SCHEMA_APP_H
#define IFM3D_TOOLS_GET_FILTER_SCHEMA_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `getFilterSchema` subcommand to the `ifm3d`
   * command-line utility.
   */

  class GetFilterSchemaApp : public Command
  {
  public:
    GetFilterSchemaApp() = default;
    GetFilterSchemaApp(const GetFilterSchemaApp&) = default;
    GetFilterSchemaApp(GetFilterSchemaApp&&) = delete;
    GetFilterSchemaApp& operator=(const GetFilterSchemaApp&) = default;
    GetFilterSchemaApp& operator=(GetFilterSchemaApp&&) = delete;
    ~GetFilterSchemaApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: class GetFilterSchemaApp
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_GET_FILTER_SCHEMA_APP_H
