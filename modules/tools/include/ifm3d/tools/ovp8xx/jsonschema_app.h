// -*- c++ -*-
/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_JSONSCHEMA_APP_H
#define IFM3D_TOOLS_JSONSCHEMA_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `jsonschema` subcommand to the `ifm3d`
   * command-line utility.
   */
  class JSONSchemaApp : public Command
  {
  public:
    JSONSchemaApp() = default;
    JSONSchemaApp(const JSONSchemaApp&) = default;
    JSONSchemaApp(JSONSchemaApp&&) = delete;
    JSONSchemaApp& operator=(const JSONSchemaApp&) = default;
    JSONSchemaApp& operator=(JSONSchemaApp&&) = delete;
    ~JSONSchemaApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

  }; // end: class JSONSchemaApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_JSONSCHEMA_APP_H
