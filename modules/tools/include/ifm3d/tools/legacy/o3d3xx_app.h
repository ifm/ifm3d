// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_O3D3XX_APP_H
#define IFM3D_TOOLS_O3D3XX_APP_H

#include <string>
#include <CLI/CLI.hpp>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <ifm3d/tools.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `o3d3xx` subcommand to the `ifm3d`
   * command-line utility.
   */
  class O3D3XX : public Command
  {
  public:
    ~O3D3XX();
    virtual CLI::App* CreateCommand(CLI::App* parent) override;
    virtual void Execute(CLI::App* app) override;
    virtual bool CheckCompatibility() override;

  }; // end: class O3D3XX

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_O3D3XX_APP_H
