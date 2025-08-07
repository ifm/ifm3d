// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_O3X_APP_H
#define IFM3D_TOOLS_O3X_APP_H
#pragma once

#include <CLI/CLI.hpp>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `o3x1xx and o3x2xx` subcommand to the
   * `ifm3d` command-line utility.
   */
  class O3X1XX_O3X2XX : public Command // NOLINT(readability-identifier-naming)
  {
  public:
    O3X1XX_O3X2XX() = default;
    O3X1XX_O3X2XX(const O3X1XX_O3X2XX&) = default;
    O3X1XX_O3X2XX(O3X1XX_O3X2XX&&) = delete;
    O3X1XX_O3X2XX& operator=(const O3X1XX_O3X2XX&) = default;
    O3X1XX_O3X2XX& operator=(O3X1XX_O3X2XX&&) = delete;
    ~O3X1XX_O3X2XX() override;
    CLI::App* CreateCommand(CLI::App* parent) override;
    void Execute(CLI::App* app) override;
    bool CheckCompatibility() override;

  }; // end: class O3X1XX_O3X2XX

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_O3X_APP_H
