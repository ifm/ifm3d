// -*- c++ -*-
/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_O3CXXX_APP_H
#define IFM3D_TOOLS_O3CXXX_APP_H
#pragma once

#include <CLI/CLI.hpp>
#include <ifm3d/tools/command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `o3cxxx` subcommand to the `ifm3d`
   * command-line utility. Shares implementation with ovp8xx.
   */
  class O3Cxxx : public Command
  {
  public:
    O3Cxxx() = default;
    O3Cxxx(const O3Cxxx&) = default;
    O3Cxxx(O3Cxxx&&) = delete;
    O3Cxxx& operator=(const O3Cxxx&) = default;
    O3Cxxx& operator=(O3Cxxx&&) = delete;
    ~O3Cxxx() override;
    CLI::App* CreateCommand(CLI::App* parent) override;
    void Execute(CLI::App* app) override;
    bool CheckCompatibility() override;

  }; // end: class O3Cxxx

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_O3CXXX_APP_H
