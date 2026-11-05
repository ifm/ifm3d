// -*- c++ -*-
/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_OVP8XX_APP_H
#define IFM3D_TOOLS_OVP8XX_APP_H
#pragma once

#include <CLI/CLI.hpp>
#include <ifm3d/tools/command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `ovpxxx` subcommand to the `ifm3d`
   * command-line utility. Also used to register the deprecated `ovp8xx`
   * alias (see MainCommand::CreateCommand).
   */
  class OVP8xx : public Command
  {
  public:
    explicit OVP8xx(std::string subcommand_name = "ovpxxx");
    OVP8xx(const OVP8xx&) = default;
    OVP8xx(OVP8xx&&) = delete;
    OVP8xx& operator=(const OVP8xx&) = default;
    OVP8xx& operator=(OVP8xx&&) = delete;
    ~OVP8xx() override;
    CLI::App* CreateCommand(CLI::App* parent) override;
    void Execute(CLI::App* app) override;
    bool CheckCompatibility() override;

  private:
    std::string subcommand_name_;

  }; // end: class OVP8xx

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_OVP8XX_APP_H
