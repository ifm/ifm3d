// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_REBOOT_APP_H
#define IFM3D_TOOLS_REBOOT_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>

namespace ifm3d
{
  /**
   * Concrete implementation of the `reboot` subcommand to the `ifm3d`
   * command-line utility.
   */
  class RebootApp : public Command
  {
  public:
    RebootApp() = default;
    RebootApp(const RebootApp&) = default;
    RebootApp(RebootApp&&) = delete;
    RebootApp& operator=(const RebootApp&) = default;
    RebootApp& operator=(RebootApp&&) = delete;
    ~RebootApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;
    bool CheckCompatibility() override;

  private:
    bool _recovery{false};
    bool _wait{false};

  }; // end: class RebootApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_REBOOT_APP_H
