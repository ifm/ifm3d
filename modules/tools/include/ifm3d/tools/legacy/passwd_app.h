// -*- c++ -*-
/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_PASSWD_APP_H
#define IFM3D_TOOLS_PASSWD_APP_H
#pragma once

#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `passwd` subcommand to the `ifm3d`
   * command-line utility.
   */
  class PasswordApp : public Command
  {
  public:
    PasswordApp() = default;
    PasswordApp(const PasswordApp&) = default;
    PasswordApp(PasswordApp&&) = delete;
    PasswordApp& operator=(const PasswordApp&) = default;
    PasswordApp& operator=(PasswordApp&&) = delete;
    ~PasswordApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    std::string new_password;
    bool disable_password{false};
  }; // end: class PasswordApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_PASSWD_APP_H
