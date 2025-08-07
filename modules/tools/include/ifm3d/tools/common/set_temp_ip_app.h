// -*- c++ -*-
/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SET_TEMP_IP_APP_H
#define IFM3D_TOOLS_SET_TEMP_IP_APP_H
#pragma once

#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/tools/command.hpp>
#include <ifm3d/tools/main_command.hpp>
#include <string>

namespace ifm3d
{
  /**
   * Concrete implementation of the `discover set-temporary-ip` subcommand to
   * the `ifm3d` command-line utility.
   */
  class SetTemporaryIPApp : public Command
  {
  public:
    SetTemporaryIPApp() = default;
    SetTemporaryIPApp(const SetTemporaryIPApp&) = default;
    SetTemporaryIPApp(SetTemporaryIPApp&&) = delete;
    SetTemporaryIPApp& operator=(const SetTemporaryIPApp&) = default;
    SetTemporaryIPApp& operator=(SetTemporaryIPApp&&) = delete;
    ~SetTemporaryIPApp() override;
    void Execute(CLI::App* app) override;
    CLI::App* CreateCommand(CLI::App* parent) override;

    std::string mac;
    std::string temp_ip;

  }; // end: class SetTemporaryIPApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_SET_TEMP_IP_APP_H
