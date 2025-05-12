/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/device/device.h"
#include <CLI/Validators.hpp>
#include <ifm3d/tools/common/set_temp_ip_app.h>
#include <regex>

ifm3d::SetTemporaryIPApp::~SetTemporaryIPApp() = default;

void
ifm3d::SetTemporaryIPApp::Execute(CLI::App* /*app*/)
{
  ifm3d::Device::SetTempIPAddress(this->mac, this->temp_ip);
}

CLI::App*
ifm3d::SetTemporaryIPApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("set-temporary-ip", "Set temporary IP to device")
      ->require_subcommand(0, 0);

  command->add_option("--mac", this->mac, "MAC address of Device")
    ->required()
    ->check([](const std::string& mac) {
      const std::regex mac_regex("^([0-9A-Fa-f]{2}[:]){5}([0-9A-Fa-f]{2})$");
      if (std::regex_match(mac, mac_regex))
        {
          return std::string();
        }
      return std::string(
        "Invalid MAC Address, expected format [xx::xx::xx::xx::xx::xx]");
    });

  command->add_option("--ip", this->temp_ip, "Temporary IP address to be set")
    ->check(CLI::ValidIPV4)
    ->required();

  return command;
}
