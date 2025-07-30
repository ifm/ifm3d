/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <CLI/Error.hpp>
#include "ifm3d/device/legacy_device.h"
#include <ifm3d/tools/legacy/passwd_app.h>
#include <memory>
#include <string>

ifm3d::PasswordApp::~PasswordApp() = default;

void
ifm3d::PasswordApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();

  auto const new_password_status = !new_password.empty();
  std::string password;

  if (new_password_status && disable_password)
    {
      throw CLI::ArgumentMismatch("Error, Invalid options combination.",
                                  CLI::ExitCodes::ArgumentMismatch);
    }
  if (new_password_status)
    {
      password = new_password;
      std::static_pointer_cast<ifm3d::LegacyDevice>(device)->SetPassword(
        password);
    }
  else if (disable_password)
    {
      std::static_pointer_cast<ifm3d::LegacyDevice>(device)->SetPassword(
        password);
    }
  else
    {
      throw CLI::CallForHelp();
    }
}

CLI::App*
ifm3d::PasswordApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("passwd", "Sets the password on the sensor.")
      ->require_subcommand(0, 0)
      ->require_option();

  command->add_option("--new",
                      this->new_password,
                      "password to be set on sensor");

  command->add_flag("--disable",
                    this->disable_password,
                    "disable password on sensor");

  return command;
}
