/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/rm_app.h>
#include <iostream>
#include <ifm3d/device.h>

ifm3d::RmApp::~RmApp() {}

void
ifm3d::RmApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();
  std::static_pointer_cast<ifm3d::LegacyDevice>(device)->DeleteApplication(
    index);
}

CLI::App*
ifm3d::RmApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("rm", "Deletes an application from the sensor.")
      ->require_subcommand(0, 0);

  command->add_option("--index", index, "Index of application to remove")
    ->default_val(-1);

  return command;
}
