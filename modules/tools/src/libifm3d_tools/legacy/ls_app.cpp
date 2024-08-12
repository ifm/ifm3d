/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/ls_app.h>
#include <iostream>

ifm3d::LsApp::~LsApp() {}

void
ifm3d::LsApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  json apps =
    std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ApplicationList();
  std::cout << apps.dump(2) << std::endl;
}

CLI::App*
ifm3d::LsApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand(
        "ls",
        "Lists the applications currently installed on the sensor.")
      ->require_subcommand(0, 0);

  return command;
}
