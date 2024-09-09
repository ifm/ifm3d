/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/ovp8xx/get_init_app.h>
#include <iostream>
#include <string>
#include <ifm3d/device.h>

ifm3d::GetInitApp::~GetInitApp() {}

void
ifm3d::GetInitApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();
  std::cout << std::static_pointer_cast<ifm3d::O3R>(device)->GetInit().dump(2);
}

CLI::App*
ifm3d::GetInitApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("getInit", "Get the initial JSON configuration")
      ->require_subcommand(0, 0);

  return command;
}
