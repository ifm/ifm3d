/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/ovp8xx/save_init_app.h>
#include <iostream>
#include <string>
#include <ifm3d/device.h>

ifm3d::SaveInitApp::~SaveInitApp() {}

void
ifm3d::SaveInitApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();
  std::static_pointer_cast<ifm3d::O3R>(device)->SaveInit(this->paths);
}

CLI::App*
ifm3d::SaveInitApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand(
        "saveInit",
        "Save the current configuration as initial configuration")
      ->require_subcommand(0, 0);

  command
    ->add_option(
      "--path",
      this->paths,
      "Limit which part of the current configuration should be saved as "
      "initial JSON. Can be provided multiple times to save multiple paths.")
    ->expected(0, -1);

  return command;
}
