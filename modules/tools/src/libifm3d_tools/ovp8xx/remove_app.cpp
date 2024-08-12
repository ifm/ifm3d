/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/ovp8xx/remove_app.h>
#include <iostream>
#include <string>
#include <ifm3d/device.h>

ifm3d::RemoveApp::~RemoveApp() {}

void
ifm3d::RemoveApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();
  std::static_pointer_cast<ifm3d::O3R>(device)->Remove(this->path);
}

CLI::App*
ifm3d::RemoveApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("remove",
                       "Removes an object from the JSON, for example to "
                       "remove an application instance.")
      ->require_subcommand(0, 0);

  command->add_option("--path",
                      this->path,
                      "A JSON Pointer to the object to be removed");

  return command;
}
