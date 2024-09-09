/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/ovp8xx/jsonschema_app.h>
#include <iostream>
#include <ifm3d/device/o3r.h>

ifm3d::JSONSchemaApp::~JSONSchemaApp() {}

void
ifm3d::JSONSchemaApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  json schema = std::static_pointer_cast<ifm3d::O3R>(device)->GetSchema();
  std::cout << schema.dump(2) << std::endl;
}

CLI::App*
ifm3d::JSONSchemaApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("getSchema",
                       "Gets the JSON schema for the device configuration")
      ->require_subcommand(0, 0);

  return command;
}
