/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/device/o3r.h>
#include <ifm3d/tools/ovp8xx/jsonschema_app.h>
#include <iostream>
#include <memory>

ifm3d::JSONSchemaApp::~JSONSchemaApp() = default;

void
ifm3d::JSONSchemaApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();

  json const schema =
    std::static_pointer_cast<ifm3d::O3R>(device)->GetSchema();
  std::cout << schema.dump(2) << '\n';
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
