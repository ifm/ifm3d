/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/ovp8xx/get_filter_schema_app.h>
#include <iostream>
#include <string>
#include <ifm3d/device/o3r.h>

ifm3d::GetFilterSchemaApp::~GetFilterSchemaApp() {}

void
ifm3d::GetFilterSchemaApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  std::cout << std::static_pointer_cast<ifm3d::O3R>(device)
                 ->GetDiagnosticFilterSchema()
                 .dump(2)
            << std::endl;
}

CLI::App*
ifm3d::GetFilterSchemaApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("getFilterSchema",
                       "Get the JSON schema for filter expressions")
      ->require_subcommand(0, 0);

  return command;
}
