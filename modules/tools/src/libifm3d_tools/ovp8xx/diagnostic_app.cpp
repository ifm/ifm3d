/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/ovp8xx/diagnostic_app.h>
#include <iostream>
#include <string>
#include <ifm3d/device/o3r.h>

ifm3d::DiagnosticApp::~DiagnosticApp() {}

void
ifm3d::DiagnosticApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  if (this->filterSchema)
    {
      std::cout << std::static_pointer_cast<ifm3d::O3R>(device)
                     ->GetDiagnosticFilterSchema()
                     .dump(2)
                << std::endl;
    }
  else if (!this->filter_expression.empty())
    {
      std::cout << std::static_pointer_cast<ifm3d::O3R>(device)
                     ->GetDiagnosticFiltered(
                       json::parse(this->filter_expression))
                     .dump(2)
                << std::endl;
    }
  else
    {
      std::cout
        << std::static_pointer_cast<ifm3d::O3R>(device)->GetDiagnostic().dump(
             2)
        << std::endl;
    }
}

CLI::App*
ifm3d::DiagnosticApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("diagnostic",
                       "Access the device diagnostic information")
      ->require_subcommand(0);

  /*RegisterSubcommand<ifm3d::GetDiagnosticApp>(command);
  RegisterSubcommand<ifm3d::GetFilteredApp>(command);
  RegisterSubcommand<ifm3d::GetFilterSchemaApp>(command);*/

  command
    ->add_option("-f,--filter",
                 this->filter_expression,
                 "A filter expression in JSON format")
    ->default_str("{}");

  command->add_option("--filterSchema",
                      this->filterSchema,
                      "Get the JSON schema for filter expressions");

  return command;
}
