/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/ovp8xx/get_diagnostic_app.h>
#include <iostream>
#include <string>
#include <ifm3d/device/o3r.h>

ifm3d::GetDiagnosticApp::~GetDiagnosticApp() {}

void
ifm3d::GetDiagnosticApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  if (this->filter_expression.empty())
    {
      std::cout
        << std::static_pointer_cast<ifm3d::O3R>(device)->GetDiagnostic().dump(
             2)
        << std::endl;
    }
  else
    {
      std::cout << std::static_pointer_cast<ifm3d::O3R>(device)
                     ->GetDiagnosticFiltered(
                       json::parse(this->filter_expression))
                     .dump(2)
                << std::endl;
    }
}

CLI::App*
ifm3d::GetDiagnosticApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand(
        "get",
        "Get the content of the diagnostic memory formatted in JSON and "
        "filtered according to the JSON filter expression")
      ->require_subcommand(0, 0);

  command
    ->add_option("-f,--filter",
                 this->filter_expression,
                 "A filter expression in JSON format")
    ->default_str("{}");

  return command;
}
