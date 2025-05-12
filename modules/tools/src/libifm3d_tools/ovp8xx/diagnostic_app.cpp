/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/tools/ovp8xx/get_diagnostic_app.h"
#include "ifm3d/tools/ovp8xx/get_filter_schema_app.h"
#include <ifm3d/tools/ovp8xx/diagnostic_app.h>
#include <string>

ifm3d::DiagnosticApp::~DiagnosticApp() = default;

void
ifm3d::DiagnosticApp::Execute(CLI::App* app)
{}

CLI::App*
ifm3d::DiagnosticApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("diagnostic",
                       "Access the device diagnostic information. By default, "
                       "only the active diagnostic is shown.")
      ->require_subcommand(1);

  RegisterSubcommand<ifm3d::GetDiagnosticApp>(command);
  RegisterSubcommand<ifm3d::GetFilterSchemaApp>(command);

  return command;
}
