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
