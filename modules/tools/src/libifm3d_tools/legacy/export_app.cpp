/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/export_app.h>

ifm3d::ExportApp::~ExportApp() {}

void
ifm3d::ExportApp::Execute(CLI::App* app)
{}

CLI::App*
ifm3d::ExportApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("export",
                       "Export an application or whole "
                       "sensor configuration into a format compatible with "
                       "ifm Vision Assistant.")
      ->require_subcommand(1);

  RegisterSubcommand<ifm3d::ExportApplicationApp>(command);
  RegisterSubcommand<ifm3d::ExportDeviceApp>(command);

  return command;
}
