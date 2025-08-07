/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <ifm3d/tools/legacy/export_app.h>
#include <ifm3d/tools/legacy/export_application_app.h>
#include <ifm3d/tools/legacy/export_device_app.h>

ifm3d::ExportApp::~ExportApp() = default;

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
