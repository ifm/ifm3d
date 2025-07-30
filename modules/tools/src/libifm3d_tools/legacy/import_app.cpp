/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/tools/legacy/import_application_app.h"
#include "ifm3d/tools/legacy/import_device_app.h"
#include <ifm3d/tools/legacy/import_app.h>

ifm3d::ImportApp::~ImportApp() = default;

void
ifm3d::ImportApp::Execute(CLI::App* app)
{}

CLI::App*
ifm3d::ImportApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("import",
                       "Import an application or whole sensor configuration "
                       "that is compatible with "
                       "ifm Vision Assistant's export format (*.o3d3xxapp).")
      ->require_subcommand(1);

  RegisterSubcommand<ifm3d::ImportApplicationApp>(command);
  RegisterSubcommand<ifm3d::ImportDeviceApp>(command);

  return command;
}
