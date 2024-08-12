/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/fg/stat_app.h>
#include <iostream>
#include <string>
#include <ifm3d/device/o3r.h>

ifm3d::StatApp::~StatApp() {}

void
ifm3d::StatApp::Execute(CLI::App* app)
{}

CLI::App*
ifm3d::StatApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("stat",
                       "Commands for interacting with the device and get "
                       "statistics using PCIC")
      ->require_subcommand(1);

  RegisterSubcommand<ifm3d::HzApp>(command);
  RegisterSubcommand<ifm3d::JitterApp>(command);

  return command;
}
