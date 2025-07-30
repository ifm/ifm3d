/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/tools/common/fg/hz_app.h"
#include "ifm3d/tools/common/fg/jitter_app.h"
#include <ifm3d/tools/common/fg/stat_app.h>
#include <string>

ifm3d::StatApp::~StatApp() = default;

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
