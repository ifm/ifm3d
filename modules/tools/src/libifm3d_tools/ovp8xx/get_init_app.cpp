/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/device/o3r.h"
#include <ifm3d/tools/ovp8xx/get_init_app.h>
#include <iostream>
#include <memory>
#include <string>

ifm3d::GetInitApp::~GetInitApp() = default;

void
ifm3d::GetInitApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();
  std::cout << std::static_pointer_cast<ifm3d::O3R>(device)->GetInit().dump(2);
}

CLI::App*
ifm3d::GetInitApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("getInit", "Get the initial JSON configuration")
      ->require_subcommand(0, 0);

  return command;
}
