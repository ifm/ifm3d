/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/device/legacy_device.h"
#include <ifm3d/tools/legacy/cp_app.h>
#include <memory>
#include <string>

ifm3d::CpApp::~CpApp() = default;

void
ifm3d::CpApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();

  std::static_pointer_cast<ifm3d::LegacyDevice>(device)->CopyApplication(
    this->index);
}

CLI::App*
ifm3d::CpApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("cp",
                       "Create a new application on the sensor, "
                       "bootstrapped from a copy of an existing one.")
      ->require_subcommand(0, 0);
  command
    ->add_option("--index", this->index, "Index of source application to copy")
    ->default_val(-1)
    ->required();

  return command;
}
