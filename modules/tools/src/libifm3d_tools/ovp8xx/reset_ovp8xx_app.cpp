/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/device/o3r.h"
#include <ifm3d/tools/ovp8xx/reset_ovp8xx_app.h>
#include <memory>
#include <string>

ifm3d::ResetOvp8xxApp::~ResetOvp8xxApp() = default;

void
ifm3d::ResetOvp8xxApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();
  std::static_pointer_cast<ifm3d::O3R>(device)->Reset(this->path);
}

CLI::App*
ifm3d::ResetOvp8xxApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand(
        "reset",
        "Reset a parameter to its default value.\nThe parameter "
        "is addressed by a JSON pointer. If no pointer is "
        "provided, the complete JSON is reset.")
      ->require_subcommand(0, 0);

  command->add_option("--path",
                      this->path,
                      "A JSON Pointer to the object to be set to default");

  return command;
}
