/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <ifm3d/tools/common/config_set_app.h>
#include <ifm3d/tools/common/dump_app.h>
#include <ifm3d/tools/ovp8xx/config_ovp8xx_app.h>
#include <ifm3d/tools/ovp8xx/get_init_app.h>
#include <ifm3d/tools/ovp8xx/jsonschema_app.h>
#include <ifm3d/tools/ovp8xx/remove_app.h>
#include <ifm3d/tools/ovp8xx/reset_ovp8xx_app.h>
#include <ifm3d/tools/ovp8xx/save_init_app.h>
#include <string>

ifm3d::ConfigOvp8xxApp::~ConfigOvp8xxApp() = default;

void
ifm3d::ConfigOvp8xxApp::Execute(CLI::App* app)
{}

CLI::App*
ifm3d::ConfigOvp8xxApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand(
        "config",
        "Commands for reading and setting device configurations using JSON.")
      ->require_subcommand(1);

  RegisterSubcommand<ifm3d::ConfigSetApp>(command)->SetDetails(
    "set",
    "Overwrites parts of the temporary JSON configuration.");

  RegisterSubcommand<ifm3d::DumpApp>(command)->SetDetails(
    "get",
    "Gets the complete JSON config string or gets only a JSON string "
    "containing all queried subsets if provided JSON pointers exists");

  RegisterSubcommand<ifm3d::JSONSchemaApp>(command);
  RegisterSubcommand<ifm3d::GetInitApp>(command);
  RegisterSubcommand<ifm3d::SaveInitApp>(command);
  RegisterSubcommand<ifm3d::RemoveApp>(command);
  RegisterSubcommand<ifm3d::ResetOvp8xxApp>(command);

  return command;
}
