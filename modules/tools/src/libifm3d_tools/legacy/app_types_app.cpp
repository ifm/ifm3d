/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/app_types_app.h>
//#include <ifm3d/tools/HelpFormatter.hpp>
#include <iostream>

ifm3d::AppTypesApp::~AppTypesApp() {}

void
ifm3d::AppTypesApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  json app_types(
    std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ApplicationTypes());
  std::cout << app_types.dump(2) << std::endl;
}

CLI::App*
ifm3d::AppTypesApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("app-types",
                       "List the application types supported by the sensor.")
      ->require_subcommand(0, 0);
  // command->set_help_flag("");
  // command->add_flag("-h,--help", "Print this help message and exit");

  // use HelpFormatter
  // auto fmt = std::make_shared<CLI::HelpFormatter>();
  // command->formatter(fmt);

  return command;
}
