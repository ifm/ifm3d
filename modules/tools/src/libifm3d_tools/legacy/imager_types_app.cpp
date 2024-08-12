/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/imager_types_app.h>
#include <iostream>
#include <ifm3d/device.h>

ifm3d::ImagerApp::~ImagerApp() {}

void
ifm3d::ImagerApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  json im_types(
    std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ImagerTypes());
  std::cout << im_types.dump(2) << std::endl;
}

CLI::App*
ifm3d::ImagerApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("imager-types",
                       "List the imager types supported by the sensor.")
      ->require_subcommand(0, 0);

  return command;
}
