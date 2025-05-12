/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/common/json_impl.hpp"
#include "ifm3d/device/legacy_device.h"
#include <ifm3d/tools/legacy/imager_types_app.h>
#include <iostream>
#include <memory>

ifm3d::ImagerApp::~ImagerApp() = default;

void
ifm3d::ImagerApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();

  json const im_types(
    std::static_pointer_cast<ifm3d::LegacyDevice>(device)->ImagerTypes());
  std::cout << im_types.dump(2) << '\n';
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
