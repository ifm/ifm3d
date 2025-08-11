/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/tools/common/dump_app.h>
#include <ifm3d/tools/ovp8xx/ovp8xx_app.h>
#include <iostream>
#include <memory>
#include <string>

ifm3d::DumpApp::~DumpApp() = default;

void
ifm3d::DumpApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();
  if (device->AmI(Device::DeviceFamily::O3R))
    {
      if (this->paths.empty())
        {
          std::cout << device->ToJSONStr() << '\n';
        }
      else
        {
          std::cout << std::static_pointer_cast<ifm3d::O3R>(device)
                         ->Get(this->paths)
                         .dump(2)
                    << '\n';
        }
    }
  else
    {
      std::cout << device->ToJSONStr() << '\n';
    }
}

CLI::App*
ifm3d::DumpApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("dump", "Serialize the sensor state to JSON.")
      ->require_subcommand(0, 0);

  if (Parent<ifm3d::OVP8xx>())
    {
      command
        ->add_option(
          "--path",
          this->paths,
          "Limit the retrieved json to a specific fragment corresponding to "
          "the given path. Can be provided multiple times to retrieve a "
          "fragment containing multiple paths.")
        ->expected(0, -1);
    }

  return command;
}
