/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/dump_app.h>
#include <iostream>
#include <string>

ifm3d::DumpApp::~DumpApp() {}

void
ifm3d::DumpApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();
  if (device->AmI(Device::device_family::O3R))
    {
      if (this->paths.empty())
        {
          std::cout << device->ToJSONStr() << std::endl;
        }
      else
        {
          std::cout << std::static_pointer_cast<ifm3d::O3R>(device)
                         ->Get(this->paths)
                         .dump(2)
                    << std::endl;
        }
    }
  else
    {
      std::cout << device->ToJSONStr() << std::endl;
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
