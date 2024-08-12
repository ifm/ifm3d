/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/reset_app.h>
#include <iostream>
#include <memory>
#include <string>
#include <ifm3d/device.h>

ifm3d::ResetApp::~ResetApp() {}

void
ifm3d::ResetApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  if (device->AmI(ifm3d::Device::device_family::O3R))
    {
      std::static_pointer_cast<ifm3d::O3R>(device)->FactoryReset(
        network_settings);
    }
  else
    {
      std::static_pointer_cast<ifm3d::LegacyDevice>(device)->FactoryReset();
    }
  if (!Parent<ifm3d::OVP8xx>())
    {
      if (reboot > 0)
        {
          device->Reboot();
        }
    }
}

CLI::App*
ifm3d::ResetApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("reset", "Reset the sensor to factory defaults.")
      ->require_subcommand(0, 0);

  if (!(Parent<ifm3d::OVP8xx>()))
    {
      command
        ->add_flag("-r,--reboot",
                   this->reboot,
                   "Reboot the sensor after reset. Default: False")
        ->default_val(false)
        ->default_str("flag");
    }

  command
    ->add_option("--keepNetworkSettings",
                 this->network_settings,
                 "Keep the current network settings")
    ->default_val(true);

  return command;
}
