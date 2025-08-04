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

  if (Parent<ifm3d::OVP8xx>())
    {
      if (reset_network_opt != nullptr && reset_network_opt->count() >= 1)
        {
          reset_network_settings = this->reset_network_settings;
        }

      if (keep_network_opt != nullptr && keep_network_opt->count() >= 1)
        {
          reset_network_settings = !this->network_settings;
        }
    }

  if (device->AmI(ifm3d::Device::device_family::O3R))
    {
      std::static_pointer_cast<ifm3d::O3R>(device)->FactoryReset(
        !reset_network_settings);
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
      command->add_flag("-r,--reboot",
                        this->reboot,
                        "Reboot the sensor after reset. Default: False");
    }

  if (Parent<ifm3d::OVP8xx>())
    {
      keep_network_opt =
        command->add_flag("--keepNetworkSettings", this->network_settings)
          ->description("Keep the current network settings")
          ->group("");

      reset_network_opt =
        command
          ->add_flag("--resetNetworkSettings", this->reset_network_settings)
          ->default_val(false)
          ->default_str("")
          ->description("Reset network settings to factory default (Mutually "
                        "exclusive with --keepNetworkSettings)");

      keep_network_opt->excludes(reset_network_opt);
      reset_network_opt->excludes(keep_network_opt);
    }

  return command;
}
