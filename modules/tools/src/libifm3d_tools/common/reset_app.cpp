/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/device/device.h"
#include "ifm3d/device/o3r.h"
#include "ifm3d/device/legacy_device.h"
#include "ifm3d/tools/ovp8xx/ovp8xx_app.h"
#include <ifm3d/tools/common/reset_app.h>
#include <memory>
#include <string>

ifm3d::ResetApp::~ResetApp() = default;

void
ifm3d::ResetApp::Execute(CLI::App* /*app*/)
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
      command->add_flag("-r,--reboot",
                        this->reboot,
                        "Reboot the sensor after reset. Default: False");
    }

  if (Parent<ifm3d::OVP8xx>())
    {
      command->add_flag("--keepNetworkSettings",
                        this->network_settings,
                        "Keep the current network settings");
    }

  return command;
}
