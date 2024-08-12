/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/reboot_app.h>
#include <iostream>
#include <ifm3d/device/device.h>
#include <ifm3d/swupdater.h>

ifm3d::RebootApp::~RebootApp() {}

void
ifm3d::RebootApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice(false);

  ifm3d::Device::boot_mode mode = recovery ?
                                    ifm3d::Device::boot_mode::RECOVERY :
                                    ifm3d::Device::boot_mode::PRODUCTIVE;

  ifm3d::SWUpdater::Ptr swupdater;
  swupdater = std::make_shared<ifm3d::SWUpdater>(device);

  if (swupdater->WaitForRecovery(-1))
    {
      swupdater->RebootToProductive();
    }
  else
    {
      device->Reboot(mode);
    }
}

CLI::App*
ifm3d::RebootApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand(
        "reboot",
        "Reboot the sensor, potentially into recovery mode. \nRecovery mode "
        "is useful for putting the "
        "sensor into a state where it can be flashed with new firmware.")
      ->require_subcommand(0, 0);

  command
    ->add_option("-r,--recovery", this->recovery, "Reboot into recovery mode")
    ->default_val(false);

  return command;
}

bool
ifm3d::RebootApp::CheckCompatibility()
{
  return true;
}
