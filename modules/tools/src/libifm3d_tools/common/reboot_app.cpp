/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/reboot_app.h>
#include <iostream>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/common/features.h>
#if defined(BUILD_MODULE_SWUPDATER)
#  include <ifm3d/swupdater.h>
#endif

ifm3d::RebootApp::~RebootApp() {}

void
ifm3d::RebootApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice(false);

  ifm3d::Device::boot_mode mode = recovery ?
                                    ifm3d::Device::boot_mode::RECOVERY :
                                    ifm3d::Device::boot_mode::PRODUCTIVE;

#if defined(BUILD_MODULE_CRYPTO)
  if (auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(device);
      o3r && mode == ifm3d::Device::boot_mode::RECOVERY &&
      o3r->SealedBox()->IsPasswordProtected())
    {
      o3r->SealedBox()->RebootToRecovery(password.value_or(""));
      return;
    }
#endif

#if defined(BUILD_MODULE_SWUPDATER)
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
#else
  device->Reboot();
#endif
}

CLI::App*
ifm3d::RebootApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command = parent->add_subcommand("reboot", "Reboot the sensor")
                        ->require_subcommand(0, 0);

  command->add_flag(
    "-r,--recovery",
    this->recovery,
    "Reboot into recovery mode.\nRecovery mode is useful for putting the "
    "sensor into a state where it can be flashed with new firmware.");

#if defined(BUILD_MODULE_CRYPTO)
  if (Parent<ifm3d::OVP8xx>())
    {
      command->add_option("--password",
                          this->password,
                          "Password for the device if required");
    }
#endif

  return command;
}

bool
ifm3d::RebootApp::CheckCompatibility()
{
  return true;
}
