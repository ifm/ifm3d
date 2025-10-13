/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/tools/common/reboot_app.h>
#include <ifm3d/tools/ovp8xx/ovp8xx_app.h>
#if defined(BUILD_MODULE_SWUPDATER)
#  include <ifm3d/swupdater/swupdater.h>
#  include <ifm3d/tools/common/swupdater/restart_app.h>
#endif
#include <memory>
#include <string>

ifm3d::RebootApp::~RebootApp() = default;

void
ifm3d::RebootApp::Execute(CLI::App* /*app*/)
{
  const auto device = Parent<MainCommand>()->GetDevice(false);
  auto mode = _recovery ? ifm3d::Device::BootMode::RECOVERY :
                          ifm3d::Device::BootMode::PRODUCTIVE;

#if defined(BUILD_MODULE_SWUPDATER)
  const auto swu_version =
    Parent<OVP8xx>() ? Device::SWUVersion::SWU_V2 : Device::SWUVersion::SWU_V1;

  const auto swupdater = std::make_shared<ifm3d::SWUpdater>(
    device,
    [](float p, const std::string& msg) {},
    SWUPDATER_RECOVERY_PORT,
    swu_version);

  ifm3d::reboot_device(device, swupdater, mode, this->_wait);
#else
  std::cout << msg;
  device->Reboot(mode);
#endif
}

CLI::App*
ifm3d::RebootApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command = parent->add_subcommand("reboot", "Reboot the sensor")
                        ->require_subcommand(0, 0);

#if defined(BUILD_MODULE_SWUPDATER)
  command->add_flag(
    "-r,--recovery",
    this->_recovery,
    "Reboot into recovery mode.\nRecovery mode is useful for putting the "
    "sensor into a state where it can be flashed with new firmware.");

  command->add_flag(
    "-w,--wait",
    this->_wait,
    "Wait for the device to come back online after restarting.");
#endif

  return command;
}

bool
ifm3d::RebootApp::CheckCompatibility()
{
  return true;
}