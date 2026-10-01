/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <fmt/format.h> // NOLINT(*)
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/tools/common/swupdater/restart_app.h>
#include <ifm3d/tools/common/swupdater/swupdate_app.h>
#include <ifm3d/tools/o3cxxx/o3cxxx_app.h>
#include <ifm3d/tools/ovp8xx/ovp8xx_app.h>
#include <iostream>
#include <memory>
#include <optional>

#ifdef _WIN32
#  include <fcntl.h>
#  include <io.h>
#endif

ifm3d::RestartApp::~RestartApp() = default;

void
ifm3d::RestartApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice(false);
  auto swupdater = Parent<SWUpdateApp>()->CreateSWUpdater();
  ifm3d::reboot_device(device,
                       swupdater,
                       ifm3d::Device::BootMode::RECOVERY,
                       this->_wait,
                       _password);
}

CLI::App*
ifm3d::RestartApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command = parent->add_subcommand("restart", "Restart the device.")
                        ->require_subcommand(0, 0);

  command->add_flag(
    "-w,--wait",
    this->_wait,
    "Wait for the device to come back online after restarting.");

#ifdef BUILD_MODULE_CRYPTO
  if (Parent<ifm3d::OVP8xx>() || Parent<ifm3d::O3Cxxx>())
    {
      command->add_option("--password",
                          this->_password,
                          "Password for the device if required");
    }
#endif

  return command;
}

bool
ifm3d::RestartApp::CheckCompatibility()
{
  return true;
}

void
ifm3d::reboot_device(ifm3d::Device::Ptr device,
                     ifm3d::SWUpdater::Ptr swupdater,
                     ifm3d::Device::BootMode mode,
                     bool wait,
                     const std::optional<std::string>& password)
{
  const auto msg = fmt::format(
    "Rebooting device into {} mode\n",
    mode == ifm3d::Device::BootMode::RECOVERY ? "recovery" : "productive");

  if (swupdater->WaitForRecovery(-1))
    {
      std::cout << "Device is in recovery mode. Rebooting to productive mode."
                << '\n';
      swupdater->RebootToProductive();
      mode = ifm3d::Device::BootMode::PRODUCTIVE;
    }
  else
    {
      std::cout << msg;
#ifdef BUILD_MODULE_CRYPTO
      if (auto o3r = std::dynamic_pointer_cast<O3R>(device);
          o3r && o3r->SealedBox()->IsPasswordProtected() &&
          mode == ifm3d::Device::BootMode::RECOVERY)
        {
          o3r->SealedBox()->RebootToRecovery(password.value_or(""));
        }
      else
        {
#endif
          device->Reboot(mode);
#ifdef BUILD_MODULE_CRYPTO
        }
#endif
    }

  if (wait)
    {
      std::cout << "Waiting for device to come back online..." << '\n';

      if (mode == ifm3d::Device::BootMode::PRODUCTIVE ?
            swupdater->WaitForProductive() :
            swupdater->WaitForRecovery())
        {
          std::cout << "Device is available." << '\n';
        }
    }
}