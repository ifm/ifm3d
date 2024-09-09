/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/swupdater/swupdate_app.h>
#include <exception>
#include <iostream>
#include <istream>
#include <fstream>
#include <memory>
#include <vector>
#include <chrono>
#include <filesystem>
#include <ifm3d/device.h>
#include <ifm3d/swupdater.h>

#ifdef _WIN32
#  include <io.h>
#  include <fcntl.h>
#endif

ifm3d::SWUpdateApp::~SWUpdateApp() {}

void
ifm3d::SWUpdateApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice(false);

  ifm3d::SWUpdater::Ptr swupdater = std::make_shared<ifm3d::SWUpdater>(
    device,
    [](float p, const std::string& msg) -> void {});

  if (detect)
    {
      if (swupdater->WaitForRecovery(-1))
        {
          std::cout << "Device is in recovery mode." << std::endl;
        }
      else if (swupdater->WaitForProductive(-1))
        {
          std::cout << "Device is in productive mode." << std::endl;
        }
      else
        {
          std::cout << "Unable to communicate with device." << std::endl;
        }
      return;
    }
}

CLI::App*
ifm3d::SWUpdateApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("swupdate", "Perform a firmware update on the camera. ")
      ->require_subcommand(0, 1);

// Hide command if SWUpdate module is not build
#if !defined(BUILD_MODULE_SWUPDATER)
  swupdate_cmd->group("");
#endif

  command
    ->add_flag("-d,--detect", this->detect, "Check the current mode of device")
    ->default_val(false)
    ->default_str("flag");

  RegisterSubcommand<ifm3d::FlashSWApp>(command);
  RegisterSubcommand<ifm3d::RestartApp>(command);

  return command;
}

bool
ifm3d::SWUpdateApp::CheckCompatibility()
{
  return true;
}
