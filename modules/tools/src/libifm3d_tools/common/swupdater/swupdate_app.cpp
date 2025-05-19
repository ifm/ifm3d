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

#ifdef _WIN32
#  include <io.h>
#  include <fcntl.h>
#endif

ifm3d::SWUpdateApp::SWUpdateApp(
  std::optional<ifm3d::Device::swu_version> force_swu_version)
  : force_swu_version(force_swu_version)
{}
ifm3d::SWUpdateApp::~SWUpdateApp() {}

void
ifm3d::SWUpdateApp::Execute(CLI::App* app)
{
  if (!this->subcmd_flash->parsed() && !this->subcmd_restart->parsed())
    {
      auto swupdater = Parent<SWUpdateApp>()->CreateSWUpdater();

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
      else
        {
          throw CLI::CallForHelp();
        }
    }
}

std::shared_ptr<ifm3d::SWUpdater>
ifm3d::SWUpdateApp::CreateSWUpdater(bool quiet,
                                    const std::uint16_t swupdate_recovery_port)
{
  auto device = Parent<MainCommand>()->GetDevice(false);

  return std::make_shared<ifm3d::SWUpdater>(
    device,
    quiet ? static_cast<SWUpdater::FlashStatusCb>(
              [](float p, const std::string& msg) {}) :
            static_cast<SWUpdater::FlashStatusCb>(
              [](float p, const std::string& msg) {
                if (p < 1.0f)
                  {
                    int width = 50;
                    std::cout << "Uploading Firmware: [";
                    int pos = int(width * p);
                    for (int i = 0; i < width; ++i)
                      {
                        if (i < pos)
                          {
                            std::cout << "=";
                          }
                        else if (i == pos)
                          {
                            std::cout << ">";
                          }
                        else
                          {
                            std::cout << " ";
                          }
                      }
                    std::cout << "] " << int(p * 100) << "%\r";
                    std::cout.flush();
                  }
                else
                  {
                    std::cout << msg << std::endl;
                  }
              }),
    swupdate_recovery_port,
    this->force_swu_version);
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

  command->add_flag("-d,--detect",
                    this->detect,
                    "Check the current mode of device");

  subcmd_flash =
    RegisterSubcommand<ifm3d::FlashSWApp>(command)->GetSubcommandApp();
  subcmd_restart =
    RegisterSubcommand<ifm3d::RestartApp>(command)->GetSubcommandApp();

  return command;
}

bool
ifm3d::SWUpdateApp::CheckCompatibility()
{
  return true;
}
