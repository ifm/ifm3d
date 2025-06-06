/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/common/features.h>
#include <ifm3d/tools/common/swupdater/flash_sw_app.h>
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

const std::string ASCII_FORMAT_MAGIC_NUMBER = "070701";
const std::string CRC_FORMAT_MAGIC_NUMBER = "070702";

ifm3d::FlashSWApp::~FlashSWApp() {}

void
ifm3d::FlashSWApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice(false);

  auto const timeout_millisec =
    std::chrono::milliseconds(std::chrono::seconds(timeout));

  const std::chrono::time_point<std::chrono::system_clock> start_time =
    std::chrono::system_clock::now();

  auto get_remaining_timeout = [&start_time, &timeout_millisec]() -> long {
    const std::chrono::time_point<std::chrono::system_clock> now =
      std::chrono::system_clock::now();
    auto utilized_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);

    return (timeout_millisec - utilized_time).count();
  };

  auto swupdater = Parent<SWUpdateApp>()->CreateSWUpdater(quiet);

  auto validateSwuFileHeader = [](std::string& swu_file) -> bool {
    std::ifstream file(swu_file, std::ios::binary);

    if (file.good())
      {
        char magic[6];
        file.read(magic, 6);
        std::string magicStr(magic, 6);

        return (magicStr == ASCII_FORMAT_MAGIC_NUMBER ||
                magicStr == CRC_FORMAT_MAGIC_NUMBER);
      }
    else
      {
        return false;
      }
  };

  // Read the file in
  if (!swu_file.empty())
    {
      // Check if provided swu_file exists
      const std::fstream infile{swu_file};

      if ((swu_file != "-") && !validateSwuFileHeader(swu_file))
        {
          std::cerr << "ifm3d error: File not found or invalid file.\n";
          return;
        }

      // Reboot to recovery if not already in recovery
      if (!swupdater->WaitForRecovery(-1))
        {
          if (!quiet)
            {
              std::cout << "Rebooting device to recovery mode..." << std::endl;
            }
          swupdater->RebootToRecovery();
          if (!swupdater->WaitForRecovery(get_remaining_timeout()))
            {
              if (!quiet)
                {
                  std::cout << "Timed out waiting for recovery mode"
                            << std::endl;
                }
              return;
            }
        }

      if (!swupdater->FlashFirmware(swu_file, get_remaining_timeout()))
        {
          if (!quiet)
            {
              std::cout << "Timed out waiting for flashing to complete"
                        << std::endl;
            }
          return;
        }

      swupdater->RebootToProductive();
      if (!quiet)
        {
          std::cout << "Update successful, waiting for device to reboot..."
                    << std::endl;
        }
      if (!swupdater->WaitForProductive(get_remaining_timeout()))
        {
          if (!quiet)
            {
              std::cout << "Timed out waiting for productive mode"
                        << std::endl;
            }
          return;
        }
      if (!quiet)
        {
          std::cout << "SWUpdate Complete." << std::endl;
        }
    }
}

CLI::App*
ifm3d::FlashSWApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("flash",
                       "Perform a firmware update on the camera. "
                       "The command will reboot the device in recovery before "
                       "performing software update.")
      ->require_subcommand(0, 0);

// Hide command if SWUpdate module is not build
#if !defined(BUILD_MODULE_SWUPDATER)
  swupdate_cmd->group("");
#endif

  command->add_option(
    "file",
    this->swu_file,
    "Input file, use `-` to read from stdin (good for reading "
    "off a pipeline)");

  command->add_flag("-q,--quiet",
                    this->quiet,
                    "Disable status output. Default: False");

  command
    ->add_option("--timeout",
                 this->timeout,
                 "The time in seconds for the swupdate to complete")
    ->default_val(300);

  return command;
}
