/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <array>
#include <chrono>
#include <fstream>
#include <ifm3d/tools/common/swupdater/flash_sw_app.h>
#include <ifm3d/tools/common/swupdater/swupdate_app.h>
#include <iostream>
#include <string>

#ifdef _WIN32
#  include <fcntl.h>
#  include <io.h>
#endif

const std::string ASCII_FORMAT_MAGIC_NUMBER = "070701";
const std::string CRC_FORMAT_MAGIC_NUMBER = "070702";

ifm3d::FlashSWApp::~FlashSWApp() = default;

void
ifm3d::FlashSWApp::Execute(CLI::App* /*app*/)
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

  auto validate_swu_file_header = [](std::string& swu_file) -> bool {
    std::ifstream file(swu_file, std::ios::binary);

    if (file.good())
      {
        std::array<char, 6> magic{};
        file.read(magic.data(), magic.size());
        std::string const magic_str(magic.data(), magic.size());

        return (magic_str == ASCII_FORMAT_MAGIC_NUMBER ||
                magic_str == CRC_FORMAT_MAGIC_NUMBER);
      }

    return false;
  };

  // Read the file in
  if (!swu_file.empty())
    {
      // Check if provided swu_file exists
      const std::fstream infile{swu_file};

      if ((swu_file != "-") && !validate_swu_file_header(swu_file))
        {
          std::cerr << "ifm3d error: File not found or invalid file.\n";
          return;
        }

      // Reboot to recovery if not already in recovery
      if (!swupdater->WaitForRecovery(-1))
        {
          if (!quiet)
            {
              std::cout << "Rebooting device to recovery mode..." << '\n';
            }
          swupdater->RebootToRecovery();
          if (!swupdater->WaitForRecovery(get_remaining_timeout()))
            {
              if (!quiet)
                {
                  std::cout << "Timed out waiting for recovery mode" << '\n';
                }
              return;
            }
        }

      if (!swupdater->FlashFirmware(swu_file, get_remaining_timeout()))
        {
          if (!quiet)
            {
              std::cout << "Timed out waiting for flashing to complete"
                        << '\n';
            }
          return;
        }

      swupdater->RebootToProductive();
      if (!quiet)
        {
          std::cout << "Update successful, waiting for device to reboot..."
                    << '\n';
        }
      if (!swupdater->WaitForProductive(get_remaining_timeout()))
        {
          if (!quiet)
            {
              std::cout << "Timed out waiting for productive mode" << '\n';
            }
          return;
        }
      if (!quiet)
        {
          std::cout << "SWUpdate Complete." << '\n';
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
