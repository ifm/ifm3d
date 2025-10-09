/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <chrono>
#include <ifm3d/common/features.h>
#include <ifm3d/tools/common/swupdater/swupdate_deprecated_app.h>
#include <iostream>
#include <memory>
#include <string>

#if defined(BUILD_MODULE_SWUPDATER)
#  include <ifm3d/swupdater/swupdater.h>
#endif

#ifdef _WIN32
#  include <fcntl.h>
#  include <io.h>
#endif

ifm3d::SWUpdateDeprecatedApp::~SWUpdateDeprecatedApp() = default;

void
ifm3d::SWUpdateDeprecatedApp::Execute(CLI::App* /*app*/)
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

  ifm3d::SWUpdater::Ptr swupdater;
  if (quiet)
    {
      swupdater = std::make_shared<ifm3d::SWUpdater>(
        device,
        [](float p, const std::string& msg) -> void {},
        sw_port);
    }
  else
    {
      swupdater = std::make_shared<ifm3d::SWUpdater>(
        device,
        [](float p, const std::string& msg) {
          if (p < 1.0F)
            {
              int const width = 50;
              std::cout << "Uploading Firmware: [";
              int const pos = static_cast<int>(static_cast<float>(width) * p);
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
              std::cout << msg << '\n';
            }
        },
        sw_port);
    }

  if (check)
    {
      if (swupdater->WaitForRecovery(-1))
        {
          if (!quiet)
            {
              std::cout << "Device is in recovery mode." << '\n';
            }
        }
      else if (swupdater->WaitForProductive(-1))
        {
          if (!quiet)
            {
              std::cout << "Device is in productive mode." << '\n';
            }
        }
      else
        {
          if (!quiet)
            {
              std::cout << "Unable to communicate with device." << '\n';
            }
        }
      return;
    }
  if (reboot)
    {
      if (!quiet)
        {
          std::cout << "Rebooting device to productive mode..." << '\n';
        }
      swupdater->RebootToProductive();
      if (!swupdater->WaitForProductive(get_remaining_timeout()))
        {
          if (!quiet)
            {
              std::cout << "Timed out waiting for producitve mode" << '\n';
            }
          return;
        }
      return;
    }

  // Read the file in
  if (!swu_file.empty())
    {
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
ifm3d::SWUpdateDeprecatedApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand(
        "swupdate",
        "Perform a firmware update on the camera. "
        "Please ensure that the camera is booted to recovery beforehand.")
      ->require_subcommand(0, 0);

// Hide command if SWUpdate module is not build
#if !defined(BUILD_MODULE_SWUPDATER)
  swupdate_cmd->group("");
#endif

  command
    ->add_option(
      "--file",
      this->swu_file,
      "Input file, defaults to `stdin' (good for reading off a pipeline)")
    ->default_val("-");

  command->add_flag("-c,--check",
                    this->check,
                    "Check the current mode of device");

  command->add_flag("-r,--reboot",
                    this->reboot,
                    "Reboot from recovery mode to productive mode");

  command->add_flag("-q,--quiet",
                    this->quiet,
                    "Disable status output. Default: False");

  command->add_option("--swupdate-port", this->sw_port, "port for swupdate")
    ->default_val(8080);

  command
    ->add_option("--timeout",
                 this->timeout,
                 "time in seconds for the swupdate to complete")
    ->default_val(600);

  return command;
}
