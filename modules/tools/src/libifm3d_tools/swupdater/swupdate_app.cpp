/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/swupdater/swupdate_app.h>
#include <exception>
#include <iostream>
#include <istream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device.h>
#include <ifm3d/swupdater.h>

#ifdef _WIN32
#  include <io.h>
#  include <fcntl.h>
#endif

ifm3d::SWUpdateApp::SWUpdateApp(int argc,
                                const char** argv,
                                const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name, false)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("file","Input file, defaults to `stdin' (good for reading off a pipeline)",
     cxxopts::value<std::string>()->default_value("-"))
    ("c,check","Check the current mode of device",
     cxxopts::value<bool>()->default_value("false"))
    ("r,reboot","Reboot from recovery mode to productive mode",
     cxxopts::value<bool>()->default_value("false"))
    ("q,quiet","Disable status output",
     cxxopts::value<bool>()->default_value("false"))
    ("swupdate-port","port for swupdate",
     cxxopts::value<unsigned short>()->default_value("8080"))
   ("timeout", "time in seconds for the swupdate to complete",
     cxxopts::value<unsigned int>()->default_value("600"));

  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::SWUpdateApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto const check = (*this->vm_)["check"].as<bool>();
  auto const recovery_reboot = (*this->vm_)["reboot"].as<bool>();
  auto const quiet = (*this->vm_)["quiet"].as<bool>();
  auto const swupdate_port =
    (*this->vm_)["swupdate-port"].as<unsigned short>();
  auto const timeout_sec = (*this->vm_)["timeout"].as<unsigned int>();
  auto const timeout_millisec =
    std::chrono::milliseconds(std::chrono::seconds(timeout_sec));

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
        this->cam_,
        [](float p, const std::string& msg) -> void {},
        swupdate_port);
    }
  else
    {
      swupdater = std::make_shared<ifm3d::SWUpdater>(
        this->cam_,
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
        },
        swupdate_port);
    }

  if (check)
    {
      if (swupdater->WaitForRecovery(-1))
        {
          if (!quiet)
            {
              std::cout << "Device is in recovery mode." << std::endl;
            }
        }
      else if (swupdater->WaitForProductive(-1))
        {
          if (!quiet)
            {
              std::cout << "Device is in productive mode." << std::endl;
            }
        }
      else
        {
          if (!quiet)
            {
              std::cout << "Unable to communicate with device." << std::endl;
            }
        }
      return 0;
    }
  else if (recovery_reboot)
    {
      if (!quiet)
        {
          std::cout << "Rebooting device to productive mode..." << std::endl;
        }
      swupdater->RebootToProductive();
      if (!swupdater->WaitForProductive(get_remaining_timeout()))
        {
          if (!quiet)
            {
              std::cout << "Timed out waiting for producitve mode"
                        << std::endl;
            }
          return -1;
        }
      return 0;
    }
  else
    {
      // Read the file in
      std::string infile = (*this->vm_)["file"].as<std::string>();

      if (!infile.empty())
        {
          // Reboot to recovery if not already in recovery
          if (!swupdater->WaitForRecovery(-1))
            {
              if (!quiet)
                {
                  std::cout << "Rebooting device to recovery mode..."
                            << std::endl;
                }
              swupdater->RebootToRecovery();
              if (!swupdater->WaitForRecovery(get_remaining_timeout()))
                {
                  if (!quiet)
                    {
                      std::cout << "Timed out waiting for recovery mode"
                                << std::endl;
                    }
                  return -1;
                }
            }

          if (!swupdater->FlashFirmware(infile, get_remaining_timeout()))
            {
              if (!quiet)
                {
                  std::cout << "Timed out waiting for flashing to complete"
                            << std::endl;
                }
              return -1;
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
              return -1;
            }
          if (!quiet)
            {
              std::cout << "SWUpdate Complete." << std::endl;
            }
        }
    }
  return 0;
}
