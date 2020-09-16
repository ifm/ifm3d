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
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera.h>
#include <ifm3d/swupdater.h>

#ifdef _WIN32
#  include <io.h>
#  include <fcntl.h>
#endif

ifm3d::SWUpdateApp::SWUpdateApp(int argc,
                                const char** argv,
                                const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
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
     cxxopts::value<unsigned short>()->default_value("8080"));

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
      if (!swupdater->WaitForProductive(60000))
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
      std::shared_ptr<std::istream> ifs;
      std::vector<std::uint8_t> bytes;

      std::string infile = (*this->vm_)["file"].as<std::string>();
      if (infile == "-")
        {
#ifdef _WIN32
          // on windows we need to reopen stdin in binary mode
          _setmode(_fileno(stdin), O_BINARY);
#endif

          ifs.reset(&std::cin, [](...) {});

          char b;
          while (ifs->get(b))
            {
              bytes.push_back(*(reinterpret_cast<std::uint8_t*>(&b)));
            }
        }
      else
        {
          ifs.reset(
            new std::ifstream(infile, std::ios::in | std::ios::binary));
          if (!*ifs)
            {
              std::cerr << "Could not open file: " << infile << std::endl;
              throw ifm3d::error_t(IFM3D_IO_ERROR);
            }

          ifs->unsetf(std::ios::skipws);
          std::streampos file_size;
          ifs->seekg(0, std::ios::end);
          file_size = ifs->tellg();
          ifs->seekg(0, std::ios::beg);

          bytes.reserve(file_size);
          bytes.insert(bytes.begin(),
                       std::istream_iterator<std::uint8_t>(*ifs),
                       std::istream_iterator<std::uint8_t>());
        }
      if (!bytes.empty())
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
              if (!swupdater->WaitForRecovery(60000))
                {
                  if (!quiet)
                    {
                      std::cout << "Timed out waiting for recovery mode"
                                << std::endl;
                    }
                  return -1;
                }
            }

          if (!swupdater->FlashFirmware(bytes, 300000))
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
          if (!swupdater->WaitForProductive(60000))
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
