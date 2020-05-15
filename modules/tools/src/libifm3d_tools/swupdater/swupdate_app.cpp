/*
 * Copyright (C) 2019 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ifm3d/tools/swupdater/swupdate_app.h>
#include <exception>
#include <iostream>
#include <istream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera.h>
#include <ifm3d/swupdater.h>

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

ifm3d::SWUpdateApp::SWUpdateApp(int argc, const char **argv,
                                const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("file", po::value<std::string>()->default_value("-"),
     "Input file, defaults to `stdin' (good for reading off a pipeline)")
    ("check,c", po::bool_switch()->default_value(false),
     "Check the current mode of device")
    ("reboot,r", po::bool_switch()->default_value(false),
     "Reboot from recovery mode to productive mode")
    ("quiet,q", po::bool_switch()->default_value(false),
     "Disable status output");

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

int ifm3d::SWUpdateApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto const file = vm_.count("file") ? true : false;
  auto const check = vm_.count("check") ? vm_["check"].as<bool>() : false;
  auto const recovery_reboot = vm_.count("reboot") ?
    vm_["reboot"].as<bool>() : false;
  auto const quiet = vm_.count("quiet") ? vm_["quiet"].as<bool>() : false;

  ifm3d::SWUpdater::Ptr swupdater;
  if (quiet)
    {
      swupdater = std::make_shared<ifm3d::SWUpdater>(this->cam_);
    }
  else
    {
      swupdater = std::make_shared<ifm3d::SWUpdater>(
          this->cam_,
          [](float p, const std::string& msg)
          {
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
                    else if (i==pos)
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
          });

    }


  if(check)
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
  else if(recovery_reboot)
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
              std::cout << "Timed out waiting for producitve mode" << std::endl;
            }
          return -1;
        }
      return 0;
    }
  else if(file)
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

      // Read the file in
      std::shared_ptr<std::istream> ifs;
      std::vector<std::uint8_t> bytes;

      std::string infile = this->vm_["file"].as<std::string>();
      if (infile == "-")
        {
#ifdef _WIN32
          // on windows we need to reopen stdin in binary mode
          _setmode(_fileno(stdin), O_BINARY);
#endif

          ifs.reset(&std::cin, [](...){});

          char b;
          while (ifs->get(b))
            {
              bytes.push_back(*(reinterpret_cast<std::uint8_t*>(&b)));
            }
        }
      else
        {
          ifs.reset(new std::ifstream(infile, std::ios::in|std::ios::binary));
          if (! *ifs)
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
          if(!quiet)
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
  return 0;
}
