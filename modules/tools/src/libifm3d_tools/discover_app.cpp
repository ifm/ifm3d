/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/discover_app.h>
#include <iostream>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>

ifm3d::DiscoverApp::DiscoverApp(int argc,
                                const char** argv,
                                const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name, false)
{}

int
ifm3d::DiscoverApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto devices = ifm3d::Device::DeviceDiscovery();

  if (devices.empty())
    {
      std::cout << "Info: No devices available" << std::endl;
    }

  for (const auto& device : devices)
    {
      auto ip_address = device.GetIPAddress();
      try
        {
          auto cam = ifm3d::Device::MakeShared(ip_address);
          auto device_type = "";
          if (cam->AmI(ifm3d::Device::device_family::O3D))
            {
              device_type = "O3D";
            }
          else if (cam->AmI(ifm3d::Device::device_family::O3X))
            {
              device_type = "O3X";
            }
          else if (cam->AmI(ifm3d::Device::device_family::O3R))
            {
              device_type = "O3R";
            }
          else
            {
              std::cout << ip_address << " (Unsupported device)" << std::endl;
              continue;
            }
          std::cout << ip_address << " (" << device_type << ")"
                    << " [" << device.GetMACAddress() << "]" << std::endl;
        }
      catch (ifm3d::Error& e)
        {
          std::cout << ip_address << "(Unable to identify)" << std::endl;
        }
      catch (std::exception& exp)
        {
          // ignore this device and search for next devices..
        }
    }
  return 0;
}
