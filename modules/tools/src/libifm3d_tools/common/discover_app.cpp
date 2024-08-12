/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/discover_app.h>
#include <iostream>

ifm3d::DiscoverApp::~DiscoverApp() {}

std::string
ifm3d::DiscoverApp::GetDeviceType(const ifm3d::Device::Ptr& cam)
{
  if (cam->AmI(ifm3d::Device::device_family::O3D))
    return "O3D";
  if (cam->AmI(ifm3d::Device::device_family::O3X))
    return "O3X";
  if (cam->AmI(ifm3d::Device::device_family::O3R))
    return "O3R";
  return "";
}

void
ifm3d::DiscoverApp::Execute(CLI::App* app)
{
  auto devices = ifm3d::Device::DeviceDiscovery();
  std::string parent_app = app->get_parent()->get_name();
  std::stringstream ss;

  if (!devices.empty())
    {
      for (const auto& device : devices)
        {
          auto ip_address = device.GetIPAddress();
          try
            {
              auto cam = ifm3d::Device::MakeShared(ip_address);
              auto device_type = GetDeviceType(cam);
              bool display_device = false;

              if (device_type.empty())
                {
                  ss << ip_address << " (Unsupported device)" << std::endl;
                }
              else
                {
                  if ((device_type == "O3D") && (Parent<ifm3d::O3D3XX>()))
                    {
                      display_device = true;
                    }
                  else if ((device_type == "O3X") &&
                           (Parent<ifm3d::O3X1XX_O3X2XX>()))
                    {
                      display_device = true;
                    }
                  else if ((device_type == "O3R") && (Parent<ifm3d::OVP8xx>()))
                    {
                      display_device = true;
                    }
                  else if (Parent<ifm3d::MainCommand>() &&
                           !(Parent<ifm3d::OVP8xx>()) &&
                           !(Parent<ifm3d::O3X1XX_O3X2XX>()) &&
                           !(Parent<ifm3d::O3D3XX>()))
                    {
                      display_device = true;
                    }

                  if (display_device)
                    {
                      ss << ip_address << " (" << device_type << ")"
                         << " [" << device.GetMACAddress() << "]" << std::endl;
                    }
                }
            }
          catch (ifm3d::Error& e)
            {
              ss << ip_address << " (Unable to identify)" << std::endl;
            }
          catch (std::exception& exp)
            {
              // ignore this device and search for next devices..
            }
        }
      std::cout << ss.str();
    }
  if (devices.empty() || ss.str().empty())
    {
      std::cout << "Info: No devices available" << std::endl;
    }
}

CLI::App*
ifm3d::DiscoverApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("discover", "Discover ifm devices on the network.")
      ->require_subcommand(0, 0);

  return command;
}
