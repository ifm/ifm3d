/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include "ifm3d/common/err.h"
#include "ifm3d/device/device.h"
#include <CLI/App.hpp>
#include "ifm3d/tools/legacy/o3d3xx_app.h"
#include "ifm3d/tools/legacy/o3x_app.h"
#include "ifm3d/tools/ovp8xx/ovp8xx_app.h"
#include "ifm3d/tools/main_command.hpp"
#include <exception>
#include <ifm3d/tools/common/discover_app.h>
#include <iostream>
#include <string>
#include <sstream>
#include <unordered_map>

constexpr uint32_t BCAST_FLAG_WRONGSUBNET = 0x0001;

ifm3d::DiscoverApp::~DiscoverApp() = default;

std::string
ifm3d::DiscoverApp::GetDeviceType(const ifm3d::Device::Ptr& cam)
{
  if (cam->AmI(ifm3d::Device::device_family::O3D))
    {
      return "O3D";
    }
  if (cam->AmI(ifm3d::Device::device_family::O3X))
    {
      return "O3X";
    }
  if (cam->AmI(ifm3d::Device::device_family::O3R))
    {
      return "O3R";
    }
  return "";
}

void
ifm3d::DiscoverApp::Execute(CLI::App* /*app*/)
{
  if (!(this->subcmd_set_temp_ip->parsed()))
    {
      auto devices = ifm3d::Device::DeviceDiscovery();
      std::stringstream ss;

      std::unordered_map<std::string, decltype(devices)::value_type>
        unique_devices;
      for (const auto& device : devices)
        {
          auto mac = device.GetMACAddress();
          auto existing = unique_devices.find(mac);

          if (existing == unique_devices.end() ||
              (existing->second.GetFlag() == BCAST_FLAG_WRONGSUBNET &&
               device.GetFlag() != BCAST_FLAG_WRONGSUBNET))
            {
              unique_devices.emplace(mac, device);
            }
        }

      if (!unique_devices.empty())
        {
          for (const auto& [mac, device] : unique_devices)
            {
              auto ip_address = device.GetIPAddress();
              try
                {
                  if (device.GetFlag() == BCAST_FLAG_WRONGSUBNET)
                    {
                      ss << ip_address << " [" << device.GetMACAddress()
                         << "] (Device is in different "
                            "subnet, set temporary IP to connect. Use "
                            "'set-temporary-ip' subcommand) "
                         << '\n';
                    }
                  else
                    {
                      auto cam = ifm3d::Device::MakeShared(ip_address);
                      auto device_type = GetDeviceType(cam);
                      bool display_device = false;
                      if (device_type.empty())
                        {
                          ss << ip_address << " (Unsupported device)" << '\n';
                        }
                      else
                        {
                          if ((device_type == "O3D" &&
                               Parent<ifm3d::O3D3XX>()) ||
                              (device_type == "O3X" &&
                               Parent<ifm3d::O3X1XX_O3X2XX>()) ||
                              (device_type == "O3R" &&
                               Parent<ifm3d::OVP8xx>()) ||
                              (Parent<ifm3d::MainCommand>() &&
                               !(Parent<ifm3d::OVP8xx>()) &&
                               !(Parent<ifm3d::O3X1XX_O3X2XX>()) &&
                               !(Parent<ifm3d::O3D3XX>())))
                            {
                              display_device = true;
                            }
                          if (display_device)
                            {
                              ss << ip_address << " (" << device_type << ")"
                                 << " [" << device.GetMACAddress() << "]"
                                 << '\n';
                            }
                        }
                    }
                }
              catch (ifm3d::Error& e)
                {
                  if (e.code() == ifm3d::Error(IFM3D_XMLRPC_TIMEOUT).code())
                    {
                      ss << ip_address << " (Unable to identify) " << '\n';
                    }
                }
              catch (std::exception& exp)
                {
                  // IGNORE: ignore this device and search for next devices..
                }
            }
          std::cout << ss.str();
        }
      if (devices.empty() || ss.str().empty())
        {
          std::cout << "Info: No devices available" << '\n';
        }
    }
}

CLI::App*
ifm3d::DiscoverApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("discover", "Discover ifm devices on the network.")
      ->require_subcommand(0, 1);

  subcmd_set_temp_ip =
    RegisterSubcommand<ifm3d::SetTemporaryIPApp>(command)->GetSubcommandApp();

  return command;
}
