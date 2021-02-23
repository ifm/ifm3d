/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/discover_app.h>
#include <iostream>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

ifm3d::DiscoverApp::DiscoverApp(int argc,
                                const char** argv,
                                const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  all_opts_ = cxxopts::Options("ifm3d");

  // clang-format off
  this-> all_opts_.add_options("global")
    ("h,help","Produce this help message and exit");
  this->all_opts_.add_options(name)
    ("mac","MAC address of the device",
     cxxopts::value<std::string>()->default_value(""))
    ("tempip","IP address",
     cxxopts::value<std::string>()->default_value(""));

  // clang-format on
  this->_Parse(argc, argv);
}

void
ifm3d::DiscoverApp::_LocalHelp()
{
  std::string cmd = "discover";
  this->all_opts_.custom_help("[<global options>] " + cmd + " [<" + cmd +
                              " options>]");
  std::cout << this->all_opts_.help({"global", cmd}) << std::endl;
}

int
ifm3d::DiscoverApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto mac = (*this->vm_)["mac"].as<std::string>();
  auto temp_ip = (*this->vm_)["tempip"].as<std::string>();

  std::vector<ifm3d::IFMNetworkDevice> devices;

  if (temp_ip == "")
    {
      devices = ifm3d::Camera::DeviceDiscovery();
    }
  else if (mac != "")
    {
      devices = ifm3d::Camera::SetTempIpAddress(mac, temp_ip);
    }
  else
    {
      std::cout << "mac is needed with tempip" << std::endl;
      return 0;
    }

  for (const auto& device : devices)
    {
      try
        {
          auto ip_address = device.GetIPAddress();
          auto dev_mac = device.GetMACAddress();

          // mac filtering
          if (mac != "" && mac != dev_mac)
            {
              continue;
            }
          std::cout << ip_address << " mac = " << dev_mac;
          auto cam = ifm3d::Camera::MakeShared(ip_address);
          auto device_type = "unknown";
          if (cam->IsO3D())
            {
              device_type = "O3D";
            }
          else if (cam->IsO3X())
            {
              device_type = "O3X";
            }
          std::cout << " (" << device_type << ")" << std::endl;
        }
      catch (std::exception& /*exp */)
        {
          // ignore this device and search for next devices..
        }
    }
  return 0;
}
