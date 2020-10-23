/*
 * Copyright (C) 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/discover_app.h>
#include <iostream>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

ifm3d::DiscoverApp::DiscoverApp(int argc, const char **argv,
  const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{ }

int ifm3d::DiscoverApp::Run()
{
  if (this->vm_->count("help"))
   {
     this->_LocalHelp();
     return 0;
   }

  auto devices = ifm3d::Camera::DeviceDiscovery();

  for (const auto& device : devices)
    {
      auto ip_address = device.GetIPAddress();
      auto cam = ifm3d::Camera::MakeShared(ip_address);
      auto device_type = "";
      if (cam->IsO3D())
        {
          device_type = "O3D";
        }
      else if (cam->IsO3X())
        {
          device_type = "O3X";
        }
      else
        {
          continue;
        }
      std::cout << ip_address << " (" << device_type << ")" << std::endl;
    }
  return 0;
}
