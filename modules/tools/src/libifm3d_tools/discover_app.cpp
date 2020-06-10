/*
 * Copyright (C) 2020 ifm electronic GmbH
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
    auto cam = ifm3d::Camera::MakeShared(device);
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
    std::cout << device << " (" << device_type << ")" << std::endl;
  }
  return 0;
}
