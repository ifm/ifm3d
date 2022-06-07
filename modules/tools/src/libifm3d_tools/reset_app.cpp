/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/reset_app.h>
#include <iostream>
#include <memory>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device.h>

ifm3d::ResetApp::ResetApp(int argc, const char** argv, const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("r,reboot", "Reboot the sensor after reset");

  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::ResetApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  std::static_pointer_cast<ifm3d::LegacyDevice>(this->cam_)->FactoryReset();
  if (this->vm_->count("reboot"))
    {
      this->cam_->Reboot();
    }

  return 0;
}

bool
ifm3d::ResetApp::CheckCompatibility()
{
  return this->cam_->AmI(Device::device_family::O3D) ||
         this->cam_->AmI(Device::device_family::O3X);
}