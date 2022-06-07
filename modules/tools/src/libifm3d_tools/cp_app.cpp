/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/cp_app.h>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device.h>

ifm3d::CpApp::CpApp(int argc, const char** argv, const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("index",
     "Index of source application to copy", 
     cxxopts::value<int>()->default_value("-1"));
  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::CpApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }
 
  int idx = (*this->vm_)["index"].as<int>();
  std::static_pointer_cast<ifm3d::LegacyDevice>(this->cam_)->CopyApplication(idx);
  return 0;
}

bool
ifm3d::CpApp::CheckCompatibility()
{
  return this->cam_->AmI(Device::device_family::O3D) ||
         this->cam_->AmI(Device::device_family::O3X);
}