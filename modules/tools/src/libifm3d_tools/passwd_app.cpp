/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/passwd_app.h>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device.h>

ifm3d::PasswdApp::PasswdApp(int argc,
                            const char** argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("new","password to be set on sensor",
     cxxopts::value<std::string>())
    ("disable","disable password on sensor",
     cxxopts::value<bool>()->default_value("false"));

  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::PasswdApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto const new_password = this->vm_->count("new") ? true : false;
  auto const disable = (*this->vm_)["disable"].as<bool>();
  std::string password = "";

  if (new_password && disable)
    {
      std::cerr << "invalid option combination" << std::endl;
      throw cxxopts::exceptions::specification("invalid options combination");
    }
  else if (new_password)
    {
      password = (*this->vm_)["new"].as<std::string>();
      std::static_pointer_cast<ifm3d::LegacyDevice>(this->cam_)
        ->SetPassword(password);
    }
  else if (disable)
    {
      std::static_pointer_cast<ifm3d::LegacyDevice>(this->cam_)
        ->SetPassword(password);
    }
  return 0;
}

bool
ifm3d::PasswdApp::CheckCompatibility()
{
  return this->cam_->AmI(Device::device_family::O3D) ||
         this->cam_->AmI(Device::device_family::O3X);
}