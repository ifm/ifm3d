/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/rm_app.h>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

ifm3d::RmApp::RmApp(int argc, const char** argv, const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("index",
     "Index of application to remove",
     cxxopts::value<int>()->default_value("-1"));

  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::RmApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  int idx = (*this->vm_)["index"].as<int>();
  this->cam_->DeleteApplication(idx);

  return 0;
}
