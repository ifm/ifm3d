/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/dump_app.h>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>

ifm3d::DumpApp::DumpApp(int argc, const char** argv, const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{}

int
ifm3d::DumpApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  std::cout << this->cam_->ToJSONStr() << std::endl;

  return 0;
}
