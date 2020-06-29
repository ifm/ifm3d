/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/reboot_app.h>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

namespace po = boost::program_options;

ifm3d::RebootApp::RebootApp(int argc,
                            const char** argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->local_opts_.add_options()
    ("recovery,r", "Reboot into recovery mode");
  // clang-format on

  po::store(po::command_line_parser(argc, argv)
              .options(this->local_opts_)
              .allow_unregistered()
              .run(),
            this->vm_);
  po::notify(this->vm_);
}

int
ifm3d::RebootApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  ifm3d::Camera::boot_mode mode = this->vm_.count("recovery") ?
                                    ifm3d::Camera::boot_mode::RECOVERY :
                                    ifm3d::Camera::boot_mode::PRODUCTIVE;

  this->cam_->Reboot(mode);

  return 0;
}
