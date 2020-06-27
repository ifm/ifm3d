/*
 * Copyright (C) 2017 Love Park Robotics, LLC
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

#include <ifm3d/tools/reboot_app.h>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

namespace po = boost::program_options;

ifm3d::RebootApp::RebootApp(int argc, const char **argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->local_opts_.add_options()
    ("recovery,r", "Reboot into recovery mode");
  // clang-format on

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
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

  ifm3d::Camera::boot_mode mode =
    this->vm_.count("recovery") ?
    ifm3d::Camera::boot_mode::RECOVERY :
    ifm3d::Camera::boot_mode::PRODUCTIVE;

  this->cam_->Reboot(mode);

  return 0;
}
