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

#include <ifm3d/tools/reset_app.h>
#include <iostream>
#include <memory>
#include <string>
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

namespace po = boost::program_options;

ifm3d::ResetApp::ResetApp(int argc, const char **argv,
                          const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("reboot,r", "Reboot the sensor after reset");

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

int
ifm3d::ResetApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  this->cam_->FactoryReset();
  if (this->vm_.count("reboot"))
    {
      this->cam_->Reboot();
    }

  return 0;
}
