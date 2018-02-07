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

#include <ifm3d/tools/time_app.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

namespace po = boost::program_options;

ifm3d::TimeApp::TimeApp(int argc, const char **argv,
                        const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("epoch", po::value<int>(),
     "Secs since Unix epoch encoding time to be set on camera (-1 == now)");

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

int ifm3d::TimeApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  json dump = this->cam_->ToJSON();
  if (dump["/ifm3d/Time"_json_pointer].empty())
    {
      std::cout << "Time support currently requires an O3X or "
                << "an O3D3XX with firmware >= "
                << ifm3d::O3D_TIME_SUPPORT_MAJOR << "."
                << ifm3d::O3D_TIME_SUPPORT_MINOR << "."
                << ifm3d::O3D_TIME_SUPPORT_PATCH
                << std::endl;
      return 0;
    }

  if (this->vm_.count("epoch"))
    {
      this->cam_->SetCurrentTime(this->vm_["epoch"].as<int>());
    }

  dump = this->cam_->ToJSON();
  int curr_time =
    std::stoi(dump["ifm3d"]["Time"]["CurrentTime"].get<std::string>());
  std::time_t curr_time_t = curr_time;
  std::cout << "Local time on camera is: "
            << std::asctime(std::localtime(&curr_time_t))
            << std::endl;

  return 0;
}
