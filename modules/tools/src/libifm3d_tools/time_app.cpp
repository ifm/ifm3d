/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/time_app.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera.h>

ifm3d::TimeApp::TimeApp(int argc, const char** argv, const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format on
  this->all_opts_.add_options(name)(
    "epoch",
    "Secs since Unix epoch encoding time to be set on camera (-1 == now)",
    cxxopts::value<int>());

  // clang-format off
  this->_Parse(argc, argv);
}

int
ifm3d::TimeApp::Run()
{
  if (this->vm_->count("help"))
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
                << ifm3d::O3D_TIME_SUPPORT_PATCH << std::endl;
      return 0;
    }

  if (this->vm_->count("epoch"))
    {
      std::static_pointer_cast<ifm3d::Camera>(this->cam_)->SetCurrentTime((*this->vm_)["epoch"].as<int>());
    }

  dump = this->cam_->ToJSON();
  int curr_time =
    std::stoi(dump["ifm3d"]["Time"]["CurrentTime"].get<std::string>());
  std::time_t curr_time_t = curr_time;
  std::cout << "Local time on camera is: "
            << std::asctime(std::localtime(&curr_time_t)) << std::endl;

  return 0;
}

bool
ifm3d::TimeApp::CheckCompatibility()
{
  return this->cam_->IsO3D() || this->cam_->IsO3X();
}