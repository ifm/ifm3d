/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/legacy/time_app.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <ifm3d/device.h>

ifm3d::TimeApp::~TimeApp() {}

void
ifm3d::TimeApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  json dump = device->ToJSON();
  if (dump["/ifm3d/Time"_json_pointer].empty())
    {
      std::cout << "Time support currently requires an O3X or "
                << "an O3D3XX with firmware >= "
                << ifm3d::O3D_TIME_SUPPORT_MAJOR << "."
                << ifm3d::O3D_TIME_SUPPORT_MINOR << "."
                << ifm3d::O3D_TIME_SUPPORT_PATCH << std::endl;
    }

  if (opt_epoch->count())
    {
      std::static_pointer_cast<ifm3d::LegacyDevice>(device)->SetCurrentTime(
        epoch);
    }

  dump = device->ToJSON();
  int curr_time =
    std::stoi(dump["ifm3d"]["Time"]["CurrentTime"].get<std::string>());
  std::time_t curr_time_t = curr_time;
  std::cout << "Local time on camera is: "
            << std::asctime(std::localtime(&curr_time_t)) << std::endl;
}

CLI::App*
ifm3d::TimeApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("time", "Get/set the current time on the camera.")
      ->require_subcommand(0, 0);
  opt_epoch = command->add_option(
    "--epoch",
    this->epoch,
    "Secs since Unix epoch encoding time to be set on camera (-1 == now)");

  return command;
}
