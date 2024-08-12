/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/common/fg/hz_app.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <ifm3d/device.h>
#include <ifm3d/fg.h>

const int ifm3d::FG_TIMEOUT = 10000;

ifm3d::HzApp::~HzApp() {}

void
ifm3d::HzApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  ifm3d::FrameGrabber::Ptr fg =
    std::make_shared<ifm3d::FrameGrabber>(device, pcic_port);

  double median = 0.0;

  if (this->nframes <= 0)
    {
      this->nframes = 10;
    }

  std::vector<double> stats;

  fg->Start({});
  for (int i = 0; i < this->nruns; i++)
    {
      auto start = std::chrono::steady_clock::now();
      for (int j = 0; j < this->nframes; j++)
        {
          if (this->sw_trigger)
            {
              fg->SWTrigger();
            }

          auto future = fg->WaitForFrame();
          if (future.wait_for(std::chrono::milliseconds(FG_TIMEOUT)) !=
              std::future_status::ready)
            {
              std::cerr << "Timeout waiting for camera!" << std::endl;
              // return -1;
            }
          future.get()->TimeStamps();
        }
      auto stop = std::chrono::steady_clock::now();
      auto diff = stop - start;
      stats.push_back(std::chrono::duration<double>(diff).count());
    }

  if (nruns >= 1)
    {
      std::size_t sz = stats.size();
      std::sort(stats.begin(), stats.end());

      if (sz % 2 == 0)
        {
          median = (stats.at(sz / 2 - 1) + stats.at(sz / 2)) / 2;
        }
      else
        {
          median = stats.at(sz / 2);
        }

      std::cout << "FrameGrabber running at: " << this->nframes / median
                << " Hz" << std::endl
                << this->nframes << " frames captured, over " << this->nruns
                << " runs" << std::endl;
    }
}

CLI::App*
ifm3d::HzApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand(
        "hz",
        "Compute the actual frequency at which the FrameGrabber is running.")
      ->require_subcommand(0, 0);

  command
    ->add_option("--pcic-port",
                 pcic_port,
                 "port number for pcic communication")
    ->default_val(ifm3d::DEFAULT_PCIC_PORT);

  command
    ->add_option("--nframes", this->nframes, "Number of frames to capture")
    ->default_val(10);

  command
    ->add_option("--nruns",
                 this->nruns,
                 "Number of runs to compute summary statistics over")
    ->default_val(1);

  command
    ->add_flag("--sw",
               this->sw_trigger,
               "Software Trigger the FrameGrabber. Default: false")
    ->default_val(false)
    ->default_str("flag");

  return command;
}
