/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/fg/hz_app.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <ifm3d/tools/fg/fg_app.h>
#include <ifm3d/device.h>
#include <ifm3d/fg.h>

// Dummy/minimal image container -- used for calculating hz
// recieving bytes but not constructing any images
class NullOrganizer : public ifm3d::Organizer
{
public:
  NullOrganizer() : ifm3d::Organizer() {}

  virtual Result
  Organize(const std::vector<uint8_t>& data,
           const std::set<ifm3d::buffer_id>& requestedImages)
  {
    return {};
  };

  virtual std::set<ifm3d::image_chunk>
  GetImageChunks(ifm3d::buffer_id id)
  {
    return {};
  };
};

ifm3d::HzApp::HzApp(int argc, const char** argv, const std::string& name)
  : ifm3d::FgApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("nframes",
     "Number of frames to capture",
     cxxopts::value<int>()->default_value("10"))
    ("nruns",
     "Number of runs to compute summary statistics over",
     cxxopts::value<int>()->default_value("1"))
    ("sw",
     "Software Trigger the FrameGrabber");
  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::HzApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  bool sw_trigger = this->vm_->count("sw") ? true : false;
  int nframes = (*this->vm_)["nframes"].as<int>();
  int nruns = (*this->vm_)["nruns"].as<int>();

  double median = 0.0;

  if (nframes <= 0)
    {
      nframes = 10;
    }

  std::vector<double> stats;

  this->fg_->Start({});
  for (int i = 0; i < nruns; i++)
    {
      auto start = std::chrono::steady_clock::now();
      for (int j = 0; j < nframes; j++)
        {
          if (sw_trigger)
            {
              this->fg_->SWTrigger();
            }

          auto future = this->fg_->WaitForFrame();
          if (future.wait_for(std::chrono::milliseconds(FG_TIMEOUT)) !=
              std::future_status::ready)
            {
              std::cerr << "Timeout waiting for camera!" << std::endl;
              return -1;
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

      std::cout << "FrameGrabber running at: " << nframes / median << " Hz"
                << std::endl
                << nframes << " frames captured, over " << nruns << " runs"
                << std::endl;
    }

  return 0;
}
