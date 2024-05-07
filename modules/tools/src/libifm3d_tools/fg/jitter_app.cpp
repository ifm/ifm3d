/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/fg/jitter_app.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <ratio>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device.h>
#include <ifm3d/fg.h>

// Dummy/minimal image container -- used for testing jitter in
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

// Make sure we use a steady_clock -- prefer the high_resolution_clock
// on platforms where it is steady.
using Clock_t = std::conditional<std::chrono::high_resolution_clock::is_steady,
                                 std::chrono::high_resolution_clock,
                                 std::chrono::steady_clock>::type;

ifm3d::JitterApp::JitterApp(int argc,
                            const char** argv,
                            const std::string& name)
  : ifm3d::FgApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("nframes",
     "Number of frames to capture",
     cxxopts::value<int>()->default_value("100") )
    ("outfile",
     "Raw data output file, if not specified, nothing is written",
     cxxopts::value<std::string>()->default_value("-"));

  //clang-format on
  this->_Parse(argc, argv);
}

//
// utility functions for computing summary statistics
//
template <typename T>
T
median(const std::vector<T>& arr)
{
  T median = 0;
  std::size_t sz = arr.size();

  // we make a copy b/c we do not want to mutate the input `arr`.
  std::vector<T> arr_cp(sz);
  std::copy(arr.begin(), arr.end(), arr_cp.begin());
  std::sort(arr_cp.begin(), arr_cp.end());

  if (sz > 0)
    {
      if (sz % 2 == 0)
        {
          median = (arr_cp.at((sz / 2) - 1) + arr_cp.at(sz / 2)) / 2.;
        }
      else
        {
          median = arr_cp.at(sz / 2);
        }
    }

  return median;
}

template <typename T>
std::tuple<T, T>
mean_stdev(const std::vector<T>& arr)
{
  if (arr.size() < 1)
    {
      return std::make_tuple(0, 0);
    }

  T sum = std::accumulate(arr.begin(), arr.end(), 0.f);
  T mean = sum / arr.size();

  T ssd = 0;
  std::for_each(arr.begin(), arr.end(), [&](const T val) -> void {
    ssd += ((val - mean) * (val - mean));
  });

  T stdev = std::sqrt(ssd / (arr.size() - 1));
  return std::make_tuple(mean, stdev);
}

template <typename T>
T
mad(const std::vector<T>& arr, T center)
{
  std::size_t n = arr.size();
  std::vector<T> arr_d(n);
  for (int i = 0; i < n; ++i)
    {
      arr_d[i] = std::abs(arr[i] - center);
    }

  return median(arr_d);
}

//
// Captures the timing metrics
//
void
capture_frames(ifm3d::FrameGrabber::Ptr fg, std::vector<float>& results)
{
  int nframes = results.size();

  // get one-time allocations out of the way, and, make
  // sure it doesn't get optimized away by the compiler
  if (fg->WaitForFrame().wait_for(std::chrono::milliseconds(ifm3d::FG_TIMEOUT)) != std::future_status::ready)
    {
      std::cerr << "Timeout waiting for first image acquisition!" << std::endl;
      return;
    }

  for (int i = 0; i < nframes; ++i)
    {
      auto t1 = Clock_t::now();
      auto future = fg->WaitForFrame();
      if (future.wait_for(std::chrono::milliseconds(ifm3d::FG_TIMEOUT)) !=
          std::future_status::ready)
        {
          std::cerr << "Timeout waiting for image acquisition!" << std::endl;
          return;
        }
      future.get()->TimeStamps();
      auto t2 = Clock_t::now();

      std::chrono::duration<float, std::milli> fp_ms = t2 - t1;
      results[i] = fp_ms.count();
      std::cout << fp_ms.count() << std::endl;
    }
}

//
// here is our "main()"
//
int
ifm3d::JitterApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  //this->fg_->SetOrganizer(std::make_unique<NullOrganizer>());

  int nframes = (*this->vm_)["nframes"].as<int>();
  nframes = nframes <= 0 ? 100 : nframes;
  std::string outfile = (*this->vm_)["outfile"].as<std::string>();

  //
  // capture data for computing jitter statistics
  //
  std::vector<float> bb_results(nframes, 0.);
  std::cout << "Capturing frame data..." << std::endl;

  this->fg_->Start({});
  capture_frames(this->fg_, bb_results);
  float bb_median = median(bb_results);
  float bb_mean, bb_stdev;
  std::tie(bb_mean, bb_stdev) = mean_stdev(bb_results);
  float bb_mad = mad(bb_results, bb_median);

  std::cout << "Mean:   " << bb_mean << " ms" << std::endl;
  std::cout << "Median: " << bb_median << " ms" << std::endl;
  std::cout << "Stdev:  " << bb_stdev << " ms" << std::endl;
  std::cout << "Mad:    " << bb_mad << " ms" << std::endl;


  //
  // compute summary statistics
  //
  if (outfile != "-")
    {
      std::ofstream out;
      out.open(outfile);

      // headers in the csv
      out << "ByteBuffer";

      out << std::endl;

      // csv data
      for (int i = 0; i < nframes; ++i)
        {
          out << bb_results[i];
          out << std::endl;
        }
      out.close();

      std::cout << "Raw data has been written to: " << outfile << std::endl;
    }

  return 0;
}
