/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <future>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/fg/frame_grabber.h>
#include <ifm3d/fg/organizer.h>
#include <ifm3d/tools/common/fg/jitter_app.h>
#include <iostream>
#include <memory>
#include <numeric>
#include <ratio>
#include <set>
#include <tuple>
#include <type_traits>
#include <vector>

const int ifm3d::FG_JITTER_TIMEOUT = 10000;

// Dummy/minimal image container -- used for testing jitter in
// recieving bytes but not constructing any images
class NullOrganizer : public ifm3d::Organizer
{
public:
  NullOrganizer() = default;

  virtual Result
  Organize(const std::vector<uint8_t>& /*data*/,
           const std::set<ifm3d::buffer_id>& /*requestedImages*/)
  {
    return {};
  };

  virtual std::set<ifm3d::image_chunk>
  GetImageChunks(ifm3d::buffer_id /*id*/)
  {
    return {};
  };
};

// Make sure we use a steady_clock-- prefer the high_resolution_clock
// on platforms where it is steady.
using ClockT =
  std::conditional_t<std::chrono::high_resolution_clock::is_steady,
                     std::chrono::high_resolution_clock,
                     std::chrono::steady_clock>;

namespace
{
  //
  // utility functions for computing summary statistics
  //
  template <typename T>
  T
  median(const std::vector<T>& arr)
  {
    T median = 0;
    std::size_t const sz = arr.size();

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
    if (arr.empty())
      {
        return std::make_tuple(0, 0);
      }

    T sum = std::accumulate(arr.begin(), arr.end(), 0.F);
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
    std::size_t const n = arr.size();
    std::vector<T> arr_d(n);
    for (std::size_t i = 0; i < n; ++i)
      {
        arr_d[i] = std::abs(arr[i] - center);
      }

    return median(arr_d);
  }
}
ifm3d::JitterApp::~JitterApp() = default;

void
ifm3d::JitterApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();

  ifm3d::FrameGrabber::Ptr const fg =
    std::make_shared<ifm3d::FrameGrabber>(device, pcic_port);
  nframes = nframes <= 0 ? 100 : nframes;

  //
  // capture data for computing jitter statistics
  //
  std::vector<float> bb_results(nframes, 0.);
  std::cout << "Capturing frame data..." << '\n';

  fg->Start({});
  CaptureFrames(fg, bb_results);
  float const bb_median = median(bb_results);
  float bb_mean = NAN;
  float bb_stdev = NAN;
  std::tie(bb_mean, bb_stdev) = mean_stdev(bb_results);
  float const bb_mad = mad(bb_results, bb_median);

  std::cout << "Mean:   " << bb_mean << " ms" << '\n';
  std::cout << "Median: " << bb_median << " ms" << '\n';
  std::cout << "Stdev:  " << bb_stdev << " ms" << '\n';
  std::cout << "Mad:    " << bb_mad << " ms" << '\n';

  //
  // compute summary statistics
  //
  if (output_file != "-")
    {
      std::ofstream out;
      out.open(output_file);

      // headers in the csv
      out << "ByteBuffer";

      out << '\n';

      // csv data
      for (int i = 0; i < nframes; ++i)
        {
          out << bb_results[i];
          out << '\n';
        }
      out.close();

      std::cout << "Raw data has been written to: " << output_file << '\n';
    }
}

CLI::App*
ifm3d::JitterApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("jitter",
                       "Collects statistics on framegrabber (and optionally, "
                       "image construction) jitter.")
      ->require_subcommand(0, 0);

  command
    ->add_option("--pcic-port",
                 this->pcic_port,
                 "port number for pcic communication")
    ->default_val(ifm3d::DEFAULT_PCIC_PORT);

  command
    ->add_option("--nframes", this->nframes, "Number of frames to capture")
    ->default_val(100);

  command->add_option(
    "--outfile",
    this->output_file,
    "Raw data output file, if not specified, nothing is written");

  return command;
}

void
ifm3d::JitterApp::CaptureFrames(const ifm3d::FrameGrabber::Ptr& fg,
                                std::vector<float>& results)
{
  int const nframes = static_cast<int>(results.size());

  // get one-time allocations out of the way, and, make
  // sure it doesn't get optimized away by the compiler
  if (fg->WaitForFrame().wait_for(std::chrono::milliseconds(
        ifm3d::FG_JITTER_TIMEOUT)) != std::future_status::ready)
    {
      std::cerr << "Timeout waiting for first image acquisition!" << '\n';
      return;
    }

  for (int i = 0; i < nframes; ++i)
    {
      auto t1 = ClockT::now();
      auto future = fg->WaitForFrame();
      if (future.wait_for(std::chrono::milliseconds(
            ifm3d::FG_JITTER_TIMEOUT)) != std::future_status::ready)
        {
          std::cerr << "Timeout waiting for image acquisition!" << '\n';
          return;
        }
      future.get()->TimeStamps();
      auto t2 = ClockT::now();

      std::chrono::duration<float, std::milli> const fp_ms = t2 - t1;
      results[i] = fp_ms.count();
      std::cout << fp_ms.count() << '\n';
    }
}
