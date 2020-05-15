/*
 * Copyright (C) 2018 ifm electronic, gmbh
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
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>

#if defined(BUILD_MODULE_IMAGE)
#include <ifm3d/image.h>
#endif

#if defined(BUILD_MODULE_OPENCV)
#include <ifm3d/opencv.h>
#endif

// Make sure we use a steady_clock -- prefer the high_resolution_clock
// on platforms where it is steady.
using Clock_t = std::conditional<std::chrono::high_resolution_clock::is_steady,
                                 std::chrono::high_resolution_clock,
                                 std::chrono::steady_clock>::type;

// Dummy/minimal image container -- used for testing jitter in
// recieving bytes but not constructing any image containers
class MyBuff : public ifm3d::ByteBuffer<MyBuff>
{
public:
  MyBuff() : ifm3d::ByteBuffer<MyBuff>()
  { }

  template <typename T>
  void ImCreate(ifm3d::image_chunk im,
                std::uint32_t fmt,
                std::size_t idx,
                std::uint32_t width,
                std::uint32_t height,
                int nchan,
                std::uint32_t npts,
                const std::vector<std::uint8_t>& bytes)
  { }

  template <typename T>
  void CloudCreate(std::uint32_t fmt,
                   std::size_t xidx,
                   std::size_t yidx,
                   std::size_t zidx,
                   std::uint32_t width,
                   std::uint32_t height,
                   std::uint32_t npts,
                   const std::vector<std::uint8_t>& bytes)
  { }
};

ifm3d::JitterApp::JitterApp(int argc, const char **argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("nframes",
     po::value<int>()->default_value(100),
     "Number of frames to capture")
    ("outfile",
     po::value<std::string>()->default_value("-"),
     "Raw data output file, if not specified, nothing is written");

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

//
// utility functions for computing summary statistics
//
template <typename T>
T median(const std::vector<T>& arr)
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
          median = (arr_cp.at((sz/2)-1)+arr_cp.at(sz/2))/2.;
        }
      else
        {
          median = arr_cp.at(sz/2);
        }
    }

  return median;
}

template <typename T>
std::tuple<T, T> mean_stdev(const std::vector<T>& arr)
{
  if (arr.size() < 1)
    {
      return std::make_tuple(0, 0);
    }

  T sum = std::accumulate(arr.begin(), arr.end(), 0.f);
  T mean = sum / arr.size();

  T ssd = 0;
  std::for_each(arr.begin(), arr.end(),
                [&](const T val) -> void
                { ssd += ((val - mean) * (val - mean)); });

  T stdev = std::sqrt(ssd / (arr.size()-1));
  return std::make_tuple(mean, stdev);
}

template <typename T>
T mad(const std::vector<T>& arr, T center)
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
template <typename T>
void capture_frames(ifm3d::Camera::Ptr cam, T buff, std::vector<float>& results)
{
  int nframes = results.size();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  // get one-time allocations out of the way, and, make
  // sure it doesn't get optimized away by the compiler
  if (! fg->WaitForFrame(buff.get(), 1000))
    {
      std::cerr << "Timeout waiting for first image acquisition!" << std::endl;
      return;
    }

  for (int i = 0; i < nframes; ++i)
    {
      auto t1 = Clock_t::now();
      if (! fg->WaitForFrame(buff.get(), 1000))
        {
          std::cerr << "Timeout waiting for image acquisition!" << std::endl;
          return;
        }
      auto t2 = Clock_t::now();

      std::chrono::duration<float, std::milli> fp_ms = t2 - t1;
      results[i] = fp_ms.count();
    }
}

//
// here is our "main()"
//
int ifm3d::JitterApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  int nframes = this->vm_["nframes"].as<int>();
  nframes = nframes <= 0 ? 100 : nframes;
  std::string outfile = this->vm_["outfile"].as<std::string>();

  //
  // capture data for computing jitter statistics
  //
  std::vector<float> bb_results(nframes, 0.);
  std::cout << "Capturing frame data for ifm3d::ByteBuffer..." << std::endl;
  auto bb = std::make_shared<ifm3d::ByteBuffer<MyBuff> >();
  capture_frames(this->cam_, bb, bb_results);
  float bb_median = median(bb_results);
  float bb_mean, bb_stdev;
  std::tie(bb_mean, bb_stdev) = mean_stdev(bb_results);
  float bb_mad = mad(bb_results, bb_median);

  std::cout << "Mean:   " << bb_mean << " ms" << std::endl;
  std::cout << "Median: " << bb_median << " ms" << std::endl;
  std::cout << "Stdev:  " << bb_stdev << " ms" << std::endl;
  std::cout << "Mad:    " << bb_mad << " ms" << std::endl;

#if defined(BUILD_MODULE_IMAGE)
  std::vector<float> im_results(nframes, 0.);
  std::cout << std::endl
            << "Capturing frame data for ifm3d::ImageBuffer..." << std::endl;
  auto im = std::make_shared<ifm3d::ImageBuffer>();
  capture_frames(this->cam_, im, im_results);
  float im_median = median(im_results);
  float im_mean, im_stdev;
  std::tie(im_mean, im_stdev) = mean_stdev(im_results);
  float im_mad = mad(im_results, im_median);

  std::cout << "Mean:   " << im_mean << " ms" << std::endl;
  std::cout << "Median: " << im_median << " ms" << std::endl;
  std::cout << "Stdev:  " << im_stdev << " ms" << std::endl;
  std::cout << "Mad:    " << im_mad << " ms" << std::endl;
#endif

#if defined(BUILD_MODULE_OPENCV)
  std::vector<float> oc_results(nframes, 0.);
  std::cout << std::endl
            << "Capturing frame data for ifm3d::OpenCVBuffer..." << std::endl;
  auto oc = std::make_shared<ifm3d::OpenCVBuffer>();
  capture_frames(this->cam_, oc, oc_results);
  float oc_median = median(oc_results);
  float oc_mean, oc_stdev;
  std::tie(oc_mean, oc_stdev) = mean_stdev(oc_results);
  float oc_mad = mad(oc_results, oc_median);

  std::cout << "Mean:   " << oc_mean << " ms" << std::endl;
  std::cout << "Median: " << oc_median << " ms" << std::endl;
  std::cout << "Stdev:  " << oc_stdev << " ms" << std::endl;
  std::cout << "Mad:    " << oc_mad << " ms" << std::endl;
#endif


  //
  // compute summary statistics
  //
  if (outfile != "-")
    {
      std::ofstream out;
      out.open(outfile);

      // headers in the csv
      out << "ByteBuffer";

#if defined(BUILD_MODULE_IMAGE)
      out << ",ImageBuffer";
#endif

#if defined(BUILD_MODULE_OPENCV)
      out << ",OpenCVBuffer";
#endif
      out << std::endl;

      // csv data
      for (int i = 0; i < nframes; ++i)
        {
          out << bb_results[i];

#if defined(BUILD_MODULE_IMAGE)
          out << "," << im_results[i];
#endif

#if defined(BUILD_MODULE_OPENCV)
          out << "," << oc_results[i];
#endif

          out << std::endl;
        }
      out.close();

      std::cout << "Raw data has been written to: " << outfile << std::endl;
    }

  return 0;
}
