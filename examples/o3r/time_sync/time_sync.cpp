/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 * 
 * This example mimics a multi-camera synchronization with no trigger
 * functionality: two camera heads are switched to "RUN" mode 
 * simultaneously. Frames are grabbed from each head in parallel.
 * The timestamps are used to calculate and display the time delay
 * between the reception of the frames from the two heads.
 */
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <thread>
#include <mutex>
#include <fmt/core.h>

#include <ifm3d/device/o3r.h>
#include <ifm3d/fg.h>

std::mutex mutex;  

std::string formatTimestamp(ifm3d::TimePointT timestamp)
{
  /** 
  * This function formats the timestamps for proper display
  * a.k.a converts to local time
  */
    using namespace std::chrono;
    std::time_t time = std::chrono::system_clock::to_time_t(
        std::chrono::time_point_cast<std::chrono::system_clock::duration>(
          timestamp));

    milliseconds milli = duration_cast<milliseconds>(
          timestamp.time_since_epoch() - duration_cast<seconds>(
            timestamp.time_since_epoch()));

    std::ostringstream s;
    s << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
      << ":" << std::setw(3) << std::setfill('0') << milli.count();

    return s.str();
}

int getFrameTime(ifm3d::FrameGrabber::Ptr fg, ifm3d::TimePointT &t, int &ready){
  /**
   * Grab frame from a camera head and update global variable 
   * with the timestamp of the frame.
   * When the frame has been captured, switches the global variable ready
   * to release the display thread.
   */
  int i = 0;
  while (true) {
    auto frame = fg->WaitForFrame();
    if (frame.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready)
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }
    else {
      i+=1;
      if (i == 10){
        std::lock_guard<std::mutex> guard(mutex);
        t = frame.get()->TimeStamps().front();
        i = 0;
        ready = 1;
      }
    }
  }
  return 1;
}

int main(){
  //////////////////////////
  // Declare the objects:
  //////////////////////////
  // Declare the device object (one object only, corresponding to the VPU)
  auto o3r = std::make_shared<ifm3d::O3R>();
  ifm3d::json conf = o3r->Get();
  // Declare the FrameGrabber and ImageBuffer objects. 
  // One FrameGrabber per camera head (define the port number).
  auto fg0 = std::make_shared<ifm3d::FrameGrabber>(o3r, 50012);
  auto fg1 = std::make_shared<ifm3d::FrameGrabber>(o3r, 50013);

  //Start the framegrabbber with empty schema
  fg0->Start({});
  fg1->Start({});

  std::thread thread0;
  std::thread thread1;
  //////////////////////////
  // Set framerate:
  //////////////////////////
  o3r->Set(ifm3d::json::parse(R"({"ports":{"port2":{"acquisition": {"framerate": 10}}, 
                                    "port3":{"acquisition": {"framerate": 10}}}})"));


  //////////////////////////
  // Start the cameras at
  // the same time
  /////////////////////////
  o3r->Set(ifm3d::json::parse(R"({"ports":{"port2":{"state": "CONF"}, 
                                    "port3":{"state": "CONF"}}})"));
  std::this_thread::sleep_for(std::chrono::seconds(1));
  o3r->Set(ifm3d::json::parse(R"({"ports":{"port2":{"state": "RUN"}, 
                                    "port3":{"state": "RUN"}}})"));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Check which frame comes first:
  std::vector<ifm3d::TimePointT> timestamps;
  std::vector<ifm3d::FrameGrabber::Ptr> fgs;
  fgs.push_back(fg0); fgs.push_back(fg1);
  for (auto fg: fgs) {
    auto frame = fg->WaitForFrame();
    if (frame.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready)
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }
    else {
      timestamps.push_back(frame.get()->TimeStamps().front());
    }
  }
  ifm3d::TimePointT t0;
  ifm3d::TimePointT t1;
  auto ready0 = 0;
  auto ready1 = 0;
  // Start grabbing frames in the right order
  if (abs(std::chrono::duration_cast<std::chrono::milliseconds>(timestamps[0] - timestamps[1]).count())>=50)
  {
    thread1 = std::thread{ getFrameTime, fg1, std::ref(t1), std::ref(ready1) };
    thread0 = std::thread{ getFrameTime, fg0, std::ref(t0), std::ref(ready0)};
  }
  else {
    thread0 = std::thread{ getFrameTime, fg0, std::ref(t0), std::ref(ready0) };
    thread1 = std::thread{ getFrameTime, fg1, std::ref(t1), std::ref(ready1) };
  }
  using namespace std::chrono_literals;
  // Display delay
  while (true){
    std::this_thread::sleep_for(10ms);
    if (ready0 == 1 && ready1 == 1){
        std::lock_guard<std::mutex> guard0(mutex);
        ready0 = 0 ; ready1 = 0;
        fmt::print("Delay (ms): {}\n", abs(std::chrono::duration_cast<std::chrono::milliseconds>(t0 - t1).count()));
    }
  }

  thread0.join();
  thread1.join();

  return 0;
}