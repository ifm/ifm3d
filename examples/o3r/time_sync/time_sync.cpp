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
#include <unistd.h>
#include <cstdlib>
#include <iomanip>
#include <thread>
#include <mutex>
#include <fmt/core.h>

#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/stlimage.h>
#include <ifm3d/fg/distance_image_info.h>

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

int getFrameTime(ifm3d::FrameGrabber::Ptr fg, ifm3d::StlImageBuffer::Ptr im, ifm3d::TimePointT &t, int &ready){
  /**
   * Grab frame from a camera head and update global variable 
   * with the timestamp of the frame.
   * When the frame has been captured, switches the global variable ready
   * to release the display thread.
   */
  int i = 0;
  while (true) {
    if (! fg->WaitForFrame(im.get(), 1000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }
    else {
      i+=1;
      if (i == 10){
        std::lock_guard<std::mutex> guard(mutex);
        t = im->TimeStamp();
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
  auto o3r = std::make_shared<ifm3d::O3RCamera>();
  json conf = o3r->Get();
  // Declare the FrameGrabber and ImageBuffer objects. 
  // One FrameGrabber per camera head (define the port number).
  auto im0 =  std::make_shared<ifm3d::StlImageBuffer>(); 
  auto im1 =  std::make_shared<ifm3d::StlImageBuffer>();   
  auto fg0 = std::make_shared<ifm3d::FrameGrabber>(o3r, ifm3d::DEFAULT_SCHEMA_MASK, 50012);
  auto fg1 = std::make_shared<ifm3d::FrameGrabber>(o3r, ifm3d::DEFAULT_SCHEMA_MASK, 50013);

  std::thread thread0;
  std::thread thread1;
  //////////////////////////
  // Set framerate:
  //////////////////////////
  o3r->Set(json::parse(R"({"ports":{"port2":{"acquisition": {"framerate": 10}}, 
                                    "port3":{"acquisition": {"framerate": 10}}}})"));


  //////////////////////////
  // Start the cameras at
  // the same time
  /////////////////////////
  o3r->Set(json::parse(R"({"ports":{"port2":{"state": "CONF"}, 
                                    "port3":{"state": "CONF"}}})"));
  sleep(1);
  o3r->Set(json::parse(R"({"ports":{"port2":{"state": "RUN"}, 
                                    "port3":{"state": "RUN"}}})"));
  sleep(0.5);

  // Check which frame comes first:
  std::vector<ifm3d::TimePointT> timestamps;
  std::vector<ifm3d::FrameGrabber::Ptr> fgs;
  fgs.push_back(fg0); fgs.push_back(fg1);
  for (auto fg: fgs) {
    if (! fg->WaitForFrame(im0.get(), 1000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }
    else {
      timestamps.push_back(im0->TimeStamp());
    }
  }
  ifm3d::TimePointT t0;
  ifm3d::TimePointT t1;
  auto ready0 = 0;
  auto ready1 = 0;
  // Start grabbing frames in the right order
  if (abs(std::chrono::duration_cast<std::chrono::milliseconds>(timestamps[0] - timestamps[1]).count())>=50)
  {
    thread1 = std::thread{ getFrameTime, fg1, im1, std::ref(t1), std::ref(ready1) };
    thread0 = std::thread{ getFrameTime, fg0, im0, std::ref(t0), std::ref(ready0)};
  }
  else {
    thread0 = std::thread{ getFrameTime, fg0, im0, std::ref(t0), std::ref(ready0) };
    thread1 = std::thread{ getFrameTime, fg1, im1, std::ref(t1), std::ref(ready1) };
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