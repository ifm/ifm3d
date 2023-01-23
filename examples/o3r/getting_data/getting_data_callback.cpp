/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <iostream>
#include <chrono>
#include <thread>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg.h>

using namespace std::chrono_literals;
using namespace ifm3d::literals;

void Callback(ifm3d::Frame::Ptr frame){
  auto dist = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
  std::cout << dist.height() << " " << dist.width() << std::endl;
}

int
main()
{

  //////////////////////////
  // Declare the objects:
  //////////////////////////
  // Declare the device object (one object only, corresponding to the VPU)
  auto cam = std::make_shared<ifm3d::O3R>();
  // Declare the FrameGrabber and ImageBuffer objects.
  // One FrameGrabber per camera head (define the port number).
  const auto FG_PCIC_PORT =
    cam->Get()["/ports/port2/data/pcicTCPPort"_json_pointer];
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, FG_PCIC_PORT);

  //Set Schema and start the grabber
  fg->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE, ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,ifm3d::buffer_id::XYZ});

  //Register callback function
  fg->OnNewFrame(&Callback);

  // This sleep is to prevent the program from before the
  // callback has time to execute.
  std::this_thread::sleep_for(1s);
  fg->Stop();

  return 0;
}