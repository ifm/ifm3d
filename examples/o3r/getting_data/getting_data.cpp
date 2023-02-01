/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <iostream>
#include <chrono>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg.h>

using namespace std::chrono_literals;
using namespace ifm3d::literals;

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

  //////////////////////////
  // Get a frame:
  //////////////////////////
  auto future = fg->WaitForFrame();
  if (future.wait_for(3s) != std::future_status::ready)
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;
    }
  auto frame = future.get();

  //////////////////////////
  // Example for 3D data:
  //////////////////////////
  auto dist = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);

  std::cout << dist.height() << " " << dist.width() << std::endl;

  fg->Stop();
  return 0;
}