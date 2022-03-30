/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <iostream>
#include <chrono>
#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/fg/distance_image_info.h>

using namespace std::chrono_literals;

int
main()
{

  //////////////////////////
  // Declare the objects:
  //////////////////////////
  // Declare the device object (one object only, corresponding to the VPU)
  auto cam = std::make_shared<ifm3d::O3RCamera>();
  // Declare the FrameGrabber and ImageBuffer objects.
  // One FrameGrabber per camera head (define the port number).
  const auto FG_PCIC_PORT =
    cam->Get()["/ports/port2/data/pcicTCPPort"_json_pointer];
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, FG_PCIC_PORT);

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
  auto dist = frame->GetImage(ifm3d::image_chunk::RADIAL_DISTANCE);

  std::cout << dist.height() << " " << dist.width() << std::endl;

  return 0;
}