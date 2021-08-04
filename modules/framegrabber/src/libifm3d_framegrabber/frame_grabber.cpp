/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/fg/frame_grabber.h>
#include <cstdint>
#include <functional>
#include <vector>
#include <ifm3d/camera/camera.h>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/camera/err.h>
#include <frame_grabber_impl.hpp>

ifm3d::FrameGrabber::FrameGrabber(ifm3d::CameraBase::Ptr cam,
                                  std::uint16_t mask,
                                  const std::uint16_t nat_pcic_port)
  : pImpl(new ifm3d::FrameGrabber::Impl(cam, mask, nat_pcic_port))
{}

ifm3d::FrameGrabber::~FrameGrabber() = default;

void
ifm3d::FrameGrabber::SWTrigger()
{
  this->pImpl->SWTrigger();
}

bool
ifm3d::FrameGrabber::WaitForFrame(
  long timeout_millis,
  std::function<void(std::vector<std::uint8_t>&)> set_bytes)
{
  return this->pImpl->WaitForFrame(timeout_millis, set_bytes);
}
