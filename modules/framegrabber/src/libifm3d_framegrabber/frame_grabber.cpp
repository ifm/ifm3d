/*
 * Copyright (C) 2017 Love Park Robotics, LLC
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

#include <ifm3d/fg/frame_grabber.h>
#include <cstdint>
#include <functional>
#include <vector>
#include <ifm3d/camera/camera.h>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/camera/err.h>
#include <frame_grabber_impl.hpp>

ifm3d::FrameGrabber::FrameGrabber(ifm3d::Camera::Ptr cam,
                                  std::uint16_t mask)
  : pImpl(new ifm3d::FrameGrabber::Impl(cam, mask))
{ }

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
