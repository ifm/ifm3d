// -*- c++ -*-
/*
 * Copyright (C) 2019 ifm electronics GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ifm3d/fg_udp/frame_grabber_udp.h>
#include <cstdint>
#include <functional>
#include <vector>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/camera/err.h>
#include <frame_grabber_udp_impl.hpp>

ifm3d::FrameGrabberUdp::FrameGrabberUdp(int port,
                                        std::uint16_t max_payload_size)
  : pImpl(new ifm3d::FrameGrabberUdp::Impl(port, max_payload_size))
{ }

ifm3d::FrameGrabberUdp::~FrameGrabberUdp() = default;

bool
ifm3d::FrameGrabberUdp::WaitForFrame(
  long timeout_millis,
  std::function<void(std::vector<std::uint8_t>&)> set_bytes)
{
  return this->pImpl->WaitForFrame(timeout_millis, set_bytes);
}
