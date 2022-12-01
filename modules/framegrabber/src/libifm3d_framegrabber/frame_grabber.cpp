/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/fg/frame_grabber.h>
#include <cstdint>
#include <functional>
#include <vector>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <frame_grabber_impl.hpp>

ifm3d::FrameGrabber::FrameGrabber(ifm3d::Device::Ptr cam,
                                  std::optional<std::uint16_t> pcic_port)
  : pImpl(new ifm3d::FrameGrabber::Impl(cam, pcic_port))
{}

ifm3d::FrameGrabber::~FrameGrabber() = default;

std::shared_future<void>
ifm3d::FrameGrabber::SWTrigger()
{
  return this->pImpl->SWTrigger();
}

void
ifm3d::FrameGrabber::OnNewFrame(NewFrameCallback callback)
{
  this->pImpl->OnNewFrame(callback);
}

bool
ifm3d::FrameGrabber::Start(
  const std::vector<std::variant<std::uint64_t, int, ifm3d::buffer_id>>&
    buffers,
  const std::optional<json>& schema)
{
  std::vector<buffer_id> buffer_ids;
  std::transform(
    buffers.begin(),
    buffers.end(),
    std::back_inserter(buffer_ids),
    [](const std::variant<std::uint64_t, int, ifm3d::buffer_id>& id) {
      if (std::holds_alternative<std::uint64_t>(id))
        return static_cast<ifm3d::buffer_id>(std::get<std::uint64_t>(id));
      if (std::holds_alternative<int>(id))
        return static_cast<ifm3d::buffer_id>(std::get<int>(id));
      if (std::holds_alternative<ifm3d::buffer_id>(id))
        return std::get<ifm3d::buffer_id>(id);
      return static_cast<ifm3d::buffer_id>(0);
    });

  return this->pImpl->Start(
    std::set<ifm3d::buffer_id>(buffer_ids.begin(), buffer_ids.end()),
    schema);
}

std::shared_future<void>
ifm3d::FrameGrabber::Stop()
{
  return this->pImpl->Stop();
}

bool
ifm3d::FrameGrabber::IsRunning()
{
  return this->pImpl->IsRunning();
}

std::shared_future<ifm3d::Frame::Ptr>
ifm3d::FrameGrabber::WaitForFrame()
{
  return this->pImpl->WaitForFrame();
}

void
ifm3d::FrameGrabber::SetOrganizer(std::unique_ptr<Organizer> organizer)
{
  this->pImpl->SetOrganizer(std::move(organizer));
}

void
ifm3d::FrameGrabber::OnAsyncError(AsyncErrorCallback callback)
{
  this->pImpl->OnAsyncError(callback);
}

void
ifm3d::FrameGrabber::OnAsyncNotification(AsyncNotificationCallback callback)
{
  this->pImpl->OnAsyncNotification(callback);
}

void
ifm3d::FrameGrabber::OnError(ErrorCallback callback)
{
  this->pImpl->OnError(callback);
}
