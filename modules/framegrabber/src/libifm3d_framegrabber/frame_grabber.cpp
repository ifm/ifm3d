/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/fg/frame_grabber.h>
#include <cstdint>
#include <functional>
#include <vector>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/err.h>
#include <frame_grabber_impl.hpp>

ifm3d::FrameGrabber::FrameGrabber(ifm3d::CameraBase::Ptr cam,
                                  std::optional<std::uint16_t> pcic_port)
  : pImpl(new ifm3d::FrameGrabber::Impl(cam, pcic_port))
{}

ifm3d::FrameGrabber::~FrameGrabber() = default;

void
ifm3d::FrameGrabber::SWTrigger()
{
  this->pImpl->SWTrigger();
}

void
ifm3d::FrameGrabber::OnNewFrame(NewFrameCallback callback)
{
  this->pImpl->OnNewFrame(callback);
}

bool
ifm3d::FrameGrabber::Start(const std::set<ImageId>& images)
{
  return this->pImpl->Start(images);
}

bool
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
