/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <future>
#include "ifm3d/common/json_impl.hpp"
#include "ifm3d/fg/buffer_id.h"
#include "ifm3d/fg/frame.h"
#include "ifm3d/fg/organizer.h"
#include <ifm3d/fg/frame_grabber.h>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>
#include <set>
#include <memory>
#include <vector>
#include <ifm3d/device/device.h>
#include <frame_grabber_impl.hpp>

ifm3d::FrameGrabber::FrameGrabber(ifm3d::Device::Ptr cam,
                                  std::optional<std::uint16_t> pcic_port)
  : pImpl(new ifm3d::FrameGrabber::Impl(std::move(cam), pcic_port))
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
  this->pImpl->OnNewFrame(std::move(callback));
}

template <class>
struct always_false : std::false_type
{
};

std::shared_future<void>
ifm3d::FrameGrabber::Start(const BufferList& buffers,
                           const std::optional<json>& schema)
{
  std::vector<buffer_id> buffer_ids;
  buffer_ids.reserve(buffers.size());

  for (const auto& id_variant : buffers)
    {
      std::visit(
        [&](const auto& id) {
          using T = std::decay_t<decltype(id)>;
          if constexpr (std::is_same_v<T, std::uint64_t> ||
                        std::is_same_v<T, int>)
            {
              buffer_ids.emplace_back(static_cast<ifm3d::buffer_id>(id));
            }
          else if constexpr (std::is_same_v<T, ifm3d::buffer_id>)
            {
              buffer_ids.emplace_back(id);
            }
          else
            {
              static_assert(always_false<T>::value, "Unsupported type");
            }
        },
        id_variant);
    }

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
  this->pImpl->OnAsyncError(std::move(callback));
}

void
ifm3d::FrameGrabber::OnAsyncNotification(AsyncNotificationCallback callback)
{
  this->pImpl->OnAsyncNotification(std::move(callback));
}

void
ifm3d::FrameGrabber::OnError(ErrorCallback callback)
{
  this->pImpl->OnError(std::move(callback));
}

void
ifm3d::FrameGrabber::SetMasking(const bool mask)
{
  this->pImpl->SetMasking(mask);
}

bool
ifm3d::FrameGrabber::IsMasking()
{
  return this->pImpl->IsMasking();
}