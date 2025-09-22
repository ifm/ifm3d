/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "frame_grabber_impl.hpp"
#include <cstdint>
#include <future>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/device/pcic_command.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/fg/frame_grabber.h>
#include <ifm3d/fg/organizer.h>
#include <memory>
#include <optional>
#include <set>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

ifm3d::FrameGrabber::FrameGrabber(ifm3d::Device::Ptr cam,
                                  std::optional<std::uint16_t> pcic_port)
  : _impl(new ifm3d::FrameGrabber::Impl(std::move(cam), pcic_port))
{}

ifm3d::FrameGrabber::~FrameGrabber() = default;

std::shared_future<void>
ifm3d::FrameGrabber::SWTrigger()
{
  return this->_impl->SWTrigger();
}

void
ifm3d::FrameGrabber::OnNewFrame(NewFrameCallback callback)
{
  this->_impl->OnNewFrame(std::move(callback));
}

template <class>
struct AlwaysFalse : std::false_type
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
              static_assert(AlwaysFalse<T>::value, "Unsupported type");
            }
        },
        id_variant);
    }

  return this->_impl->Start(
    std::set<ifm3d::buffer_id>(buffer_ids.begin(), buffer_ids.end()),
    schema);
}

std::shared_future<void>
ifm3d::FrameGrabber::Stop()
{
  return this->_impl->Stop();
}

bool
ifm3d::FrameGrabber::IsRunning()
{
  return this->_impl->IsRunning();
}

std::shared_future<ifm3d::Frame::Ptr>
ifm3d::FrameGrabber::WaitForFrame()
{
  return this->_impl->WaitForFrame();
}

void
ifm3d::FrameGrabber::SetOrganizer(std::unique_ptr<Organizer> organizer)
{
  this->_impl->SetOrganizer(std::move(organizer));
}

void
ifm3d::FrameGrabber::OnAsyncError(AsyncErrorCallback callback)
{
  this->_impl->OnAsyncError(std::move(callback));
}

void
ifm3d::FrameGrabber::OnAsyncNotification(AsyncNotificationCallback callback)
{
  this->_impl->OnAsyncNotification(std::move(callback));
}

void
ifm3d::FrameGrabber::OnError(ErrorCallback callback)
{
  this->_impl->OnError(std::move(callback));
}

void
ifm3d::FrameGrabber::SetMasking(const bool mask)
{
  this->_impl->SetMasking(mask);
}

bool
ifm3d::FrameGrabber::IsMasking()
{
  return this->_impl->IsMasking();
}

std::shared_future<ifm3d::FrameGrabber::PCICCommandResponse>
ifm3d::FrameGrabber::SendCommand(const PCICCommand& command)
{
  return this->_impl->SendCommand(command);
}
