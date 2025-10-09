/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "frame_impl.hpp"
#include <cstddef>
#include <cstdint>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/fg/frame.h>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

ifm3d::Frame::Frame(const BufferDataListMap& images,
                    const std::vector<TimePointT>& timestamps,
                    uint64_t frame_count)
  : _impl(std::make_unique<Impl>(images, timestamps, frame_count))
{}

ifm3d::Frame::~Frame() = default;

ifm3d::Frame::Frame(const Frame& t) : _impl(std::make_unique<Impl>(*t._impl))
{}

ifm3d::Frame&
ifm3d::Frame::operator=(const Frame& t)
{
  if (this == &t)
    {
      return *this;
    }

  *_impl = *t._impl;
  return *this;
}

ifm3d::Frame::Frame(Frame&& t) noexcept = default;

ifm3d::Frame& ifm3d::Frame::operator=(Frame&& t) noexcept = default;

std::vector<ifm3d::TimePointT>
ifm3d::Frame::TimeStamps()
{
  return _impl->TimeStamps();
}

uint32_t
ifm3d::Frame::FrameCount()
{
  return _impl->FrameCount();
}

bool
ifm3d::Frame::HasBuffer(buffer_id id)
{
  return _impl->HasBuffer(id);
}

ifm3d::Buffer&
ifm3d::Frame::GetBuffer(buffer_id key, std::optional<size_t> index)
{
  return _impl->GetBuffer(key, index);
}

size_t
ifm3d::Frame::GetBufferCount(buffer_id id)
{
  return _impl->GetBufferCount(id);
}

std::vector<ifm3d::buffer_id>
ifm3d::Frame::GetBuffers()
{
  return _impl->GetBuffers();
};

decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::BufferList>>().begin())
ifm3d::Frame::begin() noexcept
{
  return _impl->begin();
}

decltype(std::declval<const std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
           .begin())
ifm3d::Frame::begin() const noexcept
{
  return _impl->begin();
}

decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::BufferList>>().end())
ifm3d::Frame::end() noexcept
{
  return _impl->end();
}

decltype(std::declval<const std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
           .end())
ifm3d::Frame::end() const noexcept
{
  return _impl->end();
}