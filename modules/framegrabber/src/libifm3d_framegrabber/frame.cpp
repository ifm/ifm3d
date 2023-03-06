/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/fg/frame.h>
#include <frame_impl.hpp>

ifm3d::Frame::Frame(const std::map<buffer_id, Buffer>& images,
                    const std::vector<TimePointT> timestamps,
                    uint64_t frame_count)
  : pImpl(std::make_unique<Impl>(images, timestamps, frame_count))
{}

ifm3d::Frame::~Frame() = default;

ifm3d::Frame::Frame(const Frame& t) : pImpl(std::make_unique<Impl>(*t.pImpl))
{}

ifm3d::Frame&
ifm3d::Frame::operator=(const Frame& t)
{
  *pImpl = *t.pImpl;
  return *this;
}

ifm3d::Frame::Frame(Frame&& t) = default;

ifm3d::Frame& ifm3d::Frame::operator=(Frame&& t) = default;

std::vector<ifm3d::TimePointT>
ifm3d::Frame::TimeStamps()
{
  return pImpl->TimeStamps();
}

uint32_t
ifm3d::Frame::FrameCount()
{
  return pImpl->FrameCount();
}

bool
ifm3d::Frame::HasBuffer(buffer_id id)
{
  return pImpl->HasBuffer(id);
}

ifm3d::Buffer&
ifm3d::Frame::GetBuffer(buffer_id key)
{
  return pImpl->GetBuffer(key);
}

std::vector<ifm3d::buffer_id>
ifm3d::Frame::GetBuffers()
{
  return pImpl->GetBuffers();
};

decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::Buffer>>().begin())
ifm3d::Frame::begin() noexcept
{
  return pImpl->begin();
}

decltype(
  std::declval<const std::map<ifm3d::buffer_id, ifm3d::Buffer>>().begin())
ifm3d::Frame::begin() const noexcept
{
  return pImpl->begin();
}

decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::Buffer>>().end())
ifm3d::Frame::end() noexcept
{
  return pImpl->end();
}

decltype(std::declval<const std::map<ifm3d::buffer_id, ifm3d::Buffer>>().end())
ifm3d::Frame::end() const noexcept
{
  return pImpl->end();
}