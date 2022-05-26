/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/fg/frame.h>
#include <frame_impl.hpp>

ifm3d::Frame::Frame(const std::map<buffer_id, Image>& images,
                    const std::vector<TimePointT> timestamps)
  : pImpl(std::make_unique<Impl>(images, timestamps))
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

bool
ifm3d::Frame::HasImage(buffer_id id)
{
  return pImpl->HasImage(id);
}

ifm3d::Image&
ifm3d::Frame::GetImage(buffer_id key)
{
  return pImpl->GetImage(key);
}

decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::Image>>().begin())
ifm3d::Frame::begin() noexcept
{
  return pImpl->begin();
}

decltype(std::declval<const std::map<ifm3d::buffer_id, ifm3d::Image>>().begin())
ifm3d::Frame::begin() const noexcept
{
  return pImpl->begin();
}

decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::Image>>().end())
ifm3d::Frame::end() noexcept
{
  return pImpl->end();
}

decltype(std::declval<const std::map<ifm3d::buffer_id, ifm3d::Image>>().end())
ifm3d::Frame::end() const noexcept
{
  return pImpl->end();
}