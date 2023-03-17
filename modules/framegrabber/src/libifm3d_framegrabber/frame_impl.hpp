// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_FRAME_IMPL_H
#define IFM3D_FG_FRAME_IMPL_H

#include <map>
#include <ifm3d/fg/frame_grabber_export.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/frame.h>
#include <fmt/format.h>

namespace ifm3d
{
  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_FRAME_GRABBER_LOCAL Frame::Impl
  {
  public:
    Impl(const std::map<buffer_id, Buffer>& images,
         const std::vector<TimePointT> timestamps,
         uint64_t frame_count);

    std::vector<ifm3d::TimePointT> TimeStamps();

    bool HasBuffer(buffer_id id);

    Buffer& GetBuffer(buffer_id id);

    std::vector<ifm3d::buffer_id> GetBuffers();

    uint64_t FrameCount();

    decltype(std::declval<std::map<buffer_id, Buffer>>().begin())
    begin() noexcept;
    decltype(std::declval<const std::map<buffer_id, Buffer>>().begin()) begin()
      const noexcept;
    decltype(std::declval<std::map<buffer_id, Buffer>>().end()) end() noexcept;
    decltype(std::declval<const std::map<buffer_id, Buffer>>().end()) end()
      const noexcept;

  protected:
    //---------------------
    // State
    //---------------------
    std::map<buffer_id, Buffer> images_;
    std::vector<TimePointT> timestamps_;
    uint32_t frame_count_;

  }; // end: class FrameGrabber::Impl

} // end: namespace ifm3d

ifm3d::Frame::Impl::Impl(const std::map<buffer_id, Buffer>& images,
                         const std::vector<TimePointT> timestamps,
                         uint64_t frame_count)
  : images_(images),
    timestamps_(timestamps),
    frame_count_(frame_count)
{}

std::vector<ifm3d::TimePointT>
ifm3d::Frame::Impl::TimeStamps()
{
  return timestamps_;
}

bool
ifm3d::Frame::Impl::HasBuffer(buffer_id id)
{
  return images_.find(id) != images_.end();
}

ifm3d::Buffer&
ifm3d::Frame::Impl::GetBuffer(buffer_id id)
{
  if (HasBuffer(id))
    {
      return images_.at(id);
    }
  throw ifm3d::Error(
    IFM3D_BUFFER_ID_NOT_AVAILABLE,
    fmt::format("buffer_id: {}", std::to_string(static_cast<int>(id))));
}

std::vector<ifm3d::buffer_id>
ifm3d::Frame::Impl::GetBuffers()
{
  std::vector<buffer_id> keys;

  std::transform(images_.begin(),
                 images_.end(),
                 std::back_inserter(keys),
                 [](const auto& pair) { return pair.first; });

  return keys;
}

uint64_t
ifm3d::Frame::Impl::FrameCount()
{
  return frame_count_;
}

decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::Buffer>>().begin())
ifm3d::Frame::Impl::begin() noexcept
{
  return images_.begin();
}

decltype(
  std::declval<const std::map<ifm3d::buffer_id, ifm3d::Buffer>>().begin())
ifm3d::Frame::Impl::begin() const noexcept
{
  return images_.begin();
}

decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::Buffer>>().end())
ifm3d::Frame::Impl::end() noexcept
{
  return images_.end();
}

decltype(std::declval<const std::map<ifm3d::buffer_id, ifm3d::Buffer>>().end())
ifm3d::Frame::Impl::end() const noexcept
{
  return images_.end();
}

//============================================================
// Impl -- Implementation Details
//============================================================

#endif // IFM3D_FG_FRAME_IMPL_H
