// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_FRAME_IMPL_H
#define IFM3D_FG_FRAME_IMPL_H

#include <fmt/format.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/fg/module_frame_grabber.h>
#include <map>
#include <utility>

namespace ifm3d
{
  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_NO_EXPORT Frame::Impl
  {
  public:
    Impl(BufferDataListMap images,
         std::vector<TimePointT> timestamps,
         uint64_t frame_count);

    std::vector<ifm3d::TimePointT> TimeStamps();

    bool HasBuffer(buffer_id id);

    Buffer& GetBuffer(buffer_id id, std::optional<size_t> index);
    size_t GetBufferCount(buffer_id id);

    std::vector<ifm3d::buffer_id> GetBuffers();

    [[nodiscard]] uint64_t FrameCount() const;

    // NOLINTBEGIN(readability-identifier-naming)
    decltype(std::declval<std::map<buffer_id, BufferList>>().begin())
    begin() noexcept;
    [[nodiscard]] decltype(std::declval<
                             const std::map<buffer_id, BufferList>>()
                             .begin())
    begin() const noexcept;
    decltype(std::declval<std::map<buffer_id, BufferList>>().end())
    end() noexcept;
    [[nodiscard]] decltype(std::declval<
                             const std::map<buffer_id, BufferList>>()
                             .end())
    end() const noexcept;
    // NOLINTEND(readability-identifier-naming)
  protected:
    //---------------------
    // State
    //---------------------
    BufferDataListMap _images;
    std::vector<TimePointT> _timestamps;
    uint32_t _frame_count;

  }; // end: class FrameGrabber::Impl

} // end: namespace ifm3d

inline ifm3d::Frame::Impl::Impl(BufferDataListMap images,
                                std::vector<TimePointT> timestamps,
                                uint64_t frame_count)
  : _images(std::move(images)),
    _timestamps(std::move(timestamps)),
    _frame_count(frame_count)
{}

inline std::vector<ifm3d::TimePointT>
ifm3d::Frame::Impl::TimeStamps()
{
  return _timestamps;
}

inline bool
ifm3d::Frame::Impl::HasBuffer(buffer_id id)
{
  return _images.find(id) != _images.end();
}

inline ifm3d::Buffer&
ifm3d::Frame::Impl::GetBuffer(buffer_id id, std::optional<size_t> index)
{
  if (HasBuffer(id))
    {
      auto buffer_list = _images.at(id);
      auto index_value = index.value_or(0);
      if (index_value < buffer_list.size())
        {
          return _images.at(id)[index_value];
        }

      throw ifm3d::Error(IFM3D_INDEX_OUT_OF_RANGE,
                         fmt::format("buffer_id: {}, index = {}",
                                     std::to_string(static_cast<int>(id)),
                                     index_value));
    }
  throw ifm3d::Error(
    IFM3D_BUFFER_ID_NOT_AVAILABLE,
    fmt::format("buffer_id: {}", std::to_string(static_cast<int>(id))));
}

inline size_t
ifm3d::Frame::Impl::GetBufferCount(ifm3d::buffer_id id)
{
  return _images.at(id).size();
}

inline std::vector<ifm3d::buffer_id>
ifm3d::Frame::Impl::GetBuffers()
{
  std::vector<buffer_id> keys;

  std::transform(_images.begin(),
                 _images.end(),
                 std::back_inserter(keys),
                 [](const auto& pair) { return pair.first; });

  return keys;
}

inline uint64_t
ifm3d::Frame::Impl::FrameCount() const
{
  return _frame_count;
}

inline decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
                  .begin())
ifm3d::Frame::Impl::begin() noexcept
{
  return _images.begin();
}

inline decltype(std::declval<
                  const std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
                  .begin())
ifm3d::Frame::Impl::begin() const noexcept
{
  return _images.begin();
}

inline decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
                  .end())
ifm3d::Frame::Impl::end() noexcept
{
  return _images.end();
}

inline decltype(std::declval<
                  const std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
                  .end())
ifm3d::Frame::Impl::end() const noexcept
{
  return _images.end();
}

//============================================================
// Impl -- Implementation Details
//============================================================

#endif // IFM3D_FG_FRAME_IMPL_H
