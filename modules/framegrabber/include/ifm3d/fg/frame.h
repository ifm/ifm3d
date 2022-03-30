/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_FRAME_H
#define IFM3D_FG_FRAME_H

#include <chrono>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>
#include <ifm3d/camera/camera.h>
#include <ifm3d/fg/image.h>

namespace ifm3d
{
  using ImageId = std::uint64_t;
  using TimePointT = std::chrono::time_point<std::chrono::system_clock,
                                             std::chrono::nanoseconds>;

  class Frame
  {
  public:
    using Ptr = std::shared_ptr<Frame>;

    Frame(const std::map<ImageId, Image>& images,
          const std::vector<TimePointT> timestamps);
    ~Frame();

    Frame(const Frame& t);
    Frame& operator=(const Frame& t);

    Frame(Frame&& t);
    Frame& operator=(Frame&& t);

    std::vector<TimePointT> TimeStamps();

    bool HasImage(ImageId id);

    Image& GetImage(ImageId key);

    template <typename T>
    typename std::enable_if_t<std::is_enum_v<T>, Image&>
    GetImage(T key)
    {
      return this->GetImage(static_cast<ImageId>(key));
    }

    decltype(std::declval<std::map<ImageId, Image>>().begin())
    begin() noexcept;
    decltype(std::declval<const std::map<ImageId, Image>>().begin()) begin()
      const noexcept;
    decltype(std::declval<std::map<ImageId, Image>>().end()) end() noexcept;
    decltype(std::declval<const std::map<ImageId, Image>>().end()) end()
      const noexcept;

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
  }; // end: class Organizer

} // end: namespace ifm3d

#endif // IFM3D_FG_FRAME_H