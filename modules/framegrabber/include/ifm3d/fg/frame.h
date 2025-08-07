/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_FRAME_H
#define IFM3D_FG_FRAME_H

#include <chrono>
#include <cstdint>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/fg/module_frame_grabber.h>
#include <memory>
#include <vector>

namespace ifm3d
{
  using TimePointT = std::chrono::time_point<std::chrono::system_clock,
                                             std::chrono::nanoseconds>;

  using BufferList = std::vector<Buffer>;
  using BufferDataListMap = std::map<ifm3d::buffer_id, BufferList>;

  /** @ingroup FrameGrabber
   *
   * Represent a frame of data received from the the device.
   */
  class IFM3D_EXPORT Frame
  {
  public:
    using Ptr = std::shared_ptr<Frame>;

    Frame(const BufferDataListMap& images,
          const std::vector<TimePointT>& timestamps,
          uint64_t frame_count);
    ~Frame();

    Frame(const Frame& t);
    Frame& operator=(const Frame& t);

    Frame(Frame&& t) noexcept;
    Frame& operator=(Frame&& t) noexcept;

    /**
     * @brief Get the timestamps of the frame
     *
     * @return the timestamps
     */
    std::vector<TimePointT> TimeStamps();

    /**
     * @brief Check if a image with the given id is available in this frame
     *
     * @param id the id of the image
     * @return true if a image with the give id is available
     * @return false if no image with the given id is availale
     */
    bool HasBuffer(buffer_id id);

    /**
     * @brief Get the image with the given id
     *
     * @param id the id of the image to get
     * @param index of the image
     * @return Image& Reference to the requested buffer
     * @throw std::out_of_range if no image with the give id exists
     */
    Buffer& GetBuffer(buffer_id key,
                      std::optional<size_t> index = std::nullopt);

    /**
     * @brief Get the total number of image with the given id
     */
    size_t GetBufferCount(buffer_id id);

    /**
     * @brief Get the frame count according to algorithm output
     */
    uint32_t FrameCount();

    /**
     * @brief Get the list of available buffers
     *
     * @param id the id of the image to get
     * @return the list of available buffer_ids
     */
    std::vector<buffer_id> GetBuffers();

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

  private:
    class Impl;
    std::unique_ptr<Impl> _impl;
  }; // end: class Organizer

} // end: namespace ifm3d

#endif // IFM3D_FG_FRAME_H