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
  /**
   * image_ids available for use with the default Organizer.
   */
  enum class image_id : uint64_t
  {
    // clang-format off

    RADIAL_DISTANCE = static_cast<uint64_t>(ifm3d::image_chunk::RADIAL_DISTANCE),
    AMPLITUDE = static_cast<uint64_t>(ifm3d::image_chunk::AMPLITUDE),
    RAW_AMPLITUDE = static_cast<uint64_t>(ifm3d::image_chunk::RAW_AMPLITUDE),
    GRAY = static_cast<uint64_t>(ifm3d::image_chunk::GRAY),
    DISTANCE_NOISE = static_cast<uint64_t>(ifm3d::image_chunk::DISTANCE_NOISE),
    CARTESIAN_X = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_X),
    CARTESIAN_Y = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_Y),
    CARTESIAN_Z = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_Z),
    CARTESIAN_ALL = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_ALL),
    UNIT_VECTOR_ALL = static_cast<uint64_t>(ifm3d::image_chunk::UNIT_VECTOR_ALL),
    JPEG = static_cast<uint64_t>(ifm3d::image_chunk::JPEG),
    CONFIDENCE = static_cast<uint64_t>(ifm3d::image_chunk::CONFIDENCE),
    DIAGNOSTIC_DATA = static_cast<uint64_t>(ifm3d::image_chunk::DIAGNOSTIC_DATA),
    EXTRINSIC_CALIBRATION = static_cast<uint64_t>(ifm3d::image_chunk::EXTRINSIC_CALIBRATION),
    INTRINSIC_CALIBRATION = static_cast<uint64_t>(ifm3d::image_chunk::INTRINSIC_CALIBRATION),
    INVERSE_INTRINSIC_CALIBRATION = static_cast<uint64_t>(ifm3d::image_chunk::INVERSE_INTRINSIC_CALIBRATION),
    O3R_DISTANCE_IMAGE_INFORMATION = static_cast<uint64_t>(ifm3d::image_chunk::O3R_DISTANCE_IMAGE_INFORMATION),
    JSON_MODEL = static_cast<uint64_t>(ifm3d::image_chunk::JSON_MODEL),
    ALGO_DEBUG = static_cast<uint64_t>(ifm3d::image_chunk::ALGO_DEBUG),
    EXPOSURE_TIME,
    ILLUMINATION_TEMP,
    XYZ = std::numeric_limits<std::uint32_t>::max(), // The point cloud encoded as a 3 channel XYZ image

    // clang-format on
  };
  using TimePointT = std::chrono::time_point<std::chrono::system_clock,
                                             std::chrono::nanoseconds>;

  /**
   * Represent a frame of data received from the the device.
   */
  class Frame
  {
  public:
    using Ptr = std::shared_ptr<Frame>;

    Frame(const std::map<image_id, Image>& images,
          const std::vector<TimePointT> timestamps);
    ~Frame();

    Frame(const Frame& t);
    Frame& operator=(const Frame& t);

    Frame(Frame&& t);
    Frame& operator=(Frame&& t);

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
    bool HasImage(image_id id);

    /**
     * @brief Get the image with the given id
     *
     * @param id the id of the image to get
     * @return Image& Reference to the requrest image
     * @throw std::out_of_range if no image with the give id exists
     */
    Image& GetImage(image_id id);

    decltype(std::declval<std::map<image_id, Image>>().begin())
    begin() noexcept;
    decltype(std::declval<const std::map<image_id, Image>>().begin()) begin()
      const noexcept;
    decltype(std::declval<std::map<image_id, Image>>().end()) end() noexcept;
    decltype(std::declval<const std::map<image_id, Image>>().end()) end()
      const noexcept;

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
  }; // end: class Organizer

} // end: namespace ifm3d

#endif // IFM3D_FG_FRAME_H