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
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/frame_grabber_export.h>

namespace ifm3d
{
  /**
   * buffer_ids available for use with the default Organizer.
   */
  enum class buffer_id : uint64_t
  {
    // clang-format off

    RADIAL_DISTANCE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::RADIAL_DISTANCE_IMAGE),
    NORM_AMPLITUDE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::NORM_AMPLITUDE_IMAGE),
    AMPLITUDE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::AMPLITUDE_IMAGE),
    GRAYSCALE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::GRAYSCALE_IMAGE),
    RADIAL_DISTANCE_NOISE = static_cast<uint64_t>(ifm3d::image_chunk::RADIAL_DISTANCE_NOISE),
    REFLECTIVITY = static_cast<uint64_t>(ifm3d::image_chunk::REFLECTIVITY),
    CARTESIAN_X_COMPONENT = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_X_COMPONENT),
    CARTESIAN_Y_COMPONENT = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_Y_COMPONENT),
    CARTESIAN_Z_COMPONENT = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_Z_COMPONENT),
    CARTESIAN_ALL = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_ALL),
    UNIT_VECTOR_ALL = static_cast<uint64_t>(ifm3d::image_chunk::UNIT_VECTOR_ALL),
    MONOCHROM_2D_12BIT = static_cast<uint64_t>(ifm3d::image_chunk::MONOCHROM_2D_12BIT),
    MONOCHROM_2D = static_cast<uint64_t>(ifm3d::image_chunk::MONOCHROM_2D),
    JPEG_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::JPEG_IMAGE),
    CONFIDENCE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::CONFIDENCE_IMAGE),
    DIAGNOSTIC = static_cast<uint64_t>(ifm3d::image_chunk::DIAGNOSTIC),
    JSON_DIAGNOSTIC = static_cast<uint64_t>(ifm3d::image_chunk::JSON_DIAGNOSTIC),
    EXTRINSIC_CALIB = static_cast<uint64_t>(ifm3d::image_chunk::EXTRINSIC_CALIB),
    INTRINSIC_CALIB = static_cast<uint64_t>(ifm3d::image_chunk::INTRINSIC_CALIB),
    INVERSE_INTRINSIC_CALIBRATION = static_cast<uint64_t>(ifm3d::image_chunk::INVERSE_INTRINSIC_CALIBRATION),

    O3R_DISTANCE_IMAGE_INFO [[deprecated]] = static_cast<uint64_t>(ifm3d::image_chunk::TOF_INFO),
    O3R_RGB_IMAGE_INFO [[deprecated]] = static_cast<uint64_t>(ifm3d::image_chunk::RGB_INFO),
    TOF_INFO = static_cast<uint64_t>(ifm3d::image_chunk::TOF_INFO),
    RGB_INFO = static_cast<uint64_t>(ifm3d::image_chunk::RGB_INFO),
    JSON_MODEL = static_cast<uint64_t>(ifm3d::image_chunk::JSON_MODEL),
    ALGO_DEBUG = static_cast<uint64_t>(ifm3d::image_chunk::ALGO_DEBUG),
    O3R_ODS_OCCUPANCY_GRID = static_cast<uint64_t>(ifm3d::image_chunk::O3R_ODS_OCCUPANCY_GRID),
    O3R_ODS_INFO = static_cast<uint64_t>(ifm3d::image_chunk::O3R_ODS_INFO),
    XYZ = std::numeric_limits<std::uint32_t>::max(), // The point cloud encoded as a 3 channel XYZ image
    EXPOSURE_TIME,
    ILLUMINATION_TEMP,
    // clang-format on
  };
  using TimePointT = std::chrono::time_point<std::chrono::system_clock,
                                             std::chrono::nanoseconds>;

  /**
   * Represent a frame of data received from the the device.
   */
  class IFM3D_FRAME_GRABBER_EXPORT Frame
  {
  public:
    using Ptr = std::shared_ptr<Frame>;

    Frame(const std::map<buffer_id, Buffer>& images,
          const std::vector<TimePointT> timestamps,
          uint64_t frame_count);
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
    bool HasBuffer(buffer_id id);

    /**
     * @brief Get the image with the given id
     *
     * @param id the id of the image to get
     * @return Image& Reference to the requested buffer
     * @throw std::out_of_range if no image with the give id exists
     */
    Buffer& GetBuffer(buffer_id id);

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

    decltype(std::declval<std::map<buffer_id, Buffer>>().begin())
    begin() noexcept;
    decltype(std::declval<const std::map<buffer_id, Buffer>>().begin()) begin()
      const noexcept;
    decltype(std::declval<std::map<buffer_id, Buffer>>().end()) end() noexcept;
    decltype(std::declval<const std::map<buffer_id, Buffer>>().end()) end()
      const noexcept;

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
  }; // end: class Organizer

} // end: namespace ifm3d

#endif // IFM3D_FG_FRAME_H