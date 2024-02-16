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
  /** @ingroup FrameGrabber
   *
   * buffer_ids available for use with the default Organizer.
   */
  enum class buffer_id : uint64_t
  {
    // clang-format off
    
    /** @hideinitializer 
     * @brief \c
    */
    RADIAL_DISTANCE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::RADIAL_DISTANCE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    NORM_AMPLITUDE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::NORM_AMPLITUDE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    AMPLITUDE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::AMPLITUDE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    GRAYSCALE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::GRAYSCALE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    RADIAL_DISTANCE_NOISE = static_cast<uint64_t>(ifm3d::image_chunk::RADIAL_DISTANCE_NOISE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    REFLECTIVITY = static_cast<uint64_t>(ifm3d::image_chunk::REFLECTIVITY), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CARTESIAN_X_COMPONENT = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_X_COMPONENT), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CARTESIAN_Y_COMPONENT = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_Y_COMPONENT), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CARTESIAN_Z_COMPONENT = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_Z_COMPONENT), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CARTESIAN_ALL = static_cast<uint64_t>(ifm3d::image_chunk::CARTESIAN_ALL), 
    
    /** @hideinitializer 
     * @brief \c
    */
    UNIT_VECTOR_ALL = static_cast<uint64_t>(ifm3d::image_chunk::UNIT_VECTOR_ALL), 
    
    /** @hideinitializer 
     * @brief \c
    */
    MONOCHROM_2D_12BIT = static_cast<uint64_t>(ifm3d::image_chunk::MONOCHROM_2D_12BIT), 
    
    /** @hideinitializer 
     * @brief \c
    */
    MONOCHROM_2D = static_cast<uint64_t>(ifm3d::image_chunk::MONOCHROM_2D), 
    
    /** @hideinitializer 
     * @brief \c
    */
    JPEG_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::JPEG_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    CONFIDENCE_IMAGE = static_cast<uint64_t>(ifm3d::image_chunk::CONFIDENCE_IMAGE), 
    
    /** @hideinitializer 
     * @brief \c
    */
    DIAGNOSTIC = static_cast<uint64_t>(ifm3d::image_chunk::DIAGNOSTIC), 
    
    /** @hideinitializer 
     * @brief \c
    */
    JSON_DIAGNOSTIC = static_cast<uint64_t>(ifm3d::image_chunk::JSON_DIAGNOSTIC), 
    
    /** @hideinitializer 
     * @brief \c
    */
    EXTRINSIC_CALIB = static_cast<uint64_t>(ifm3d::image_chunk::EXTRINSIC_CALIB), 
    
    /** @hideinitializer 
     * @brief \c
    */
    INTRINSIC_CALIB = static_cast<uint64_t>(ifm3d::image_chunk::INTRINSIC_CALIB), 
    
    /** @hideinitializer 
     * @brief \c
    */
    INVERSE_INTRINSIC_CALIBRATION = static_cast<uint64_t>(ifm3d::image_chunk::INVERSE_INTRINSIC_CALIBRATION), 
    

    O3R_DISTANCE_IMAGE_INFO [[deprecated]] = static_cast<uint64_t>(ifm3d::image_chunk::TOF_INFO), 
    O3R_RGB_IMAGE_INFO [[deprecated]] = static_cast<uint64_t>(ifm3d::image_chunk::RGB_INFO), 
    
    /** @hideinitializer 
     * @brief ifm3d::TOFInfoV3 / ifm3d::TOFInfoV4
    */
    TOF_INFO = static_cast<uint64_t>(ifm3d::image_chunk::TOF_INFO), 
    
    /** @hideinitializer 
     * @brief ifm3d::RGBInfoV1
    */
    RGB_INFO = static_cast<uint64_t>(ifm3d::image_chunk::RGB_INFO), 
    
    /** @hideinitializer 
     * @brief \c
    */
    JSON_MODEL = static_cast<uint64_t>(ifm3d::image_chunk::JSON_MODEL), 
    
    /** @hideinitializer 
     * @brief \c
    */
    ALGO_DEBUG = static_cast<uint64_t>(ifm3d::image_chunk::ALGO_DEBUG), 
    
    /** @hideinitializer
     * @brief ifm3d::ODSOccupancyGridV1
    */
    O3R_ODS_OCCUPANCY_GRID = static_cast<uint64_t>(ifm3d::image_chunk::O3R_ODS_OCCUPANCY_GRID), 
    
    /** @hideinitializer
     * @brief ifm3d::ODSInfoV1
    */
    O3R_ODS_INFO = static_cast<uint64_t>(ifm3d::image_chunk::O3R_ODS_INFO), 
    
    /** @hideinitializer
     * @brief ifm3d::O3R_RESULT_JSON
    */
    O3R_RESULT_JSON = static_cast<uint64_t>(ifm3d::image_chunk::O3R_RESULT_JSON),

    /** @hideinitializer 
     * @brief ifm3d::O3R_RESULT_ARRAY2D
    */
    O3R_RESULT_ARRAY2D = static_cast<uint64_t>(ifm3d::image_chunk::O3R_RESULT_ARRAY2D),

    /** @hideinitializer 
     * @brief ifm3d::O3R_RESULT_IMU
    */
    O3R_RESULT_IMU = static_cast<uint64_t>(ifm3d::image_chunk::O3R_RESULT_IMU),


    /** @hideinitializer 
     * @brief The point cloud encoded as a 3 channel XYZ image
    */
    XYZ = std::numeric_limits<std::uint32_t>::max(),
    
    /** @hideinitializer 
     * @brief \c
     */
    EXPOSURE_TIME,
    
    /** @hideinitializer 
     * @brief \c
    */
    ILLUMINATION_TEMP,

    O3R_ODS_FLAGS,
    O3R_MCC_LIVE_IMAGE,
    O3R_MCC_MOTION_IMAGE,
    O3R_MCC_STATIC_IMAGE,

    // clang-format on
  };
  using TimePointT = std::chrono::time_point<std::chrono::system_clock,
                                             std::chrono::nanoseconds>;

  using BufferList = std::vector<Buffer>;
  using BufferDataListMap = std::map<ifm3d::buffer_id, BufferList>;

  /** @ingroup FrameGrabber
   *
   * Represent a frame of data received from the the device.
   */
  class IFM3D_FRAME_GRABBER_EXPORT Frame
  {
  public:
    using Ptr = std::shared_ptr<Frame>;

    Frame(const BufferDataListMap& images,
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
     * @param index of the image
     * @return Image& Reference to the requested buffer
     * @throw std::out_of_range if no image with the give id exists
     */
    Buffer& GetBuffer(buffer_id id,
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

    decltype(std::declval<std::map<buffer_id, BufferList>>().begin())
    begin() noexcept;
    decltype(std::declval<const std::map<buffer_id, BufferList>>().begin())
    begin() const noexcept;
    decltype(std::declval<std::map<buffer_id, BufferList>>().end())
    end() noexcept;
    decltype(std::declval<const std::map<buffer_id, BufferList>>().end()) end()
      const noexcept;

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
  }; // end: class Organizer

} // end: namespace ifm3d

#endif // IFM3D_FG_FRAME_H