/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_DISTANCE_IMAGE_INFO_H
#define IFM3D_DISTANCE_IMAGE_INFO_H

#include <cstddef>
#include <cstdint>
#include <ifm3d/fg/buffer.h>
#include <memory>
#include <vector>

#include <ifm3d/fg/module_frame_grabber.h>

namespace ifm3d
{
  constexpr auto CHUNK_SIZE_INFO_OFFSET = 4;
  constexpr auto HEADER_SIZE_INFO_OFFSET = 8;
  constexpr auto HEADER_VERSION_INFO_OFFSET = 12;
  constexpr auto DISTANCE_IMAGE_INFO_DATA_SIZE = 360;

  constexpr auto NR_MODEL_PARAMS = 32;
  constexpr auto AMPL_NORM_FACTOR_VECTOR_SIZE = 3;
  constexpr auto EXTR_OPTIC_USER_VECTOR_SIZE = 6;

  // NOLINTNEXTLINE(performance-enum-size)
  enum class ExtrinsicParam : std::uint32_t
  {
    TRANS_X = 0, // Translation along x-direction in meters.
    TRANS_Y = 1, // Translation along y-direction in meters.
    TRANS_Z = 2, // Translation along Z-direction in meters.
    ROT_X = 3,   // Rotation along x-axis in radians. Positive values indicate
                 // clockwise rotation.
    ROT_Y = 4,   // Rotation along y-axis in radians. Positive values indicate
                 // clockwise rotation.
    ROT_Z = 5    // Rotation along z-axis in radians. Positive values indicate
                 // clockwise rotation.
  };

  struct IntrinsicCalibration
  {
    uint32_t model_id;
    std::array<float, NR_MODEL_PARAMS> model_parameters;
  };

  class IFM3D_EXPORT DistanceImageInfo
  {
    float _dist_resolution;
    float _ampl_resolution;
    std::vector<float> _amp_norm_factors;
    std::vector<float> _extrinsic_optic_to_user;
    IntrinsicCalibration _intrinsic_calibration;
    IntrinsicCalibration _inverse_intrinsic_calibration;
    std::vector<std::uint16_t> _u16_distance_buffer;
    std::vector<std::uint16_t> _u16_amplitude_buffer;
    std::uint32_t _width, _height;
    std::vector<uint64_t> _timestamps_nsec;
    std::vector<float> _exposure_times_sec;

  public:
    DistanceImageInfo(const DistanceImageInfo&) = default;
    DistanceImageInfo(DistanceImageInfo&&) = delete;
    DistanceImageInfo& operator=(const DistanceImageInfo&) = delete;
    DistanceImageInfo& operator=(DistanceImageInfo&&) = delete;
    DistanceImageInfo(float dist_res,
                      float ampl_res,
                      const std::vector<float>& amp_norm_fctrs,
                      const std::vector<float>& extr_opt_to_usr,
                      const IntrinsicCalibration& intr_calib,
                      const IntrinsicCalibration& inv_intr_calib,
                      const std::vector<std::uint16_t>& distance_buffer,
                      const std::vector<std::uint16_t>& amplitude_buffer,
                      const std::vector<uint64_t>& timestamps_nsec,
                      const std::vector<float>& exposure_times_sec,
                      std::uint32_t width,
                      std::uint32_t height);
    ~DistanceImageInfo() = default;
    std::vector<std::uint8_t> GetXyzdVector();
    std::vector<std::uint8_t> GetAmplitudeVector();

    auto
    GetExtrinsicOpticToUser()
    {
      return _extrinsic_optic_to_user;
    }

    auto
    GetIntrinsicCalibration()
    {
      return _intrinsic_calibration;
    }

    auto
    GetInverseIntrinsicCalibration()
    {
      return _inverse_intrinsic_calibration;
    }

    [[nodiscard]] auto
    GetNpts() const
    {
      return (_width * _height);
    }

    [[nodiscard]] auto
    GetWidth() const
    {
      return _width;
    }

    [[nodiscard]] auto
    GetHeight() const
    {
      return _height;
    }

    /**
     * @brief returns the timestamps in nano seconds
     */
    std::vector<uint64_t>
    GetTimestamps()
    {
      return _timestamps_nsec;
    }

    /**
     * @brief return the exposure time for each
     * phase data
     */
    std::vector<float>
    GetExposureTimes()
    {
      return _exposure_times_sec;
    }

    /**
     * @brief multiply distance noise image with distance resolution
     * return Buffer with float values
     */
    [[nodiscard]] ifm3d::Buffer ApplyDistanceResolution(
      const ifm3d::Buffer& ui16_distance_buffer) const;
  };
  using DistanceImageInfoPtr = std::unique_ptr<DistanceImageInfo>;
  IFM3D_EXPORT DistanceImageInfoPtr
  create_distance_image_info(const std::vector<std::uint8_t>& data_buffer,
                             std::size_t distimageinfo_idx,
                             std::size_t dist_idx,
                             std::size_t amp_idx,
                             std::uint32_t width,
                             std::uint32_t height);
} // end: namespace ifm3d

#endif // IFM3D_DISTANCE_IMAGE_INFO_H
