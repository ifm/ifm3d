/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_DISTANCE_IMAGE_INFO_H
#define IFM3D_DISTANCE_IMAGE_INFO_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <ifm3d/fg/buffer.h>
#include <memory>

#include <ifm3d/fg/frame_grabber_export.h>

namespace ifm3d
{
  constexpr auto CHUNK_SIZE_INFO_OFFSET = 4;
  constexpr auto HEADER_SIZE_INFO_OFFSET = 8;
  constexpr auto HEADER_VERSION_INFO_OFFSET = 12;
  constexpr auto DISTANCE_IMAGE_INFO_DATA_SIZE = 360;

  constexpr auto NR_MODEL_PARAMS = 32;
  constexpr auto AMPL_NORM_FACTOR_VECTOR_SIZE = 3;
  constexpr auto EXTR_OPTIC_USER_VECTOR_SIZE = 6;

  enum class extrinsic_param : std::uint32_t
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
    uint32_t model_iD;
    float model_parameters[NR_MODEL_PARAMS];
  };

  class IFM3D_FRAME_GRABBER_EXPORT DistanceImageInfo
  {
    const float dist_resolution;
    const float ampl_resolution;
    const std::vector<float> amp_norm_factors;
    const std::vector<float> extrinsic_optic_to_user;
    const IntrinsicCalibration intrinsic_calibration;
    const IntrinsicCalibration inverse_intrinsic_calibration;
    const std::vector<std::uint16_t> u16_distance_buffer;
    const std::vector<std::uint16_t> u16_amplitude_buffer;
    const std::uint32_t width, height;
    const std::vector<uint64_t> timestamps_nsec;
    const std::vector<float> exposure_times_sec;

  public:
    DistanceImageInfo(const float dist_res,
                      const float ampl_res,
                      const std::vector<float>& amp_norm_fctrs,
                      const std::vector<float>& extr_opt_to_usr,
                      const IntrinsicCalibration& intr_calib,
                      const IntrinsicCalibration& inv_intr_calib,
                      const std::vector<std::uint16_t>& distance_buffer,
                      const std::vector<std::uint16_t>& amplitude_buffer,
                      const std::vector<uint64_t>& timestamps_nsec,
                      const std::vector<float>& exposure_times_sec,
                      const std::uint32_t width,
                      const std::uint32_t height);
    ~DistanceImageInfo() = default;
    std::vector<std::uint8_t> getXYZDVector();
    std::vector<std::uint8_t> getAmplitudeVector();

    auto
    getExtrinsicOpticToUser()
    {
      return extrinsic_optic_to_user;
    }

    auto
    getIntrinsicCalibration()
    {
      return intrinsic_calibration;
    }

    auto
    getInverseIntrinsicCalibration()
    {
      return inverse_intrinsic_calibration;
    }
    auto
    getNPTS()
    {
      return (width * height);
    }
    auto
    getWidth()
    {
      return width;
    }
    auto
    getHeight()
    {
      return height;
    }

    /**
     * @brief returns the timestamps in nano seconds
     */
    std::vector<uint64_t>
    getTimestamps()
    {
      return timestamps_nsec;
    }

    /**
     * @brief return the exposure time for each
     * phase data
     */
    std::vector<float>
    getExposureTimes()
    {
      return exposure_times_sec;
    }

    /**
     * @brief multiply distance noise image with distance resolution
     * return Buffer with float values
     */
    ifm3d::Buffer applyDistanceResolution(
      const ifm3d::Buffer& ui16_distance_buffer);
  };
  using DistanceImageInfoPtr = std::unique_ptr<DistanceImageInfo>;
  IFM3D_FRAME_GRABBER_EXPORT DistanceImageInfoPtr
  CreateDistanceImageInfo(const std::vector<std::uint8_t>& data_buffer,
                          const std::size_t distimageinfo_idx,
                          const std::size_t dist_idx,
                          const std::size_t amp_idx,
                          const std::uint32_t width,
                          const std::uint32_t height);
} // end: namespace ifm3d

#endif // IFM3D_DISTANCE_IMAGE_INFO_H
