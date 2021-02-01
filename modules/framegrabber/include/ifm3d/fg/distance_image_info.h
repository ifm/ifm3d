// -*- c++ -*-
/*
 * Copyright 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_DISTANCE_IMAGE_INFO_H
#define IFM3D_DISTANCE_IMAGE_INFO_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <memory>

namespace ifm3d
{
  constexpr auto CHUNK_SIZE_INFO_OFFSET = 4;
  constexpr auto HEADER_SIZE_INFO_OFFSET = 8;
  constexpr auto HEADER_VERSION_INFO_OFFSET = 12;
  constexpr auto DISTANCE_IMAGE_INFO_CHUNK_SIZE = 360;

  constexpr auto NR_MODEL_PARAMS = 32;
  constexpr auto AMPL_NORM_FACTOR_VECTOR_SIZE = 3;
  constexpr auto EXTR_OPTIC_USER_VECTOR_SIZE = 6;

  struct IntrinsicCalibration
  {
    uint32_t model_iD;
    float model_parameters[NR_MODEL_PARAMS];
  };

  class DistanceImageInfo
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

  public:
    DistanceImageInfo(const float dist_res,
                      const float ampl_res,
                      const std::vector<float>& amp_norm_fctrs,
                      const std::vector<float>& extr_opt_to_usr,
                      const IntrinsicCalibration& intr_calib,
                      const IntrinsicCalibration& inv_intr_calib,
                      const std::vector<std::uint16_t>& distance_buffer,
                      const std::vector<std::uint16_t>& amplitude_buffer,
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
    getNPTS()
    {
      return (width * height);
    }
  };
  using DistanceImageInfoPtr = std::unique_ptr<DistanceImageInfo>;
  DistanceImageInfoPtr CreateDistanceImageInfo(
    const std::vector<std::uint8_t>& data_buffer,
    const std::size_t didx,
    const std::size_t aidx,
    const std::uint32_t width,
    const std::uint32_t height);
} // end: namespace ifm3d

#endif // IFM3D_DISTANCE_IMAGE_INFO_H
