/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ifm3d/fg/buffer.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <o3r_uncompress_di.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/distance_image_info.h>
#include <tuple>
#include <vector>
#include <utility>

namespace ifm3d
{
  const std::size_t NUM_EXPOSURE_TIMESTAMP = 3;
  const std::size_t NUM_EXPOSURE_TIME = 3;

  constexpr auto FLOAT_DATA_SIZE = sizeof(float);
  constexpr auto UINT32_DATA_SIZE = sizeof(std::uint32_t);
  constexpr auto UINT16_DATA_SIZE = sizeof(std::uint16_t);
  using DiffT = std::vector<float>::difference_type;

  static std::vector<std::uint16_t>
  read_u16_vector(std::size_t idx,
                  const std::vector<std::uint8_t>& data_buffer,
                  const std::uint32_t npts)
  {
    const auto chunk_size = ifm3d::mkval<uint32_t>(data_buffer.data() + idx +
                                                   CHUNK_SIZE_INFO_OFFSET);
    const auto data_offset = ifm3d::mkval<std::uint32_t>(
      data_buffer.data() + idx + HEADER_SIZE_INFO_OFFSET);

    if ((chunk_size - data_offset) < npts * UINT16_DATA_SIZE)
      {
        std::cout << "invalid image size" << '\n';
        return {};
      }

    std::vector<std::uint16_t> u16_buffer(npts);
    idx += data_offset;
    for (std::uint32_t i = 0; i < npts; ++i)
      {
        u16_buffer[i] = ifm3d::mkval<std::uint16_t>(data_buffer.data() + idx);
        idx += UINT16_DATA_SIZE;
      }
    return u16_buffer;
  }

  template <typename T>
  static std::pair<std::vector<T>, size_t>
  read_t_vector(const std::uint8_t* data_buffer, std::size_t size)
  {
    std::uint32_t data_offset{};
    std::vector<T> t_vector(size);
    for (size_t i = 0; i < size; i++)
      {
        t_vector[i] = ifm3d::mkval<T>(data_buffer + data_offset);
        data_offset += sizeof(T);
      }
    return {t_vector, data_offset};
  }

  static std::vector<float>
  read_float_vector(const std::uint8_t* data_buffer, std::size_t size)
  {
    std::uint32_t data_offset{};
    std::vector<float> float_vector(size);
    for (std::size_t i = 0; i < size; i++)
      {
        float_vector[i] = ifm3d::mkval<float>(data_buffer + data_offset);
        data_offset += FLOAT_DATA_SIZE;
      }
    return float_vector;
  }

  static IntrinsicCalibration
  read_intrinsic_calibration_struct(const std::uint8_t* data_buffer)
  {
    std::uint32_t const data_offset{FLOAT_DATA_SIZE};
    IntrinsicCalibration intrinsic_calibration{};
    intrinsic_calibration.model_iD = ifm3d::mkval<std::uint32_t>(data_buffer);
    for (auto [i, offset] = std::tuple{0, FLOAT_DATA_SIZE};
         i < NR_MODEL_PARAMS;
         ++i, offset += FLOAT_DATA_SIZE)
      {
        intrinsic_calibration.model_parameters[i] =
          ifm3d::mkval<float>(data_buffer + offset);
      }
    return intrinsic_calibration;
  }

  DistanceImageInfoPtr
  CreateDistanceImageInfo(const std::vector<std::uint8_t>& data_buffer,
                          const std::size_t distimageinfo_idx,
                          const std::size_t dist_idx,
                          const std::size_t amp_idx,
                          const std::uint32_t width,
                          const std::uint32_t height)
  {
    // header size
    auto header_size = ifm3d::mkval<std::uint32_t>(
      data_buffer.data() + distimageinfo_idx + HEADER_SIZE_INFO_OFFSET);

    auto chunk_size = ifm3d::mkval<uint32_t>(
      data_buffer.data() + distimageinfo_idx + CHUNK_SIZE_INFO_OFFSET);

    if (DISTANCE_IMAGE_INFO_DATA_SIZE > chunk_size)
      {
        // the following reading of the buffer depends on a correct
        // chunk size
        LOG_VERBOSE("Incorrect chunk size: {}", chunk_size);
        return {};
      }

    auto data_offset = header_size;
    // Version
    auto dist_info_version = ifm3d::mkval<std::uint32_t>(
      data_buffer.data() + distimageinfo_idx + data_offset);
    data_offset += UINT32_DATA_SIZE;
    // Distance Resolution
    auto dist_resolution = ifm3d::mkval<float>(
      data_buffer.data() + distimageinfo_idx + data_offset);
    data_offset += FLOAT_DATA_SIZE;
    // Amplitude Resolution
    auto ampl_resolution = ifm3d::mkval<float>(
      data_buffer.data() + distimageinfo_idx + data_offset);
    data_offset += FLOAT_DATA_SIZE;
    // Ampl Normalization Factor Vector
    std::vector<float> const amp_norm_factors{
      read_float_vector(data_buffer.data() + distimageinfo_idx + data_offset,
                        AMPL_NORM_FACTOR_VECTOR_SIZE)};
    data_offset += amp_norm_factors.size() * FLOAT_DATA_SIZE;
    // Extrinsic Optic to User Vector
    std::vector<float> const extrinsic_optic_to_user{
      read_float_vector(data_buffer.data() + distimageinfo_idx + data_offset,
                        EXTR_OPTIC_USER_VECTOR_SIZE)};
    data_offset += extrinsic_optic_to_user.size() * FLOAT_DATA_SIZE;
    // Intrinsic Calibration
    IntrinsicCalibration const intrinsic_calibration =
      read_intrinsic_calibration_struct(data_buffer.data() +
                                        distimageinfo_idx + data_offset);
    data_offset += sizeof(intrinsic_calibration);
    // Inverse Intrinsic Calibration
    IntrinsicCalibration const inverse_intrinsic_calibration =
      read_intrinsic_calibration_struct(data_buffer.data() +
                                        distimageinfo_idx + data_offset);
    data_offset += sizeof(inverse_intrinsic_calibration);
    std::vector<uint64_t> timestamps_nsec{};
    std::vector<float> exposure_time_sec{};

    if (dist_info_version > 1)
      {
        size_t offset = 0;
        /* ExposureTimestamps */
        std::tie(timestamps_nsec, offset) = read_t_vector<uint64_t>(
          data_buffer.data() + distimageinfo_idx + data_offset,
          NUM_EXPOSURE_TIMESTAMP);
        data_offset += offset;

        /* Exposure Times */
        std::tie(exposure_time_sec, offset) = read_t_vector<float>(
          data_buffer.data() + distimageinfo_idx + data_offset,
          NUM_EXPOSURE_TIME);
        data_offset += offset;
      }
    /*exposure_timestamps will be blank for header version 1 of dist image
     * info*/
    if (timestamps_nsec.empty())
      {
        LOG_VERBOSE("dist image Header Version value is 1, does not support "
                    "exposure parameters");
      }

    LOG_VERBOSE("O3R_DISTANCE_IMAGE_INFORMATION\n"
                "\t-Chunk Index: {}\n"
                "\t-Chunk size: {}\n"
                "\t-Chunk end index: {}\n"
                "\t-Header size: {}\n"
                "\t-Version: {}\n"
                "\t-dist_idx: {} amp_idx: {}\n"
                "\t-DistanceResolution: {}\n"
                "\t-AmplitudeResolution: {}",
                distimageinfo_idx,
                chunk_size,
                data_offset,
                header_size,
                dist_info_version,
                dist_idx,
                amp_idx,
                dist_resolution,
                ampl_resolution);

    return std::make_unique<DistanceImageInfo>(
      dist_resolution,
      ampl_resolution,
      amp_norm_factors,
      extrinsic_optic_to_user,
      intrinsic_calibration,
      inverse_intrinsic_calibration,
      read_u16_vector(dist_idx, data_buffer, width * height),
      read_u16_vector(amp_idx, data_buffer, width * height),
      timestamps_nsec,
      exposure_time_sec,
      width,
      height);
  }

  DistanceImageInfo::DistanceImageInfo(
    const float dist_res,
    const float ampl_res,
    const std::vector<float>& amp_norm_fctrs,
    const std::vector<float>& extr_opt_to_usr,
    const IntrinsicCalibration& intr_calib,
    const IntrinsicCalibration& inv_intr_calib,
    const std::vector<std::uint16_t>& distance_buffer,
    const std::vector<std::uint16_t>& amplitude_buffer,
    const std::vector<uint64_t>& timestamps_nsec,
    const std::vector<float>& exposure_times_sec,
    const std::uint32_t w,
    const std::uint32_t h)
    : dist_resolution(dist_res),
      ampl_resolution(ampl_res),
      amp_norm_factors(amp_norm_fctrs),
      extrinsic_optic_to_user(extr_opt_to_usr),
      intrinsic_calibration(intr_calib),
      inverse_intrinsic_calibration(inv_intr_calib),
      u16_distance_buffer(distance_buffer),
      u16_amplitude_buffer(amplitude_buffer),
      width(w),
      height(h),
      timestamps_nsec(timestamps_nsec),
      exposure_times_sec(exposure_times_sec)
  {}

  std::vector<std::uint8_t>
  DistanceImageInfo::getXYZDVector()
  {
    auto npts = getNPTS();
    if (u16_distance_buffer.size() < npts)
      {
        return {};
      }

    std::vector<float> xyzd_float(4 * npts);
    auto* xyzd = (float*)xyzd_float.data();

    std::vector<uint16_t> dist_u16(npts);
    auto* u16Dist = (uint16_t*)dist_u16.data();

    for (unsigned int i = 0; i < npts; ++i)
      {
        u16Dist[i] = u16_distance_buffer[i];
      }

    if (xyzd_from_distance(xyzd + (3 * npts), // distance
                           xyzd,              // X
                           xyzd + npts,       // Y
                           xyzd + (2 * npts), // Z
                           u16Dist,
                           dist_resolution,
                           intrinsic_calibration.model_iD,
                           intrinsic_calibration.model_parameters,
                           extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::extrinsic_param::TRANS_X)],
                           extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::extrinsic_param::TRANS_Y)],
                           extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::extrinsic_param::TRANS_Z)],
                           extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::extrinsic_param::ROT_X)],
                           extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::extrinsic_param::ROT_Y)],
                           extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::extrinsic_param::ROT_Z)],
                           width,
                           height) != 0)
      {
        LOG_ERROR("xyzdFromDistance calculation interrupted");
        return {};
      }

    std::vector<std::uint8_t> xyzd_bytes(4 * npts * FLOAT_DATA_SIZE);
    std::memcpy(xyzd_bytes.data(), xyzd, 4 * npts * FLOAT_DATA_SIZE);

    return xyzd_bytes;
  }

  std::vector<std::uint8_t>
  DistanceImageInfo::getAmplitudeVector()
  {
    auto npts = getNPTS();
    if (u16_amplitude_buffer.size() < npts)
      {
        return {};
      }

    std::vector<float> amplitude_float(npts);
    auto* amplitude = (float*)amplitude_float.data();

    if (convert_amplitude(amplitude,
                          u16_amplitude_buffer.data(),
                          ampl_resolution,
                          width,
                          height) != 0)

      {
        LOG_ERROR("amplitude calculation interrupted");
        return {};
      }

    std::vector<std::uint8_t> ampl_bytes(npts * FLOAT_DATA_SIZE);
    std::memcpy(ampl_bytes.data(), amplitude, npts * FLOAT_DATA_SIZE);
    return ampl_bytes;
  }

  ifm3d::Buffer
  DistanceImageInfo::applyDistanceResolution(
    const ifm3d::Buffer& ui16_distance_buffer) const
  {
    auto dist_noise_image =
      Buffer(width, height, 1, ifm3d::pixel_format::FORMAT_32F);

    if (convert_distance_noise(dist_noise_image.ptr<float>(0),
                               ui16_distance_buffer.ptr<uint16_t>(0),
                               dist_resolution,
                               width,
                               height) != 0)

      {
        LOG_ERROR("distance noise calculation interrupted");
        return {};
      }

    return dist_noise_image;
  }

} // end: namespace ifm3d
