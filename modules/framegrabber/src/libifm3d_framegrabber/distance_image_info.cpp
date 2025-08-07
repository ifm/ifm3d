/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/distance_image_info.h>
#include <ifm3d/fg/organizer_utils.h>
#include <iostream>
#include <memory>
#include <o3r_uncompress_di.h>
#include <tuple>
#include <utility>
#include <vector>

namespace ifm3d
{
  const std::size_t NUM_EXPOSURE_TIMESTAMP = 3;
  const std::size_t NUM_EXPOSURE_TIME = 3;

  constexpr auto FLOAT_DATA_SIZE = sizeof(float);
  constexpr auto UINT32_DATA_SIZE = sizeof(std::uint32_t);
  constexpr auto UINT16_DATA_SIZE = sizeof(std::uint16_t);
  using DiffT = std::vector<float>::difference_type;

  namespace
  {
    std::vector<std::uint16_t>
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
          u16_buffer[i] =
            ifm3d::mkval<std::uint16_t>(data_buffer.data() + idx);
          idx += UINT16_DATA_SIZE;
        }
      return u16_buffer;
    }

    template <typename T>
    std::pair<std::vector<T>, size_t>
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

    std::vector<float>
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

    IntrinsicCalibration
    read_intrinsic_calibration_struct(const std::uint8_t* data_buffer)
    {
      std::uint32_t const data_offset{FLOAT_DATA_SIZE};
      IntrinsicCalibration intrinsic_calibration{};
      intrinsic_calibration.model_id =
        ifm3d::mkval<std::uint32_t>(data_buffer);
      for (auto [i, offset] = std::tuple{0, FLOAT_DATA_SIZE};
           i < NR_MODEL_PARAMS;
           ++i, offset += FLOAT_DATA_SIZE)
        {
          intrinsic_calibration.model_parameters[i] =
            ifm3d::mkval<float>(data_buffer + offset);
        }
      return intrinsic_calibration;
    }
  }

  DistanceImageInfoPtr
  create_distance_image_info(const std::vector<std::uint8_t>& data_buffer,
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
    : _dist_resolution(dist_res),
      _ampl_resolution(ampl_res),
      _amp_norm_factors(amp_norm_fctrs),
      _extrinsic_optic_to_user(extr_opt_to_usr),
      _intrinsic_calibration(intr_calib),
      _inverse_intrinsic_calibration(inv_intr_calib),
      _u16_distance_buffer(distance_buffer),
      _u16_amplitude_buffer(amplitude_buffer),
      _width(w),
      _height(h),
      _timestamps_nsec(timestamps_nsec),
      _exposure_times_sec(exposure_times_sec)
  {}

  std::vector<std::uint8_t>
  DistanceImageInfo::GetXyzdVector()
  {
    auto npts = GetNpts();
    if (_u16_distance_buffer.size() < npts)
      {
        return {};
      }

    std::vector<float> xyzd_float(static_cast<size_t>(4) * npts);
    auto* xyzd = (float*)xyzd_float.data();

    std::vector<uint16_t> dist_u16(npts);
    auto* u16_dist = (uint16_t*)dist_u16.data();

    for (unsigned int i = 0; i < npts; ++i)
      {
        u16_dist[i] = _u16_distance_buffer[i];
      }

    if (xyzd_from_distance(xyzd + (static_cast<size_t>(3 * npts)), // distance
                           xyzd,                                   // X
                           xyzd + npts,                            // Y
                           xyzd + (static_cast<size_t>(2 * npts)), // Z
                           u16_dist,
                           _dist_resolution,
                           _intrinsic_calibration.model_id,
                           _intrinsic_calibration.model_parameters.data(),
                           _extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::ExtrinsicParam::TRANS_X)],
                           _extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::ExtrinsicParam::TRANS_Y)],
                           _extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::ExtrinsicParam::TRANS_Z)],
                           _extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::ExtrinsicParam::ROT_X)],
                           _extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::ExtrinsicParam::ROT_Y)],
                           _extrinsic_optic_to_user[static_cast<int>(
                             ifm3d::ExtrinsicParam::ROT_Z)],
                           _width,
                           _height) != 0)
      {
        LOG_ERROR("xyzdFromDistance calculation interrupted");
        return {};
      }

    std::vector<std::uint8_t> xyzd_bytes(static_cast<unsigned long>(4) * npts *
                                         FLOAT_DATA_SIZE);
    std::memcpy(xyzd_bytes.data(),
                xyzd,
                static_cast<unsigned long>(4) * npts * FLOAT_DATA_SIZE);

    return xyzd_bytes;
  }

  std::vector<std::uint8_t>
  DistanceImageInfo::GetAmplitudeVector()
  {
    auto npts = GetNpts();
    if (_u16_amplitude_buffer.size() < npts)
      {
        return {};
      }

    std::vector<float> amplitude_float(npts);
    auto* amplitude = (float*)amplitude_float.data();

    if (convert_amplitude(amplitude,
                          _u16_amplitude_buffer.data(),
                          _ampl_resolution,
                          _width,
                          _height) != 0)

      {
        LOG_ERROR("amplitude calculation interrupted");
        return {};
      }

    std::vector<std::uint8_t> ampl_bytes(npts * FLOAT_DATA_SIZE);
    std::memcpy(ampl_bytes.data(), amplitude, npts * FLOAT_DATA_SIZE);
    return ampl_bytes;
  }

  ifm3d::Buffer
  DistanceImageInfo::ApplyDistanceResolution(
    const ifm3d::Buffer& ui16_distance_buffer) const
  {
    auto dist_noise_image =
      Buffer(_width, _height, 1, ifm3d::pixel_format::FORMAT_32F);

    if (convert_distance_noise(dist_noise_image.Ptr<float>(0),
                               ui16_distance_buffer.Ptr<uint16_t>(0),
                               _dist_resolution,
                               _width,
                               _height) != 0)

      {
        LOG_ERROR("distance noise calculation interrupted");
        return {};
      }

    return dist_noise_image;
  }

} // end: namespace ifm3d
