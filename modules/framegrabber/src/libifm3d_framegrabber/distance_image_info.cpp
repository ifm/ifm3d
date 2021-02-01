/*
 * Copyright 2020 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <o3r_uncompress_di.h>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/fg/distance_image_info.h>

namespace ifm3d
{
  std::vector<std::uint16_t>
  readU16Vector(std::size_t idx,
                const std::vector<std::uint8_t>& data_buffer,
                const std::uint32_t npts)
  {
    auto chunk_size = ifm3d::mkval<uint32_t>(data_buffer.data() + idx +
                                             CHUNK_SIZE_INFO_OFFSET);
    auto data_offset = ifm3d::mkval<std::uint32_t>(data_buffer.data() + idx +
                                                   HEADER_SIZE_INFO_OFFSET);

    if ((chunk_size - data_offset) < npts * UINT16_DATA_SIZE)
      {
        std::cout << "invalid image size" << std::endl;
        return {};
      }

    std::vector<std::uint16_t> u16_buffer(npts);
    idx += data_offset;
    for (auto i = 0; i < npts; ++i)
      {
        u16_buffer[i] = ifm3d::mkval<std::uint16_t>(data_buffer.data() + idx);
        idx += UINT16_DATA_SIZE;
      }
    return u16_buffer;
  }

  std::vector<float>
  readFloatVector(const std::uint8_t* data_buffer, std::size_t size)
  {
    std::uint32_t data_offset{};
    std::vector<float> floatVector(size);
    for (auto i = 0; i < size; i++)
      {
        floatVector[i] = ifm3d::mkval<float>(data_buffer + data_offset);
        data_offset += FLOAT_DATA_SIZE;
      }
    return floatVector;
  }

  IntrinsicCalibration
  readIntrinsicCalibrationStruct(const std::uint8_t* data_buffer)
  {
    std::uint32_t data_offset{FLOAT_DATA_SIZE};
    IntrinsicCalibration intrinsicCalibration;
    intrinsicCalibration.model_iD = ifm3d::mkval<std::uint32_t>(data_buffer);
    for (auto i = 0; i < NR_MODEL_PARAMS; i++)
      {
        intrinsicCalibration.model_parameters[i] =
          ifm3d::mkval<float>(data_buffer + data_offset);
        data_offset += FLOAT_DATA_SIZE;
      }
    return intrinsicCalibration;
  }

  DistanceImageInfoPtr
  CreateDistanceImageInfo(const std::vector<std::uint8_t>& data_buffer,
                          const std::size_t didx,
                          const std::size_t aidx,
                          const std::uint32_t width,
                          const std::uint32_t height)
  {
    if (data_buffer.size() < ifm3d::IMG_BUFF_START)
      {
        return {};
      }
    // Index of Distance Image Info chunk
    auto distimageidx = ifm3d::get_chunk_index(
      data_buffer,
      ifm3d::image_chunk::O3R_DISTANCE_IMAGE_INFORMATION);

    if (distimageidx == INVALID_IDX || didx == INVALID_IDX ||
        aidx == INVALID_IDX)
      {
        return {};
      }

    // header size
    auto header_size = ifm3d::mkval<std::uint32_t>(
      data_buffer.data() + distimageidx + HEADER_SIZE_INFO_OFFSET);

    auto chunk_size = ifm3d::mkval<uint32_t>(
      data_buffer.data() + distimageidx + CHUNK_SIZE_INFO_OFFSET);

    if (DISTANCE_IMAGE_INFO_CHUNK_SIZE > chunk_size)
      {
        // the following reading of the buffer depends on a correct
        // chunk size
        VLOG(IFM3D_PROTO_DEBUG) << "Incorrect chunk size: " << chunk_size;
        return {};
      }

    auto data_offset = header_size;
    // Version
    auto dist_info_version = ifm3d::mkval<std::uint32_t>(
      data_buffer.data() + distimageidx + data_offset);
    data_offset += UINT32_DATA_SIZE;
    // Distance Resolution
    auto dist_resolution =
      ifm3d::mkval<float>(data_buffer.data() + distimageidx + data_offset);
    data_offset += FLOAT_DATA_SIZE;
    // Amplitude Resolution
    auto ampl_resolution =
      ifm3d::mkval<float>(data_buffer.data() + distimageidx + data_offset);
    data_offset += FLOAT_DATA_SIZE;
    // Ampl Normalization Factor Vector
    std::vector<float> amp_norm_factors{
      readFloatVector(data_buffer.data() + distimageidx + data_offset,
                      AMPL_NORM_FACTOR_VECTOR_SIZE)};
    data_offset += amp_norm_factors.size() * FLOAT_DATA_SIZE;
    // Extrinsic Optic to User Vector
    std::vector<float> extrinsic_optic_to_user{
      readFloatVector(data_buffer.data() + distimageidx + data_offset,
                      EXTR_OPTIC_USER_VECTOR_SIZE)};
    data_offset += extrinsic_optic_to_user.size() * FLOAT_DATA_SIZE;
    // Intrinsic Calibration
    IntrinsicCalibration intrinsic_calibration =
      readIntrinsicCalibrationStruct(data_buffer.data() + distimageidx +
                                     data_offset);
    data_offset += sizeof(intrinsic_calibration);
    // Inverse Intrinsic Calibration
    IntrinsicCalibration inverse_intrinsic_calibration =
      readIntrinsicCalibrationStruct(data_buffer.data() + distimageidx +
                                     data_offset);
    data_offset += sizeof(inverse_intrinsic_calibration);

    VLOG(IFM3D_PROTO_DEBUG)
      << "O3R_DISTANCE_IMAGE_INFORMATION \n\t-Chunk Index: " << distimageidx
      << "\n\t-Chunk size: " << chunk_size
      << "\n\t-Chunk end index: " << data_offset
      << "\n\t-Header size: " << header_size
      << "\n\t-Version: " << dist_info_version << "\n\t-didx: " << didx
      << " aidx: " << aidx << "\n\t-DistanceResolution: " << dist_resolution
      << "\n\t-AmplitudeResolution: " << ampl_resolution;

    return std::make_unique<DistanceImageInfo>(
      dist_resolution,
      ampl_resolution,
      amp_norm_factors,
      extrinsic_optic_to_user,
      intrinsic_calibration,
      inverse_intrinsic_calibration,
      readU16Vector(didx, data_buffer, width * height),
      readU16Vector(aidx, data_buffer, width * height),
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
      height(h)
  {}

  std::vector<std::uint8_t>
  DistanceImageInfo::getXYZDVector()
  {
    auto npts = getNPTS();
    if (u16_distance_buffer.size() < npts)
      {
        return {};
      }

    float xyzd[4 * npts];
    uint16_t u16Dist[npts];
    for (auto i = 0; i < npts; ++i)
      {
        u16Dist[i] = u16_distance_buffer[i];
      }

    if (xyzdFromDistance(
          xyzd + 3 * npts, // distance
          xyzd,            // X
          xyzd + npts,     // Y
          xyzd + 2 * npts, // Z
          u16Dist,
          dist_resolution,
          intrinsic_calibration.model_iD,
          intrinsic_calibration.model_parameters,
          extrinsic_optic_to_user[static_cast<int>(extrinsic_param::TRANS_X)],
          extrinsic_optic_to_user[static_cast<int>(extrinsic_param::TRANS_Y)],
          extrinsic_optic_to_user[static_cast<int>(extrinsic_param::TRANS_Z)],
          extrinsic_optic_to_user[static_cast<int>(extrinsic_param::ROT_X)],
          extrinsic_optic_to_user[static_cast<int>(extrinsic_param::ROT_Y)],
          extrinsic_optic_to_user[static_cast<int>(extrinsic_param::ROT_Z)],
          width,
          height) != 0)
      {
        LOG(ERROR) << "xyzdFromDistance calculation interrupted";
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

    float amplitude[npts];
    if (convertAmplitude(amplitude,
                         u16_amplitude_buffer.data(),
                         ampl_resolution,
                         width,
                         height) != 0)

      {
        LOG(ERROR) << "amplitude calculation interrupted";
        return {};
      }

    std::vector<std::uint8_t> ampl_bytes(npts * FLOAT_DATA_SIZE);
    std::memcpy(ampl_bytes.data(), amplitude, npts * FLOAT_DATA_SIZE);
    return ampl_bytes;
  }
} // end: namespace ifm3d
