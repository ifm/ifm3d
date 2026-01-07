// -*- c++ -*-
/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_IMU_INFO_V1_HPP
#define IFM3D_DESERIALIZE_STRUCT_IMU_INFO_V1_HPP

#include <array>
#include <ifm3d/deserialize/deserialize_utils.hpp>
#include <ifm3d/deserialize/struct_calibration.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>

namespace ifm3d
{
  namespace
  {
    constexpr auto IMU_INFO_IMU_VERSION_INDEX = 0x0000;
    constexpr auto IMU_INFO_IMU_SAMPLES_INDEX = 0x0004;
    constexpr auto IMU_INFO_NUM_SAMPLE_INDEX = 0x1304;
    constexpr auto IMU_INFO_EXTRINSIC_IMU_TO_USER_INDEX = 0x1308;
    constexpr auto IMU_INFO_EXTRINSIC_IMU_TO_VPU_INDEX = 0x1320;
    constexpr auto IMU_INFO_IMU_FIFO_RCV_TIMESTAMP_INDEX = 0x1338;
    constexpr auto ARRAY_SIZE_IMU_SAMPLES = 128;
  };

  /** @ingroup Deserialize */
  class IMUInfoV1
  {
  public:
    using Ptr = std::shared_ptr<IMUInfoV1>;

    bool
    IsValid(const uint8_t*, size_t size)
    {
      return size >= IMU_INFO_V1_SIZE;
    }

    void
    Read(const uint8_t* start_ptr, size_t size)
    {
      if (!IsValid(start_ptr, size))
        {
          throw ifm3d::Error(IFM3D_CORRUPTED_STRUCT);
        }

      imu_version =
        mkval<std::uint32_t>(start_ptr + IMU_INFO_IMU_VERSION_INDEX);

      for (std::uint32_t i = 0; i < ARRAY_SIZE_IMU_SAMPLES; ++i)
        {
          constexpr size_t IMU_SAMPLE_BYTE_SIZE =
            sizeof(std::uint16_t) + sizeof(std::uint64_t) + 7 * sizeof(float);

          static_assert(sizeof(calibration::IMUSample) >= IMU_SAMPLE_BYTE_SIZE,
                        "IMUSample struct too small for serialized data");

          imu_samples[i].Read(start_ptr + IMU_INFO_IMU_SAMPLES_INDEX +
                              i * IMU_SAMPLE_BYTE_SIZE);
        }

      num_samples =
        mkval<std::uint32_t>(start_ptr + IMU_INFO_NUM_SAMPLE_INDEX);

      extrinsic_imu_to_user.Read(start_ptr +
                                 IMU_INFO_EXTRINSIC_IMU_TO_USER_INDEX);

      extrinsic_imu_to_vpu.Read(start_ptr +
                                IMU_INFO_EXTRINSIC_IMU_TO_VPU_INDEX);

      imu_fifo_rcv_timestamp = mkval<std::uint64_t>(
        start_ptr + IMU_INFO_IMU_FIFO_RCV_TIMESTAMP_INDEX);
    };

    /*@brief Version of the IMU_INFO data*/
    std::uint32_t imu_version{};
    /*@brief Array containing IMU samples data*/
    std::array<calibration::IMUSample, ARRAY_SIZE_IMU_SAMPLES> imu_samples;
    /*@brief Number of valid samples in imu_samples array*/
    std::uint32_t num_samples{};
    /*@brief Extrinsic calibration for converting between IMU and User
     * coordinates (combines both, i.e., IMUToVPU and VPUToUser, extrinsic
     * calibrations)*/
    calibration::AlgoExtrinsicCalibration extrinsic_imu_to_user;
    /*@brief Extrinsic calibration for converting between IMU and User
     * coordinates (only the IMUToVPU calibration)*/
    calibration::AlgoExtrinsicCalibration extrinsic_imu_to_vpu;
    /*@brief The receive timestamp of the IMU FIFO buffer. This is taken as
     * soon as possible after the data has been arrived and might be used for
     * manual synchronization of the IMU's hardware timestamps.*/
    std::uint64_t imu_fifo_rcv_timestamp{};

    /*@brief Size of IMU_INFO in bytes*/
    static constexpr size_t IMU_INFO_V1_SIZE = 0x1340;

    static IMUInfoV1
    Deserialize(const Buffer& imu_info_buffer)
    {
      IMUInfoV1 imu_info_v1{};

      imu_info_v1.Read(imu_info_buffer.ptr<uint8_t>(0),
                       imu_info_buffer.size());
      return imu_info_v1;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_IMU_INFO_V1_HPP