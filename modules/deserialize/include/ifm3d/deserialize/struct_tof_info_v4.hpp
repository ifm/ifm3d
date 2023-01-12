// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_TOF_INFO_V3_HPP
#define IFM3D_DESERIALIZE_STRUCT_TOF_INFO_V3_HPP

#include <ifm3d/deserialize/deserialize.h>
#include <array>
#include <chrono>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/buffer.h>

namespace ifm3d
{
  namespace
  {
    constexpr auto VERSION_INDEX = 0x0000;
    constexpr auto DISTANCE_RESOLUTION_INDEX = 0x0004;
    constexpr auto AMPLITUDE_RESOLUTION_INDEX = 0x0008;
    constexpr auto AMPLITUDE_NORM_FACTOR_INDEX = 0x000C;
    constexpr auto EXTRINSIC_OPTICAL_TO_USER_INDEX = 0x0018;
    constexpr auto INTRINSIC_CALIBRATION_INDEX = 0x0030;
    constexpr auto INVERSE_INTRINSIC_CALIBRATION_INDEX = 0x00B4;
    constexpr auto EXPOSURE_TIMESTAMPS_INDEX = 0x0138;
    constexpr auto EXPOSURE_TIMES_INDEX = 0x0150;
    constexpr auto ILLUTEMPERATURE_INDEX = 0x015C;
    constexpr auto MODE_INDEX = 0x0160;
    constexpr auto IMAGER_INDEX = 0x0180;
  };

  class TofInfoV3
  {

  public:
    using Ptr = std::shared_ptr<TofInfoV3>;
    struct ExtrinsicOpticToUser
    {
      using Ptr = std::shared_ptr<struct ExtrinsicOpticToUser>;
      float transX; // value in meter
      float transY; // value in meter
      float transZ; // value in meter
      float rotX;   // value in radians
      float rotY;   // value in radians
      float rotZ;   // value in radians

      void
      Read(const uint8_t* data)
      {
        transX = mkval<float>(data + sizeof(float) * 0);
        transY = mkval<float>(data + sizeof(float) * 1);
        transZ = mkval<float>(data + sizeof(float) * 2);
        rotX = mkval<float>(data + sizeof(float) * 3);
        rotY = mkval<float>(data + sizeof(float) * 4);
        rotZ = mkval<float>(data + sizeof(float) * 5);
      }
    };
    using ExtrinsicOpticToUser = struct ExtrinsicOpticToUser;

    struct Calibration
    {
      using Ptr = std::shared_ptr<struct Calibration>;
      uint32_t modelID;
      std::array<float, 32> modelParameters;
      void
      Read(const uint8_t* data)
      {
        modelID = mkval<uint32_t>(data);
        mkarray<float, 32>(data + sizeof(uint32_t), modelParameters);
      }
    };

    using IntrinsicCalibration = struct Calibration;
    using InverseIntrinsicCalibration = struct Calibration;

    void
    Read(const uint8_t* data, size_t size)
    {
      if (size < tof_info_v3_size)
        {
          throw ifm3d::Error(IFM3D_BUFFER_NOT_COMPATIABLE);
        }
      const uint8_t* start_ptr = data;
      version = mkval<std::uint32_t>(start_ptr + VERSION_INDEX);
      distance_resolution =
        mkval<float>(start_ptr + DISTANCE_RESOLUTION_INDEX);
      amplitude_resolution =
        mkval<float>(start_ptr + AMPLITUDE_RESOLUTION_INDEX);
      mkarray<float, 3>(start_ptr + AMPLITUDE_NORM_FACTOR_INDEX,
                        amp_normalization_factors);
      extrisic_optic_to_user.Read(start_ptr + EXTRINSIC_OPTICAL_TO_USER_INDEX);
      intrinsic_calibration.Read(start_ptr + INTRINSIC_CALIBRATION_INDEX);
      inverse_intrinsic_calibration.Read(start_ptr +
                                         INVERSE_INTRINSIC_CALIBRATION_INDEX);
      mkarray<uint64_t, 3>(start_ptr + EXPOSURE_TIMESTAMPS_INDEX,
                           exposure_timestamps_ns);
      mkarray<uint32_t, 3>(start_ptr + EXPOSURE_TIMES_INDEX, exposure_times_s);
      illu_temperature = mkval<float>(start_ptr + ILLUTEMPERATURE_INDEX);
      mkarray<char, 32>(start_ptr + MODE_INDEX, mode);
      mkarray<char, 32>(start_ptr + IMAGER_INDEX, imager);
    };

    uint32_t version;
    float distance_resolution;
    float amplitude_resolution;
    std::array<float, 3> amp_normalization_factors;
    ExtrinsicOpticToUser extrisic_optic_to_user;
    IntrinsicCalibration intrinsic_calibration;
    InverseIntrinsicCalibration inverse_intrinsic_calibration;
    std::array<uint64_t, 3> exposure_timestamps_ns;
    std::array<uint32_t, 3> exposure_times_s;
    float illu_temperature;
    std::array<char, 32> mode;
    std::array<char, 32> imager;
    const size_t tof_info_v3_size = 428;

    static TofInfoV3
    Deserialize(const Buffer& tof_info_buffer)
    {
      TofInfoV3 tof_info_v3;

      tof_info_v3.Read(tof_info_buffer.ptr<uint8_t>(0),
                       tof_info_buffer.size());
      return tof_info_v3;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_TOF_INFO_V3_HPP