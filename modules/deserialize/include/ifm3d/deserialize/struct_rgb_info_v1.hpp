// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_RGB_INFO_V1_HPP
#define IFM3D_DESERIALIZE_STRUCT_RGB_INFO_V1_HPP

#include <array>
#include <chrono>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/deserialize/deserialize_utils.hpp>
#include <ifm3d/deserialize/struct_calibration.hpp>

namespace ifm3d
{
  namespace
  {
    constexpr auto RGB_INFO_VERSION_INDEX = 0x0000;
    constexpr auto RGB_INFO_FRAME_COUNTER_INDEX = 0x0004;
    constexpr auto RGB_INFO_TIMESTAMPS_INDEX = 0x0008;
    constexpr auto RGB_INFO_EXPOSURE_TIMES_INDEX = 0x0010;
    constexpr auto RGB_INFO_EXTRINSIC_OPTICAL_TO_USER_INDEX = 0x0014;
    constexpr auto RGB_INFO_INTRINSIC_CALIBRATION_INDEX = 0x002C;
    constexpr auto RGB_INFO_INVERSE_INTRINSIC_CALIBRATION_INDEX = 0x00B0;
  };

  class RGBInfoV1
  {

  public:
    using Ptr = std::shared_ptr<RGBInfoV1>;

    void
    Read(const uint8_t* data, size_t size)
    {
      if (size < rgb_info_v1_size)
        {
          throw ifm3d::Error(IFM3D_CORRUPTED_STRUCT);
        }
      const uint8_t* start_ptr = data;
      version = mkval<std::uint32_t>(start_ptr + RGB_INFO_VERSION_INDEX);
      frame_counter =
        mkval<std::uint32_t>(start_ptr + RGB_INFO_FRAME_COUNTER_INDEX);
      timestamp_ns =
        mkval<std::uint64_t>(start_ptr + RGB_INFO_TIMESTAMPS_INDEX);
      exposure_time = mkval<float>(start_ptr + RGB_INFO_EXPOSURE_TIMES_INDEX);
      extrisic_optic_to_user.Read(start_ptr +
                                  RGB_INFO_EXTRINSIC_OPTICAL_TO_USER_INDEX);
      intrinsic_calibration.Read(start_ptr +
                                 RGB_INFO_INTRINSIC_CALIBRATION_INDEX);
      inverse_intrinsic_calibration.Read(
        start_ptr + RGB_INFO_INVERSE_INTRINSIC_CALIBRATION_INDEX);
    };
    /*@brief Version of the RGB_INFO data*/
    std::uint32_t version;
    /*@brief Frame count, The frame counter is initialized to 0 at the
     * initialization */
    std::uint32_t frame_counter;
    /*@brief The timestamp of the 2D image, given in nano second*/
    std::uint64_t timestamp_ns;
    /*@brief Actual exposure time of the 2D image*/
    float exposure_time;
    /*@brief Extrinsic optic paramter of the 2D head*/
    calibration::ExtrinsicOpticToUser extrisic_optic_to_user;
    /*@brief Intrinsic Calibration parameters*/
    calibration::IntrinsicCalibration intrinsic_calibration;
    /*@brief Inverse intrinsic Calibration parameters*/
    calibration::InverseIntrinsicCalibration inverse_intrinsic_calibration;
    /*@brief Size of RGB_INFO in bytes*/
    const size_t rgb_info_v1_size = 308;

    static RGBInfoV1
    Deserialize(const Buffer& rgb_info_buffer)
    {
      RGBInfoV1 rgb_info_v1;

      rgb_info_v1.Read(rgb_info_buffer.ptr<uint8_t>(0),
                       rgb_info_buffer.size());
      return rgb_info_v1;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_RGB_INFO_V1_HPP