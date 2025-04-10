// -*- c++ -*-
/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION_V1_HPP
#define IFM3D_DESERIALIZE_STRUCT_O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION_V1_HPP

#include <array>
#include <chrono>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/deserialize/deserialize_utils.hpp>

namespace ifm3d
{
  namespace
  {
    constexpr auto ODS_EXTRINSIC_CALIBRATION_CORRECTION_VERSION_INDEX = 0x0000;
    constexpr auto ODS_EXTRINSIC_CALIBRATION_CORRECTION_COMPLETION_RATE_INDEX =
      0x0004;
    constexpr auto ODS_EXTRINSIC_CALIBRATION_CORRECTION_ROT_DELTA_VALUE_INDEX =
      0x0008;
    constexpr auto ODS_EXTRINSIC_CALIBRATION_CORRECTION_ROT_DELTA_VALID_INDEX =
      0x0014;
    constexpr auto
      ODS_EXTRINSIC_CALIBRATION_CORRECTION_ROT_HEAD_TO_USER_INDEX = 0x0017;
  };

  /** @ingroup Deserialize */
  class ODSExtrinsicCalibrationCorrectionV1
  {
  public:
    using Ptr = std::shared_ptr<ODSExtrinsicCalibrationCorrectionV1>;

    void
    Read(const uint8_t* start_ptr, size_t size)
    {
      if (size < ods_extrinsic_calibration_correction_minimum_size)
        {
          throw ifm3d::Error(IFM3D_CORRUPTED_STRUCT);
        }

      version = mkval<std::uint32_t>(
        start_ptr + ODS_EXTRINSIC_CALIBRATION_CORRECTION_VERSION_INDEX);
      completion_rate = mkval<float>(
        start_ptr +
        ODS_EXTRINSIC_CALIBRATION_CORRECTION_COMPLETION_RATE_INDEX);
      mkarray<float, 3>(
        start_ptr + ODS_EXTRINSIC_CALIBRATION_CORRECTION_ROT_DELTA_VALUE_INDEX,
        rot_delta_value);
      mkarray<std::uint8_t, 3>(
        start_ptr + ODS_EXTRINSIC_CALIBRATION_CORRECTION_ROT_DELTA_VALID_INDEX,
        rot_delta_valid);
      mkarray<float, 3>(
        start_ptr +
          ODS_EXTRINSIC_CALIBRATION_CORRECTION_ROT_HEAD_TO_USER_INDEX,
        rot_head_to_user);
    };

    /*@brief Version Number (current=1)*/
    uint32_t version;
    /*@brief
     * The completion rate ranging from 0.0 to 1.0, where 1.0 indicates that
     * sufficient data was collected to estimate all three delta values.
     */
    float completion_rate;
    /*@brief
     * The estimated rot delta value [rad] to correct the extrinsic
     * calibration. Array of [X, Y, Z].
     */
    std::array<float, 3> rot_delta_value;
    /*@brief
     * A flag indicating a valid estimation of rotation delta value (0:
     * invalid, 1: valid). Array of [X, Y, Z].
     */
    std::array<uint8_t, 3> rot_delta_valid;
    /*@brief
     * The rotation value [rad] of the (corrected) extrinsic calibration
     * (extrinsicHeadToUser). Array of [X, Y, Z].
     */
    std::array<float, 3> rot_head_to_user;

  private:
    static constexpr size_t ods_extrinsic_calibration_correction_minimum_size =
      35;

  public:
    static ODSExtrinsicCalibrationCorrectionV1
    Deserialize(const Buffer& ods_extrinsic_calibration_correction)
    {
      ODSExtrinsicCalibrationCorrectionV1
        ods_extrinsic_calibration_correction_v1;

      ods_extrinsic_calibration_correction_v1.Read(
        ods_extrinsic_calibration_correction.ptr<uint8_t>(0),
        ods_extrinsic_calibration_correction.size());
      return ods_extrinsic_calibration_correction_v1;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION_V1_HPP