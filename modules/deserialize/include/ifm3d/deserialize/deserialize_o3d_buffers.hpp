// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_O3D_BUFFERS_HPP
#define IFM3D_DESERIALIZE_O3D_BUFFERS_HPP

#include <ifm3d/deserialize/deserialize_utils.hpp>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>

namespace ifm3d
{
  // intrinsic param key which cahn be used for indexing the vector of the
  // intrinsic parameter
  // NOLINTNEXTLINE(performance-enum-size)
  enum class IntrinsicParam : std::uint32_t
  {
    F_X = 0,   // Focal length of the camera in the sensor's x axis direction.
    F_Y = 1,   // Focal length of the camera in the sensor's y axis direction.
    M_X = 2,   // Main point in the sensor's x direction
    M_Y = 3,   // Main point in the sensor's x direction
    ALPHA = 4, // Skew parameter
    K1 = 5,    // First radial distortion coefficient
    K2 = 6,    // Second radial distortion coefficient
    K5 = 7,    // Third radial distortion coefficient
    K3 = 8,    // First tangential distortion coefficient
    K4 = 9,    // Second tangential distortion coefficient
    TRANS_X = 10, // Translation along x-direction in meters.
    TRANS_Y = 11, // Translation along y-direction in meters.
    TRANS_Z = 12, // Translation along Z-direction in meters.
    ROT_X = 13,   // Rotation along x-axis in radians. Positive values indicate
                  // clockwise rotation.
    ROT_Y = 14,   // Rotation along y-axis in radians. Positive values indicate
                  // clockwise rotation.
    ROT_Z = 15    // Rotation along z-axis in radians. Positive values indicate
                  // clockwise rotation.
  };

  namespace
  {
    constexpr auto O3D_NUMBER_OF_INTRINSIC_CALIBRATION_PARAM = 16;
    constexpr auto O3D_NUMBER_OF_INVERSE_INTRINSIC_CALIBRATION_PARAM =
      O3D_NUMBER_OF_INTRINSIC_CALIBRATION_PARAM;
    constexpr auto O3D_NUMBER_OF_EXTRINSIC_PARAM = 6;
    constexpr auto O3D_NUMBER_OF_EXPOSURE_TIMES = 3;
    constexpr auto O3D_ILLU_TEMP_VALUES = 1;
  }

  /*! @ingroup Deserialize
   * \class ifm3d::O3DInstrinsicCalibration
   * using O3DInstrinsicCalibration = ifm3d::ArrayDeserialize<float,
   * O3D_NUMBER_OF_INTRINSIC_CALIBRATION_PARAM> \see ifm3d::ArrayDeserialize
   */
  using O3DInstrinsicCalibration =
    ArrayDeserialize<float, O3D_NUMBER_OF_INTRINSIC_CALIBRATION_PARAM>;

  /*! @ingroup Deserialize
   * \class ifm3d::O3DInverseInstrinsicCalibration
   * using O3DInverseInstrinsicCalibration = ifm3d::ArrayDeserialize<float,
   * O3D_NUMBER_OF_INVERSE_INTRINSIC_CALIBRATION_PARAM> \see
   * ifm3d::ArrayDeserialize
   */
  using O3DInverseInstrinsicCalibration =
    ArrayDeserialize<float, O3D_NUMBER_OF_INVERSE_INTRINSIC_CALIBRATION_PARAM>;

  /*! @ingroup Deserialize
   * \class ifm3d::O3DExtrinsicCalibration
   * using O3DExtrinsicCalibration = ifm3d::ArrayDeserialize<float,
   * O3D_NUMBER_OF_EXTRINSIC_PARAM> \see ifm3d::ArrayDeserialize
   */
  using O3DExtrinsicCalibration =
    ArrayDeserialize<float, O3D_NUMBER_OF_EXTRINSIC_PARAM>;

  /*! @ingroup Deserialize
   * \class ifm3d::O3DExposureTimes
   * using O3DExposureTimes = ifm3d::ArrayDeserialize<uint32_t,
   * O3D_NUMBER_OF_EXPOSURE_TIMES> \see ifm3d::ArrayDeserialize
   */
  using O3DExposureTimes =
    ArrayDeserialize<uint32_t, O3D_NUMBER_OF_EXPOSURE_TIMES>;

  /*! @ingroup Deserialize
   * \class ifm3d::O3DILLUTemperature
   * using O3DILLUTemperature = ifm3d::ArrayDeserialize<float,
   * O3D_ILLU_TEMP_VALUES> \see ifm3d::ArrayDeserialize
   */
  using O3DIlluTemperature = ArrayDeserialize<float, O3D_ILLU_TEMP_VALUES>;

} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_O3D_BUFFERS_HPP