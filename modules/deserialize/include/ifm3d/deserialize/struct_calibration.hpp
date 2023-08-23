// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_CALIBRATION_HPP
#define IFM3D_DESERIALIZE_STRUCT_CALIBRATION_HPP

#include <array>
#include <chrono>
#include <ifm3d/deserialize/deserialize_utils.hpp>

namespace ifm3d
{
  namespace calibration
  {
    /** @ingroup Deserialize
     *
     * @brief
     * All items are given in SI units, i.e. transXYZ are in [m] and rotXYZ are
     * in [rad].
     */
    struct ExtrinsicOpticToUser
    {
      using Ptr = std::shared_ptr<struct ExtrinsicOpticToUser>;
      float trans_x; // value in meter
      float trans_y; // value in meter
      float trans_z; // value in meter
      float rot_x;   // value in radians
      float rot_y;   // value in radians
      float rot_z;   // value in radians

      void
      Read(const uint8_t* data)
      {
        trans_x = mkval<float>(data + sizeof(float) * 0);
        trans_y = mkval<float>(data + sizeof(float) * 1);
        trans_z = mkval<float>(data + sizeof(float) * 2);
        rot_x = mkval<float>(data + sizeof(float) * 3);
        rot_y = mkval<float>(data + sizeof(float) * 4);
        rot_z = mkval<float>(data + sizeof(float) * 5);
      }
    };
    using ExtrinsicOpticToUser = struct ExtrinsicOpticToUser;

    /** @ingroup Deserialize */
    struct Calibration
    {
      using Ptr = std::shared_ptr<struct Calibration>;
      uint32_t model_id;
      std::array<float, 32> model_parameters;
      void
      Read(const uint8_t* data)
      {
        model_id = mkval<uint32_t>(data);
        mkarray<float, 32>(data + sizeof(uint32_t), model_parameters);
      }
    };
    /*@brief Intrisnsic parameter model for the device/head*/
    using IntrinsicCalibration = struct Calibration;
    /*@brief Inverse intrisnsic parameter model for the device/head*/
    using InverseIntrinsicCalibration = struct Calibration;
  } // end namespace calibration

} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_CALIBRATION_HPP