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
  } // end namespace calibration

} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_CALIBRATION_HPP