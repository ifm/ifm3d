// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_DESERIALIZE_DESERIALIZE_H
#define IFM3D_DESERIALIZE_DESERIALIZE_H

/** \defgroup Deserialize Deserialize Module */

#include <ifm3d/deserialize/struct_o3r_ods_extrinsic_calibration_correction_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_info_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_occupancy_grid_v1.hpp>
#include <ifm3d/deserialize/struct_o3r_ods_polar_occupancy_grid_v1.hpp>
#include <ifm3d/deserialize/struct_rgb_info_v1.hpp>
#include <ifm3d/deserialize/struct_tof_info_v3.hpp>
#include <ifm3d/deserialize/struct_tof_info_v4.hpp>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>
#include <variant>

namespace ifm3d
{
  using VariantType = std::variant<std::monostate,
                                   ODSInfoV1,
                                   TOFInfoV3,
                                   TOFInfoV4,
                                   RGBInfoV1,
                                   ODSOccupancyGridV1,
                                   ODSPolarOccupancyGridV1,
                                   ODSExtrinsicCalibrationCorrectionV1>;

  template <typename T>
  static T
  create_and_read(const uint8_t* data, size_t size)
  {
    T obj{};
    obj.Read(data, size);
    return obj;
  }

  /** @ingroup Deserialize
   *
   * @brief This function examines the buffer ID of the input buffer and uses
   * it to determine the correct deserialization strategy. The result is
   * returned as a VariantType which can hold any of the supported deserialized
   * structures. If deserialization fails or the buffer ID is unrecognized,
   * std::monostate is returned.
   *
   * @param[in] buffer The input buffer containing serialized data.
   * @return VariantType A variant containing the deserialized structure,
   *         or std::monostate if deserialization fails or the buffer ID is
   * unknown.
   */
  static VariantType
  deserialize(const uint8_t* data, size_t size, ifm3d::buffer_id buffer_id)
  {
    switch (buffer_id)
      {
      case (ifm3d::buffer_id::O3R_ODS_INFO):
        return create_and_read<ODSInfoV1>(data, size);
        case (ifm3d::buffer_id::TOF_INFO): {
          if (TOFInfoV4::IsValid(data, size))
            {
              return create_and_read<TOFInfoV4>(data, size);
            }

          if (TOFInfoV3::IsValid(data, size))
            {
              return create_and_read<TOFInfoV3>(data, size);
            }

          return std::monostate{};
        }
      case (ifm3d::buffer_id::RGB_INFO):
        return create_and_read<RGBInfoV1>(data, size);
      case (ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID):
        return create_and_read<ODSOccupancyGridV1>(data, size);
      case (ifm3d::buffer_id::O3R_ODS_POLAR_OCC_GRID):
        return create_and_read<ODSPolarOccupancyGridV1>(data, size);
      case (ifm3d::buffer_id::O3R_ODS_EXTRINSIC_CALIBRATION_CORRECTION):
        return create_and_read<ODSExtrinsicCalibrationCorrectionV1>(data,
                                                                    size);
      default:
        return std::monostate{};
      }
  }

  static VariantType
  deserialize(const Buffer& buffer)
  {
    return deserialize(buffer.Ptr<uint8_t>(0),
                       buffer.Size(),
                       buffer.BufferId());
  }
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_DESERIALIZE_H
