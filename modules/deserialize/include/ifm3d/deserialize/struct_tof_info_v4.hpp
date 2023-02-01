// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_TOF_INFO_V4_HPP
#define IFM3D_DESERIALIZE_STRUCT_TOF_INFO_V4_HPP

#include <array>
#include <chrono>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/deserialize/deserialize_utils.hpp>
#include <ifm3d/deserialize/struct_tof_info_v3.hpp>

namespace ifm3d
{
  namespace
  {
    constexpr auto TOF_INFO_MEASUREMENT_BLOCK_INDEX = 0x01A0;
    constexpr auto TOF_INFO_MEASUREMENT_RANGE_MIN_INDEX = 0x01A4;
    constexpr auto TOF_INFO_MEASUREMENT_RANGE_MAX_INDEX = 0x01A8;
  };

  class TOFInfoV4 : public TOFInfoV3
  {
  public:
    using Ptr = std::shared_ptr<TOFInfoV4>;

    void
    Read(const uint8_t* data, size_t size)
    {
      if (size < tof_info_v4_size)
        {
          throw ifm3d::Error(IFM3D_CORRUPTED_STRUCT);
        }

      TOFInfoV3::Read(data, size);
      const uint8_t* start_ptr = data;
      measurement_block_index =
        mkval<std::uint32_t>(start_ptr + TOF_INFO_MEASUREMENT_BLOCK_INDEX);
      measurement_range_min =
        mkval<float>(start_ptr + TOF_INFO_MEASUREMENT_RANGE_MIN_INDEX);
      measurement_range_max =
        mkval<float>(start_ptr + TOF_INFO_MEASUREMENT_RANGE_MAX_INDEX);
    };
    /**
     * @brief Current measurement block index (range 0 to N-1, where N is the
     * number of sub-modes). This identifies the currently used sub-mode in
     * cyclic modes. In non-cyclic modes this value is always 0.
     */
    uint32_t measurement_block_index;
    /*
     * @brief Current minimum measurement range [m].
     * The value is based on the camera-individual ToF calibration.
     * It is influenced by temperature.
     **/
    float measurement_range_min;
    /*
     * @brief Current maximum measurement range [m].
     * The value is based on the camera-individual ToF calibration.
     * It is influenced by temperature.
     **/
    float measurement_range_max;
    /**
     * @brief size of ToFInfoV4 in bytes
     */
    const size_t tof_info_v4_size = 428;

    static TOFInfoV4
    Deserialize(const Buffer& tof_info_buffer)
    {
      TOFInfoV4 tof_info_v4;

      tof_info_v4.Read(tof_info_buffer.ptr<uint8_t>(0),
                       tof_info_buffer.size());
      return tof_info_v4;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_TOF_INFO_V3_HPP