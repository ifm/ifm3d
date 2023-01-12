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

  class TofInfoV4 : public TofInfoV3
  {
  public:
    using Ptr = std::shared_ptr<TofInfoV4>;

    void
    Read(const uint8_t* data, size_t size)
    {
      if (size < tof_info_v4_size)
        {
          throw ifm3d::Error(IFM3D_BUFFER_NOT_COMPATIABLE);
        }

      TofInfoV3::Read(data, size);
      const uint8_t* start_ptr = data;
      measurement_block_index =
        mkval<std::uint32_t>(start_ptr + TOF_INFO_MEASUREMENT_BLOCK_INDEX);
      measurement_range_min =
        mkval<float>(start_ptr + TOF_INFO_MEASUREMENT_RANGE_MIN_INDEX);
      measurement_range_max =
        mkval<float>(start_ptr + TOF_INFO_MEASUREMENT_RANGE_MAX_INDEX);
    };

    uint32_t measurement_block_index;
    float measurement_range_min;
    float measurement_range_max;
    const size_t tof_info_v4_size = 428;

    static TofInfoV4
    Deserialize(const Buffer& tof_info_buffer)
    {
      TofInfoV4 tof_info_v4;

      tof_info_v4.Read(tof_info_buffer.ptr<uint8_t>(0),
                       tof_info_buffer.size());
      return tof_info_v4;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_TOF_INFO_V3_HPP