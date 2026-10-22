// -*- c++ -*-
/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_RGB_INFO_V2_HPP
#define IFM3D_DESERIALIZE_STRUCT_RGB_INFO_V2_HPP

#include <ifm3d/deserialize/deserialize_utils.hpp>
#include <ifm3d/deserialize/struct_rgb_info_v1.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>

namespace ifm3d
{
  namespace
  {
    constexpr auto RGB_INFO_ROLLING_SHUTTER_ROW_OFFSET_NS_INDEX = 0x0134;
  }; // end anonymous namespace

  /** @ingroup Deserialize */
  class RGBInfoV2 : public RGBInfoV1
  {

  public:
    using Ptr = std::shared_ptr<RGBInfoV2>;

    static bool
    IsValid(const uint8_t* data, size_t size)
    {
      auto version = mkval<std::uint32_t>(data + RGB_INFO_VERSION_INDEX);

      return size >= RGB_INFO_V2_SIZE && version >= 1;
    }

    void
    Read(const uint8_t* data, size_t size)
    {
      if (!IsValid(data, size))
        {
          throw ifm3d::Error(IFM3D_CORRUPTED_STRUCT);
        }
      RGBInfoV1::Read(data, size);
      const uint8_t* start_ptr = data;
      rolling_shutter_row_offset_ns = mkval<std::uint32_t>(
        start_ptr + RGB_INFO_ROLLING_SHUTTER_ROW_OFFSET_NS_INDEX);
    };

    /** @brief Rolling-shutter offset between consecutive row timestamps, given
     * in [nanoseconds]. The exposure-center timestamp of row y is
     * t_row(y) = timestamp_ns + (2 * y - (height - 1)) *
     * rolling_shutter_row_offset_ns / 2
     **/
    std::uint32_t rolling_shutter_row_offset_ns{};
    static constexpr size_t RGB_INFO_V2_SIZE = 312;

    static RGBInfoV2
    Deserialize(const Buffer& rgb_info_buffer)
    {
      RGBInfoV2 rgb_info_v2{};

      rgb_info_v2.Read(rgb_info_buffer.Ptr<uint8_t>(0),
                       rgb_info_buffer.Size());
      return rgb_info_v2;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_RGB_INFO_V2_HPP