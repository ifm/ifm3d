// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_O3R_ODS_OCCUPANCY_GRID_V1_HPP
#define IFM3D_DESERIALIZE_STRUCT_O3R_ODS_OCCUPANCY_GRID_V1_HPP

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
    constexpr auto ODS_OCCUPANCY_GRID_TIMESTAMP_NS_INDEX = 0x0000;
    constexpr auto ODS_OCCUPANCY_GRID_WIDTH_INDEX = 0x0008;
    constexpr auto ODS_OCCUPANCY_GRID_HEIGHT_INDEX = 0x000C;
    constexpr auto ODS_OCCUPANCY_GRID_TRANSFORM_CELL_CENTER_TO_USER_INDEX =
      0x0010;
    constexpr auto ODS_OCCUPANCY_GRID_IMAGE_INDEX = 0x0028;
  };

  class ODSOccupancyGridV1
  {
  public:
    using Ptr = std::shared_ptr<ODSOccupancyGridV1>;

    void
    Read(const uint8_t* data, size_t size)
    {
      if (size < ods_occupancy_grid_v1_minimum_size)
        {
          throw ifm3d::Error(IFM3D_BUFFER_NOT_COMPATIABLE);
        }
      const uint8_t* start_ptr = data;
      timestamp_ns = mkval<std::uint64_t>(
        start_ptr + ODS_OCCUPANCY_GRID_TIMESTAMP_NS_INDEX);
      width = mkval<std::uint32_t>(start_ptr + ODS_OCCUPANCY_GRID_WIDTH_INDEX);
      height =
        mkval<std::uint32_t>(start_ptr + ODS_OCCUPANCY_GRID_HEIGHT_INDEX);
      mkarray<float, 6>(
        start_ptr + ODS_OCCUPANCY_GRID_TRANSFORM_CELL_CENTER_TO_USER_INDEX,
        transfor_cell_center_to_user);
      ods_occupancy_grid_v1_size =
        ods_occupancy_grid_v1_minimum_size + width * height;
      if (size < ods_occupancy_grid_v1_size;)
        {
          throw ifm3d::Error(IFM3D_BUFFER_NOT_COMPATIABLE);
        }

      image = ifm3d::Buffer(height, width, 1, ifm3d::pixel_format::FORMAT_8U);
      std::memcpy(image.ptr<uint8_t>(0),
                  start_ptr + ODS_OCCUPANCY_GRID_IMAGE_INDEX,
                  image.size());
    };

    uint64_t timestamp_ns;
    uint32_t width;
    uint32_t height;
    std::array<float, 6> transfor_cell_center_to_user;
    ifm3d::Buffer image;
    size_t ods_occupancy_grid_v1_size;

  private:
    const size_t ods_occupancy_grid_v1_minimum_size = 40;

    static ODSOccupancyGridV1
    Deserialize(const Buffer& ods_occupancy_buffer_grid)
    {
      ODSOccupancyGridV1 ods_occupancy_grid_v1;

      ods_occupancy_grid_v1.Read(ods_occupancy_buffer_grid.ptr<uint8_t>(0),
                                 ods_occupancy_buffer_grid.size());
      return ods_occupancy_grid_v1;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_O3R_ODS_OCCUPANCY_GRID_V1_HPP