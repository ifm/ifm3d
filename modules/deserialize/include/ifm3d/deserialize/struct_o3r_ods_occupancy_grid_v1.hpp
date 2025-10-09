// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_O3R_ODS_OCCUPANCY_GRID_V1_HPP
#define IFM3D_DESERIALIZE_STRUCT_O3R_ODS_OCCUPANCY_GRID_V1_HPP

#include <array>
#include <cstddef>
#include <ifm3d/deserialize/deserialize_utils.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>

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

  /** @ingroup Deserialize */
  class ODSOccupancyGridV1
  {
  public:
    using Ptr = std::shared_ptr<ODSOccupancyGridV1>;

    bool
    IsValid(const uint8_t*, size_t size)
    {
      return size >= ODS_OCCUPANCY_GRID_V1_MINIMUM_SIZE;
    }

    void
    Read(const uint8_t* data, size_t size)
    {
      if (!IsValid(data, size))
        {
          throw ifm3d::Error(IFM3D_CORRUPTED_STRUCT);
        }
      const uint8_t* start_ptr = data;
      timestamp_ns = mkval<std::uint64_t>(
        start_ptr + ODS_OCCUPANCY_GRID_TIMESTAMP_NS_INDEX);
      width = mkval<std::uint32_t>(start_ptr + ODS_OCCUPANCY_GRID_WIDTH_INDEX);
      height =
        mkval<std::uint32_t>(start_ptr + ODS_OCCUPANCY_GRID_HEIGHT_INDEX);
      mkarray<float, 6>(
        start_ptr + ODS_OCCUPANCY_GRID_TRANSFORM_CELL_CENTER_TO_USER_INDEX,
        transform_cell_center_to_user);
      ods_occupancy_grid_v1_size = ODS_OCCUPANCY_GRID_V1_MINIMUM_SIZE +
                                   static_cast<size_t>(width) * height;
      if (!IsValid(data, size))
        {
          throw ifm3d::Error(IFM3D_CORRUPTED_STRUCT);
        }

      image = ifm3d::Buffer(height, width, 1, ifm3d::PixelFormat::FORMAT_8U);
      std::memcpy(image.Ptr<uint8_t>(0),
                  start_ptr + ODS_OCCUPANCY_GRID_IMAGE_INDEX,
                  image.Size());
    };
    /*@brief Timestamp of occupany grid in [ns]*/
    uint64_t timestamp_ns{};
    /*@brief Number of grid cells*/
    uint32_t width{};
    /*@brief number of grid cells*/
    uint32_t height{};
    /*@brief  Values of matrix 2x3
     * affine mapping between grid cell and user coordinate system
     * e.g, multiplying the matrix with [0,0,1] gives the user cordinate
     * of the center of upper left cell
     */
    std::array<float, 6> transform_cell_center_to_user{};
    /*@brief Buffer of width* height of type uint8_t */
    ifm3d::Buffer image;
    /*@brief size of ODS_OCCUPANCY_GRID in bytes*/
    size_t ods_occupancy_grid_v1_size{};

  private:
    static constexpr size_t ODS_OCCUPANCY_GRID_V1_MINIMUM_SIZE = 40;

  public:
    static ODSOccupancyGridV1
    Deserialize(const Buffer& ods_occupancy_buffer_grid)
    {
      ODSOccupancyGridV1 ods_occupancy_grid_v1;

      ods_occupancy_grid_v1.Read(ods_occupancy_buffer_grid.Ptr<uint8_t>(0),
                                 ods_occupancy_buffer_grid.Size());
      return ods_occupancy_grid_v1;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_O3R_ODS_OCCUPANCY_GRID_V1_HPP