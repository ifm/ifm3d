// -*- c++ -*-
/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_STRUCT_O3R_ODS_POLAR_OCCUPANCY_GRID_V1_HPP
#define IFM3D_DESERIALIZE_STRUCT_O3R_ODS_POLAR_OCCUPANCY_GRID_V1_HPP

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
    constexpr auto ODS_POLAR_OCCUPANCY_GRID_VERSION_INDEX = 0x0000;
    constexpr auto ODS_POLAR_OCCUPANCY_GRID_POLAR_OCC_GRID_INDEX = 0x0004;
    constexpr auto ARRAY_SIZE = 675;
    constexpr auto ODS_POLAR_OCCUPANCY_GRID_TIMESTAMP_NS_INDEX = 0x054A;
  };

  /** @ingroup Deserialize */
  class ODSPolarOccupancyGridV1
  {
  public:
    using Ptr = std::shared_ptr<ODSPolarOccupancyGridV1>;

    void
    Read(const uint8_t* start_ptr, size_t size)
    {
      if (size < ods_polar_occupancy_grid_v1_minimum_size)
        {
          throw ifm3d::Error(IFM3D_CORRUPTED_STRUCT);
        }

      version = mkval<std::uint32_t>(start_ptr +
                                     ODS_POLAR_OCCUPANCY_GRID_VERSION_INDEX);
      mkarray<uint16_t, ARRAY_SIZE>(
        start_ptr + ODS_POLAR_OCCUPANCY_GRID_POLAR_OCC_GRID_INDEX,
        polarOccGrid);
      timestamp_ns = mkval<std::uint64_t>(
        start_ptr + ODS_POLAR_OCCUPANCY_GRID_TIMESTAMP_NS_INDEX);
    };

    /*@brief Version number(current 1) of polar occupancy grid*/
    uint32_t version;
    /* @brief A compressed version of the grid using polar coordinates.
     * The index corresponds to the angle slice i*360/675 degree to
     * (i+1)*360/675 degree. The value is the distance to the nearest occupied
     * cell on the ray from the vehicle origin, given in [mm]. In case there
     * are no occupied cells on the ray, the value 65535 is set.
     */
    std::array<uint16_t, ARRAY_SIZE> polarOccGrid;
    /*@brief Timestamp of polar occupany grid in [ns]*/
    uint64_t timestamp_ns;

  private:
    static constexpr size_t ods_polar_occupancy_grid_v1_minimum_size = 1354;

  public:
    static ODSPolarOccupancyGridV1
    Deserialize(const Buffer& ods_polar_occupancy_buffer_grid)
    {
      ODSPolarOccupancyGridV1 ods_polar_occupancy_grid_v1;

      ods_polar_occupancy_grid_v1.Read(
        ods_polar_occupancy_buffer_grid.ptr<uint8_t>(0),
        ods_polar_occupancy_buffer_grid.size());
      return ods_polar_occupancy_grid_v1;
    }
  };
} // end namespace ifm3d

#endif // IFM3D_DESERIALIZE_STRUCT_O3R_ODS_POLAR_OCCUPANCY_GRID_V1_HPP