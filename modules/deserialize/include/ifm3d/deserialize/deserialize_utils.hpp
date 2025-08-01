/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_UTILS_HPP
#define IFM3D_DESERIALIZE_UTILS_HPP

#include <cstdint>
#include <array>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>

namespace ifm3d
{

  template <typename T, std::size_t n>
  void
  mkarray(const std::uint8_t* data, std::array<T, n>& arr)
  {
    int element_index = 0;
    for (auto& val : arr)
      {
        val = mkval<T>(data + sizeof(T) * element_index);
        element_index++;
      }
  }

  template <typename T, std::size_t num_of_parameter>
  class ArrayDeserialize
  {
  public:
    void
    Read(const std::uint8_t* data, std::size_t size)
    {
      if (size < (num_of_parameter * sizeof(T)))
        {
          throw ifm3d::Error(IFM3D_CORRUPTED_STRUCT);
        }
      const std::uint8_t* start_ptr = data;
      mkarray<T, num_of_parameter>(start_ptr, this->data);
    };

    /**
     * @brief array to hold deserialize values
     */
    std::array<T, num_of_parameter> data;

    static auto
    Deserialize(const Buffer& o3d_buffer)
    {
      ArrayDeserialize<T, num_of_parameter> data;

      data.Read(o3d_buffer.ptr<uint8_t>(0), o3d_buffer.size());
      return data;
    }
  };

} // end namespace

#endif // IFM3D_DESERIALIZE_UTILS_HPP