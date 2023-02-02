/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DESERIALIZE_UTILS_HPP
#define IFM3D_DESERIALIZE_UTILS_HPP

namespace ifm3d
{

  template <typename T, size_t n>
  void
  mkarray(const uint8_t* data, std::array<T, n>& arr)
  {
    int element_index = 0;
    for (auto& val : arr)
      {
        val = mkval<T>(data + sizeof(T) * element_index);
        element_index++;
      }
  }
} // end namespace

#endif // IFM3D_DESERIALIZE_UTILS_HPP