// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_DESERIALIZE_DESERIALIZE_H
#define IFM3D_DESERIALIZE_DESERIALIZE_H

#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/organizer_utils.h>
#include <ifm3d/deserialize/struct_tof_info_v3.hpp>

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
} // end: namespace ifm3d

#endif // IFM3D_DESERIALIZE_DESERIALIZE_H
