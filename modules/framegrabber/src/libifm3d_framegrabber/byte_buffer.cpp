/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ifm3d/fg/byte_buffer.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
#include <string>
#include <glog/logging.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/camera/err.h>

//-------------------------------------
// Utilities
//-------------------------------------

const std::size_t ifm3d::NUM_EXTRINSIC_PARAM = 6;
const std::size_t ifm3d::NUM_INTRINSIC_PARAM = 16;

std::size_t
ifm3d::get_chunk_index(const std::vector<std::uint8_t>& buff,
                       ifm3d::image_chunk chunk_type,
                       std::size_t start_idx)
{
  std::size_t idx = start_idx; // start of first chunk
  std::size_t size = buff.size();

  while (idx < size)
    {
      if (static_cast<std::uint32_t>(chunk_type) ==
          ifm3d::mkval<std::uint32_t>(buff.data()+idx))
        {
          return idx;
        }

      // move to the beginning of the next chunk
      std::uint32_t incr = ifm3d::mkval<std::uint32_t>(buff.data()+idx+4);
      if (incr <= 0)
        {
          LOG(WARNING) << "Next chunk is supposedly "
                       << incr << " bytes from the current one ... failing!";
          break;
        }
      idx += incr;
    }

  return std::numeric_limits<std::size_t>::max();
}

//------------------------------------------------------------
// NOTE: ByteBuffer<Derived> class is
// now implemented in: include/ifm3d/fg/detail/byte_buffer.hpp
//------------------------------------------------------------
