/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_ORGANIZER_H
#define IFM3D_FG_ORGANIZER_H

#include <cstdint>
#include <memory>
#include <vector>
#include <set>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/fg/frame_grabber_export.h>

namespace ifm3d
{
  /**
   *
   */
  class IFM3D_FRAME_GRABBER_EXPORT Organizer
  {
  public:
    struct Result
    {
      std::map<buffer_id, Buffer> images;
      std::vector<ifm3d::TimePointT> timestamps;
    };

    virtual ~Organizer() {}

    virtual Result Organize(const std::vector<uint8_t>& data,
                            const std::set<buffer_id>& requestedImages) = 0;
  }; // end: class Organizer

} // end: namespace ifm3d

#endif // IFM3D_FG_ORGANIZER_H