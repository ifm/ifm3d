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
#include <ifm3d/fg/image.h>
#include <ifm3d/fg/frame.h>

namespace ifm3d
{
  /**
   * ImageIds available for use with the default Organizer.
   * Additionally to the list below any image_chunk can be passed to the
   * default Organizer to receive the raw chunk data
   */
  enum class image_id : uint64_t
  {
    IMAGES_START = std::numeric_limits<std::uint32_t>::max(),
    XYZ, // The point cloud encoded as a 3 channel XYZ image
  };

  /**
   *
   */
  class Organizer
  {
  public:
    struct Result
    {
      std::map<ImageId, Image> images;
      std::vector<ifm3d::TimePointT> timestamps;
    };

    virtual ~Organizer() {}

    virtual Result Organize(const std::vector<uint8_t>& data,
                            const std::set<ImageId>& requestedImages) = 0;

    virtual std::set<image_chunk> GetImageChunks(ImageId id) = 0;
  }; // end: class Organizer

} // end: namespace ifm3d

#endif // IFM3D_FG_ORGANIZER_H