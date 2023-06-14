/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_O3R_ORGANIZER_3D_H
#  define IFM3D_FG_O3D_ORGANIZER_3D_H

#  include <optional>
#  include <ifm3d/fg/organizer.h>

namespace ifm3d
{
  class DistanceImageInfo;
  /**
   * organizer for O3R 3D port
   */
  class O3ROrganizer3D : public Organizer
  {
  public:
    O3ROrganizer3D() = default;
    ~O3ROrganizer3D() = default;

    Result Organize(const std::vector<uint8_t>& data,
                    const std::set<buffer_id>& requestedImages,
                    const bool masking = false) override;

  private:
    bool ShouldMask(buffer_id id);
    std::map<ifm3d::buffer_id, ifm3d::Buffer> ExtractDistanceImageInfo(
      std::shared_ptr<DistanceImageInfo> distance_image_info,
      const std::optional<Buffer>& mask);

  }; // end: class O3ROrganizer

} // end: namespace ifm3d

#endif // IFM3D_FG_O3D_ORGANIZER_H