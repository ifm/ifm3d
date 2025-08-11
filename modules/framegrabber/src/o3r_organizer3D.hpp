/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_O3R_ORGANIZER_3D_H
#define IFM3D_FG_O3R_ORGANIZER_3D_H

#include <ifm3d/fg/organizer.h>
#include <optional>

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
    O3ROrganizer3D(const O3ROrganizer3D&) = default;
    O3ROrganizer3D(O3ROrganizer3D&&) = delete;
    O3ROrganizer3D& operator=(const O3ROrganizer3D&) = default;
    O3ROrganizer3D& operator=(O3ROrganizer3D&&) = delete;
    ~O3ROrganizer3D() override = default;

    Result Organize(const std::vector<uint8_t>& data,
                    const std::set<buffer_id>& requested_images,
                    bool masking = false) override;

  private:
    bool should_mask(buffer_id id);
    std::map<ifm3d::buffer_id, ifm3d::Buffer> extract_distance_image_info(
      const std::shared_ptr<DistanceImageInfo>& distance_image_info,
      const std::optional<Buffer>& mask);

  }; // end: class O3ROrganizer

} // end: namespace ifm3d

#endif // IFM3D_FG_O3R_ORGANIZER_3D_H