/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_DEFAULT_ORGANIZER_H
#define IFM3D_FG_DEFAULT_ORGANIZER_H

#include <ifm3d/fg/organizer.h>
#include <optional>

namespace ifm3d
{
  class DistanceImageInfo;
  /**
   *
   */
  class DefaultOrganizer : public Organizer
  {
  public:
    DefaultOrganizer() = default;
    DefaultOrganizer(const DefaultOrganizer&) = default;
    DefaultOrganizer(DefaultOrganizer&&) = delete;
    DefaultOrganizer& operator=(const DefaultOrganizer&) = default;
    DefaultOrganizer& operator=(DefaultOrganizer&&) = delete;
    ~DefaultOrganizer() override = default;

    Result Organize(const std::vector<uint8_t>& data,
                    const std::set<buffer_id>& requested_images,
                    bool masking = true) override;

  private:
    Buffer create_pixel_mask(Buffer& confidence);
    std::map<buffer_id, Buffer> extract_distance_image_info(
      std::shared_ptr<DistanceImageInfo> distance_image_info,
      const std::optional<Buffer>& mask);
    bool should_mask(buffer_id id);

  }; // end: class DefaultOrganizer

} // end: namespace ifm3d

#endif // IFM3D_FG_DEFAULT_ORGANIZER_H