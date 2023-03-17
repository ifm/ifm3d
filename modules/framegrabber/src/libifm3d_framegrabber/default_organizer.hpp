/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_DEFAULT_ORGANIZER_H
#define IFM3D_FG_DEFAULT_ORGANIZER_H

#include <optional>
#include <ifm3d/fg/organizer.h>

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
    ~DefaultOrganizer() = default;

    Result Organize(const std::vector<uint8_t>& data,
                    const std::set<buffer_id>& requestedImages,
                    const bool masking = true) override;

  private:
    Buffer CreatePixelMask(Buffer& confidence);
    std::map<buffer_id, Buffer> ExtractDistanceImageInfo(
      std::shared_ptr<DistanceImageInfo> distance_image_info,
      const std::optional<Buffer>& mask);
    bool ShouldMask(buffer_id id);

  }; // end: class DefaultOrganizer

} // end: namespace ifm3d

#endif // IFM3D_FG_DEFAULT_ORGANIZER_H