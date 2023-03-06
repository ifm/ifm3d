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

  /**
   * Organizer for O3X buffer
   */
  class O3XOrganizer : public Organizer
  {
  public:
    O3XOrganizer() = default;
    ~O3XOrganizer() = default;

    Result Organize(const std::vector<uint8_t>& data,
                    const std::set<buffer_id>& requestedImages,
                    const bool masking) override;

  private:
    void MaskImages(std::map<ifm3d::buffer_id, ifm3d::Buffer>& images,
                    ifm3d::Buffer& mask);

    bool ShouldMask(buffer_id id);
  }; // end: class DefaultOrganizer

} // end: namespace ifm3d

#endif // IFM3D_FG_DEFAULT_ORGANIZER_H