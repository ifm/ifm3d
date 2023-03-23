/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_O3R_ORGANIZER_2D_H
#  define IFM3D_FG_O3D_ORGANIZER_2D_H

#  include <optional>
#  include <ifm3d/fg/organizer.h>

namespace ifm3d
{

  /**
   * organizer for O3R 2D port
   */
  class O3ROrganizer2D : public Organizer2D
  {
  public:
    O3ROrganizer2D() = default;
    ~O3ROrganizer2D() = default;

    Result Organize(const std::vector<uint8_t>& data,
                    const std::set<buffer_id>& requestedImages,
                    const bool masking = false) override;

  }; // end: class O3ROrganizer

} // end: namespace ifm3d

#endif // IFM3D_FG_O3D_ORGANIZER_H