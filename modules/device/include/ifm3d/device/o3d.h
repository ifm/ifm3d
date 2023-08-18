/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_O3D_H
#define IFM3D_DEVICE_O3D_H

#include <ifm3d/device/legacy_device.h>

namespace ifm3d
{
  /** @ingroup Device
   *
   * Device specialization for O3D
   *
   * Note that O3D support is currently experimental- Use at your own risk!.
   */
  class IFM3D_DEVICE_EXPORT O3D : public LegacyDevice
  {
  public:
    using Ptr = std::shared_ptr<O3D>;
    O3D(const std::string& ip = ifm3d::DEFAULT_IP,
        const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
        const std::string& password = ifm3d::DEFAULT_PASSWORD);

    virtual ~O3D();
    O3D(O3D&&) = delete;
    O3D& operator=(O3D&&) = delete;
    O3D(O3D&) = delete;
    O3D& operator=(O3D&) = delete;

    std::unordered_map<std::string, std::string> TimeInfo() override;
    device_family WhoAmI() override;
  }; // end: class O3D
}
#endif // IFM3D_DEVICE_O3D_H