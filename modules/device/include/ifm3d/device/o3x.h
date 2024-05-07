/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_O3X_H
#define IFM3D_DEVICE_O3X_H

#include <ifm3d/device/legacy_device.h>

namespace ifm3d
{
  /** @ingroup Device
   *
   * Device specialization for O3X
   *
   * Note that O3X support is currently experimental- Use at your own risk!.
   */
  class IFM3D_DEVICE_EXPORT O3X : public LegacyDevice
  {
  public:
    using Ptr = std::shared_ptr<O3X>;
    O3X(const std::string& ip = ifm3d::DEFAULT_IP,
        const std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT,
        const std::string& password = ifm3d::DEFAULT_PASSWORD);

    virtual ~O3X();
    O3X(O3X&&) = delete;
    O3X& operator=(O3X&&) = delete;
    O3X(O3X&) = delete;
    O3X& operator=(O3X&) = delete;

    device_family WhoAmI() override;
  }; // end: class O3X
}
#endif // IFM3D_DEVICE_O3X_H