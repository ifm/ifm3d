/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_O3C_H
#define IFM3D_DEVICE_O3C_H

#include <ifm3d/device/device.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/pcic_command.h>
#include <sstream>

namespace ifm3d
{
  static const int NET_WAIT_O3C_SET =
    std::getenv("IFM3D_NET_WAIT_O3C_SET") == nullptr ?
      15000 :
      std::stoi(std::getenv("IFM3D_NET_WAIT_O3C_SET"));

  /** @ingroup Device
   *
   * Device specialization for O3C
   */
  class IFM3D_EXPORT O3C : public O3R
  {
  public:
    using Ptr = std::shared_ptr<O3C>;
    O3C(const std::string& ip = ifm3d::DEFAULT_IP,
        std::uint16_t xmlrpc_port = ifm3d::DEFAULT_XMLRPC_PORT);

    ~O3C() override;
    O3C(O3C&&) = delete;
    O3C& operator=(O3C&&) = delete;
    O3C(O3C&) = delete;
    O3C& operator=(O3C&) = delete;

    DeviceFamily WhoAmI() override;
    ifm3d::Device::SWUVersion SwUpdateVersion() override;

    using SetTemporaryApplicationParameter =
      O3R::SetTemporaryApplicationParameter;
  };
}

#endif // IFM3D_DEVICE_O3C_H