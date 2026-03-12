// -*- c++ -*-
/*
 * Copyright 2020-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_IFM_NETWORK_DEVICE_H
#define IFM3D_IFM_NETWORK_DEVICE_H

#include <cstdint>
#include <ifm3d/device/module_device.h>
#include <string>
#include <vector>

namespace ifm3d
{
  using Data = std::vector<unsigned char>;
  class IFM3D_EXPORT IFMNetworkDevice
  {
  public:
    IFMNetworkDevice(Data& data, std::string ip_address_via_interface);

    /// Returns the IP address of the device.
    [[nodiscard]] std::string GetIPAddress() const;

    /// Returns the MAC address of the device.
    [[nodiscard]] std::string GetMACAddress() const;

    /// Returns the network subnet mask.
    [[nodiscard]] std::string GetNetmask() const;

    /// Returns the default gateway of the device.
    [[nodiscard]] std::string GetGateway() const;

    /// Returns the port number used for device discovery.
    [[nodiscard]] uint16_t GetPort() const;

    /// Returns flags providing additional device information.
    [[nodiscard]] uint16_t GetFlags() const;

    /// Checks if a specific flag is set in the device flags.
    [[nodiscard]] bool HasFlag(uint16_t flag) const;

    /// Returns the hostname of the device.
    [[nodiscard]] std::string GetHostName() const;

    /// Returns the device name.
    [[nodiscard]] std::string GetDeviceName() const;

    /// Returns the vendor ID of the device.
    [[nodiscard]] uint16_t GetVendorId() const;

    /// Returns the device ID of the device.
    [[nodiscard]] uint16_t GetDeviceId() const;

    /// Returns the network interface through which the device was discovered.
    [[nodiscard]] std::string GetFoundVia() const;

  private:
    std::string _ip_address;
    std::string _mac;
    std::string _subnet;
    std::string _gateway;
    uint16_t _port;
    uint16_t _flags;
    std::string _hostname;
    std::string _device_name;
    uint16_t _vendor_id;
    uint16_t _device_id;
    std::string _found_via;
  };

} // end: namespace ifm3d

#endif // IFM3D_IFM_NETWORK_DEVICE_H
