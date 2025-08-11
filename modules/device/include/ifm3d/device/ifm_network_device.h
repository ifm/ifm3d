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

    /** Ip Address of the device */
    [[nodiscard]] std::string GetIPAddress() const;

    /** Mac Address of the device */
    [[nodiscard]] std::string GetMACAddress() const;

    /** Netmask of the network of camera */
    [[nodiscard]] std::string GetNetmask() const;

    /** Gateway of the device*/
    [[nodiscard]] std::string GetGateway() const;

    /** Port on which device discovery is done*/
    [[nodiscard]] uint16_t GetPort() const;

    /** Device gives some additional information via those flags */
    [[nodiscard]] uint16_t GetFlag() const;

    /** Hostname of the device */
    [[nodiscard]] std::string GetHostName() const;

    /** Device name */
    [[nodiscard]] std::string GetDeviceName() const;

    /** Vendor ID of the device */
    [[nodiscard]] uint16_t GetVendorId() const;

    /** Device ID of the device */
    [[nodiscard]] uint16_t GetDeviceId() const;

    /** Founf via interface */
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
