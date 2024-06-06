// -*- c++ -*-
/*
 * Copyright 2020-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_IFM_NETWORK_DEVICE_H
#define IFM3D_IFM_NETWORK_DEVICE_H

#include <memory>
#include <string>
#include <vector>
#include <ifm3d/device/module_device.h>

namespace ifm3d
{
  using Data = std::vector<unsigned char>;
  class IFM3D_EXPORT IFMNetworkDevice
  {
  public:
    IFMNetworkDevice(Data& data, const std::string& ip_address_via_interface);

    /** Ip Address of the device */
    std::string GetIPAddress() const;

    /** Mac Address of the device */
    std::string GetMACAddress() const;

    /** Netmask of the network of camera */
    std::string GetNetmask() const;

    /** Gateway of the device*/
    std::string GetGateway() const;

    /** Port on which device discovery is done*/
    uint16_t GetPort() const;

    /** Device gives some additional information via those flags */
    uint16_t GetFlag() const;

    /** Hostname of the device */
    std::string GetHostName() const;

    /** Device name */
    std::string GetDeviceName() const;

    /** Vendor ID of the device */
    uint16_t GetVendorId() const;

    /** Device ID of the device */
    uint16_t GetDeviceId() const;

    /** Founf via interface */
    std::string GetFoundVia() const;

  private:
    std::string ip_address_;
    std::string mac_;
    std::string subnet_;
    std::string gateway_;
    uint16_t port_;
    uint16_t flags_;
    std::string hostname_;
    std::string device_name_;
    uint16_t vendor_id_;
    uint16_t device_id_;
    std::string found_via_;
  };

} // end: namespace ifm3d

#endif // IFM3D_IFM_NETWORK_DEVICE_H
