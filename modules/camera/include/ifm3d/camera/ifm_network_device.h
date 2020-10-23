// -*- c++ -*-
/*
 * Copyright 2020-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_IFM_NETWORK_DEVICE_H__
#define __IFM3D_IFM_NETWORK_DEVICE_H__

#include <memory>
#include <string>
#include <vector>

namespace ifm3d
{
  using Data = std::vector<unsigned char>;
  class IFMNetworkDevice
  {
  public:
    IFMNetworkDevice(Data& data, const std::string& ip_address_via_interface);

    /** @brief Ip Address of the device */
    std::string GetIPAddress() const;

    /** @brief Mac Address of the device */
    std::string GetMACAddress() const;

    /** @brief Netmask of the network of camera */
    std::string GetNetmask() const;

    /** @brief Gateway of the device*/
    std::string GetGateway() const;

    /** @brief Port on which device discovery is done*/
    uint16_t GetPort() const;

    /** @brief the device gives some additional information via those flags */
    uint16_t GetFlag() const;

    /** @brief hostmake of the device */
    std::string GetHostName() const;

    /** @brief Device name */
    std::string GetDeviceName() const;

    /** @brief Vendor ID of the device */
    uint16_t GetVendorId() const;

    /** @brief Device ID of the device */
    uint16_t GetDeviceId() const;

    /** @brief founf via interface */
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

#endif // __IFM3D_IFM_NETWORK_DEVICE_H__
