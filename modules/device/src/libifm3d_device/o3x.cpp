/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include "ifm3d/device/legacy_device.h"
#include "ifm3d/device/device.h"
#include <ifm3d/device/o3x.h>
#include <string>

//================================================
// O3XCamera class - the public interface
//================================================

ifm3d::O3X::O3X(const std::string& ip,
                const std::uint16_t xmlrpc_port,
                const std::string& password)
  : ifm3d::LegacyDevice::LegacyDevice(ip, xmlrpc_port, password)
{}

ifm3d::O3X::~O3X() = default;

ifm3d::Device::device_family
ifm3d::O3X::WhoAmI()
{
  return device_family::O3X;
}