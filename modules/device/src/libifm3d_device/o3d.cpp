/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include "ifm3d/device/legacy_device.h"
#include "ifm3d/device/device.h"
#include "ifm3d/common/json_impl.hpp"
#include <ifm3d/device/o3d.h>
#include <string>
#include <unordered_map>

//================================================
// O3D class - the public interface
//================================================

ifm3d::O3D::O3D(const std::string& ip,
                const std::uint16_t xmlrpc_port,
                const std::string& password)
  : ifm3d::LegacyDevice::LegacyDevice(ip, xmlrpc_port, password)
{}

ifm3d::O3D::~O3D() = default;

std::unordered_map<std::string, std::string>
ifm3d::O3D::TimeInfo()
{
  if (this->CheckMinimumFirmwareVersion(ifm3d::O3D_TIME_SUPPORT_MAJOR,
                                        ifm3d::O3D_TIME_SUPPORT_MINOR,
                                        ifm3d::O3D_TIME_SUPPORT_PATCH))
    {
      return ifm3d::LegacyDevice::TimeInfo();
    }
  return json::parse("{}");
}

ifm3d::Device::device_family
ifm3d::O3D::WhoAmI()
{
  return device_family::O3D;
}