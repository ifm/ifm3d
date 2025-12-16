/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3c.h>
#include <ifm3d/device/o3r.h>
#include <string>

//================================================
// O3C class - the public interface
//================================================

ifm3d::O3C::O3C(const std::string& ip, const std::uint16_t xmlrpc_port)
  : O3R(ip, xmlrpc_port)
{}

ifm3d::O3C::~O3C() = default;

ifm3d::Device::DeviceFamily
ifm3d::O3C::WhoAmI()
{
  return DeviceFamily::O3C;
}

ifm3d::Device::SWUVersion
ifm3d::O3C::SwUpdateVersion()
{
  return O3R::SwUpdateVersion();
}
