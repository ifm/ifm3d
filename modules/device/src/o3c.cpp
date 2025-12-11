/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/device/o3c.h>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

//================================================
// O3C class - the public interface
//================================================

ifm3d::O3C::O3C(const std::string& ip, const std::uint16_t xmlrpc_port)
  : O3R(ip, xmlrpc_port)
{}

ifm3d::O3C::~O3C() = default;

void
ifm3d::O3C::FactoryReset(bool keep_network_settings)
{
  O3R::FactoryReset(keep_network_settings);
}

void
ifm3d::O3C::Reboot(const BootMode& mode)
{
  O3R::Reboot(mode);
}

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

ifm3d::json
ifm3d::O3C::ToJSON()
{
  return O3R::ToJSON();
}

void
ifm3d::O3C::FromJSON(const json& j)
{
  O3R::FromJSON(j);
}
