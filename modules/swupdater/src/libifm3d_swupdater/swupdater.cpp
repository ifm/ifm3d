/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include <ifm3d/device/device.h>
#include <ifm3d/swupdater/swupdater.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <swupdater_impl.hpp>
#include <swupdater_v2_impl.hpp>

const std::uint16_t ifm3d::SWUPDATER_RECOVERY_PORT = 8080;

namespace
{
  const auto MAKE_SWU_IMPLEMENTOR =
    [](const ifm3d::Device::Ptr& cam,
       const ifm3d::SWUpdater::FlashStatusCb& cb,
       const std::uint16_t swupdate_recovery_port,
       std::optional<ifm3d::Device::swu_version> force_swu_version =
         std::nullopt) -> ifm3d::SWUpdater::Impl* {
    switch (force_swu_version.value_or(cam->SwUpdateVersion()))
      {
      case ifm3d::Device::swu_version::SWU_V1:
        return new ifm3d::SWUpdater::Impl(
          cam,
          cb,
          std::to_string(swupdate_recovery_port));
      case ifm3d::Device::swu_version::SWU_V2:
        return new ifm3d::ImplV2(cam,
                                 cb,
                                 std::to_string(swupdate_recovery_port));
        break;
      default:
        throw std::runtime_error("swupdate not supported");
        break;
      }
  };
}

ifm3d::SWUpdater::SWUpdater(
  const ifm3d::Device::Ptr& cam,
  const ifm3d::SWUpdater::FlashStatusCb& cb,
  const std::uint16_t swupdate_recovery_port,
  std::optional<ifm3d::Device::swu_version> force_swu_version)
  : _impl(
      MAKE_SWU_IMPLEMENTOR(cam, cb, swupdate_recovery_port, force_swu_version))
{}

ifm3d::SWUpdater::~SWUpdater() = default;

void
ifm3d::SWUpdater::RebootToRecovery()
{
  this->_impl->RebootToRecovery();
}

bool
ifm3d::SWUpdater::WaitForRecovery(long timeout_millis)
{
  return this->_impl->WaitForRecovery(timeout_millis);
}

void
ifm3d::SWUpdater::RebootToProductive()
{
  this->_impl->RebootToProductive();
}

bool
ifm3d::SWUpdater::WaitForProductive(long timeout_millis)
{
  return this->_impl->WaitForProductive(timeout_millis);
}

bool
ifm3d::SWUpdater::FlashFirmware(const std::string& swu_file,
                                long timeout_millis)
{
  return this->_impl->FlashFirmware(swu_file, timeout_millis);
}
