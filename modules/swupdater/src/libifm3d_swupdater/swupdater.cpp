/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/swupdater/swupdater.h>
#include <vector>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/err.h>
#include <swupdater_impl.hpp>

ifm3d::SWUpdater::SWUpdater(ifm3d::Camera::Ptr cam,
                            const ifm3d::SWUpdater::FlashStatusCb& cb)
  : pImpl(new ifm3d::SWUpdater::Impl(cam, cb))
{ }

ifm3d::SWUpdater::~SWUpdater() = default;

void
ifm3d::SWUpdater::RebootToRecovery()
{
  this->pImpl->RebootToRecovery();
}

bool
ifm3d::SWUpdater::WaitForRecovery(long timeout_millis)
{
  return this->pImpl->WaitForRecovery(timeout_millis);
}

void
ifm3d::SWUpdater::RebootToProductive()
{
  this->pImpl->RebootToProductive();
}

bool
ifm3d::SWUpdater::WaitForProductive(long timeout_millis)
{
  return this->pImpl->WaitForProductive(timeout_millis);
}

bool
ifm3d::SWUpdater::FlashFirmware(const std::vector<std::uint8_t>& bytes,
                                long timeout_millis)
{
  return this->pImpl->FlashFirmware(bytes, timeout_millis);
}
