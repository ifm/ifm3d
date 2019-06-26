/*
 * Copyright (C) 2019 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
ifm3d::SWUpdater::FlashFirmware(
    const std::vector<std::uint8_t>& bytes,
    long timeout_millis)
{
  return this->pImpl->FlashFirmware(bytes, timeout_millis);
}
