/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/camera/camera_o3r.h>
#include <camera_o3r_impl.hpp>

//================================================
// O3RCamera class - the public interface
//================================================

ifm3d::O3RCamera::O3RCamera(const std::string& ip,
                            const std::uint16_t xmlrpc_port)
  : ifm3d::CameraBase::CameraBase(ip, xmlrpc_port),
    pImpl(new ifm3d::O3RCamera::Impl(XWrapper()))
{}

ifm3d::O3RCamera::~O3RCamera() = default;

void
ifm3d::O3RCamera::FactoryReset(bool keepNetworkSettings)
{
  this->pImpl->FactoryReset(keepNetworkSettings);
}

ifm3d::CameraBase::device_family
ifm3d::O3RCamera::WhoAmI()
{
  return device_family::O3R;
}

void
ifm3d::O3RCamera::FromJSON(const json& j)
{
  this->pImpl->SetTemporaryConfiguration(j.dump());
  this->pImpl->SaveInitConfiguration();
}

json
ifm3d::O3RCamera::ToJSON()
{
  return json::parse(this->pImpl->GetTemporaryConfiguration());
}

void
ifm3d::O3RCamera::Reboot(const boot_mode& mode)
{
  this->pImpl->Reboot();
}

ifm3d::CameraBase::swu_version
ifm3d::O3RCamera::SwUpdateVersion()
{
  return ifm3d::CameraBase::swu_version::SWU_V2;
}

json
ifm3d::O3RCamera::Schema()
{
  return this->pImpl->Schema();
}
