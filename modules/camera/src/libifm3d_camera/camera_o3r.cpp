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

json
ifm3d::O3RCamera::Get(const std::vector<std::string>& path)
{
  return json::parse(this->pImpl->Get(path));
}

void
ifm3d::O3RCamera::Set(const json& j)
{
  return this->pImpl->Set(j.dump());
}

json
ifm3d::O3RCamera::GetInit()
{
  return this->pImpl->GetInit();
}

void
ifm3d::O3RCamera::SaveInit()
{
  return this->pImpl->SaveInit();
}

std::string
ifm3d::O3RCamera::GetInitStatus()
{
  return this->pImpl->GetInitStatus();
}

json
ifm3d::O3RCamera::GetSchema()
{
  return json::parse(this->pImpl->GetSchema());
}

void
ifm3d::O3RCamera::Lock(const std::string& password)
{
  return this->pImpl->Lock(password);
}

void
ifm3d::O3RCamera::Unlock(const std::string& password)
{
  return this->pImpl->Unlock(password);
}

ifm3d::CameraBase::device_family
ifm3d::O3RCamera::WhoAmI()
{
  return device_family::O3R;
}

void
ifm3d::O3RCamera::FromJSON(const json& j)
{
  this->Set(j);
  this->SaveInit();
}

json
ifm3d::O3RCamera::ToJSON()
{
  return this->Get();
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
