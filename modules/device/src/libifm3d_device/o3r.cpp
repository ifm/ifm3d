/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include "ifm3d/device/device.h"
#include "ifm3d/common/json_impl.hpp"
#include <ifm3d/device/o3r.h>
#include <memory>
#include <o3r_impl.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

//================================================
// O3R class - the public interface
//================================================

ifm3d::O3R::O3R(const std::string& ip, const std::uint16_t xmlrpc_port)
  : ifm3d::Device::Device(ip, xmlrpc_port),
    pImpl(new ifm3d::O3R::Impl(XWrapper()))
{}

ifm3d::O3R::~O3R() = default;

void
ifm3d::O3R::FactoryReset(bool keep_network_settings)
{
  this->pImpl->FactoryReset(keep_network_settings);
}

ifm3d::json
ifm3d::O3R::Get(const std::vector<std::string>& path)
{
  return this->pImpl->Get(path);
}

ifm3d::json
ifm3d::O3R::ResolveConfig(const json::json_pointer& ptr)
{
  return this->pImpl->ResolveConfig(ptr);
}

void
ifm3d::O3R::Set(const json& j)
{
  this->pImpl->Set(j.dump());
}

void
ifm3d::O3R::Remove(const std::string& json_pointer)
{
  this->pImpl->Remove(json_pointer);
}

void
ifm3d::O3R::Reset(const std::string& json_pointer)
{
  this->pImpl->Reset(json_pointer);
}

ifm3d::json
ifm3d::O3R::GetInit()
{
  return this->pImpl->GetInit();
}

void
ifm3d::O3R::SaveInit(const std::vector<std::string>& pointers)
{
  this->pImpl->SaveInit(pointers);
}

std::string
ifm3d::O3R::GetInitStatus()
{
  return this->pImpl->GetInitStatus();
}

ifm3d::json
ifm3d::O3R::GetSchema()
{
  return json::parse(this->pImpl->GetSchema());
}

void
ifm3d::O3R::Lock(const std::string& password)
{
  this->pImpl->Lock(password);
}

void
ifm3d::O3R::Unlock(const std::string& password)
{
  this->pImpl->Unlock(password);
}

ifm3d::PortInfo
ifm3d::O3R::Port(const std::string& port)
{
  return this->pImpl->Port(port);
}

std::vector<ifm3d::PortInfo>
ifm3d::O3R::Ports()
{
  return this->pImpl->Ports();
}

ifm3d::Device::device_family
ifm3d::O3R::WhoAmI()
{
  return device_family::O3R;
}

void
ifm3d::O3R::FromJSON(const json& j)
{
  this->Set(j);
}

ifm3d::json
ifm3d::O3R::ToJSON()
{
  return this->Get();
}

ifm3d::json
ifm3d::O3R::GetDiagnostic()
{
  return this->pImpl->GetDiagnostic();
}

ifm3d::json
ifm3d::O3R::GetDiagnosticFilterSchema()
{
  return this->pImpl->GetDiagnosticFilterSchema();
}

ifm3d::json
ifm3d::O3R::GetDiagnosticFiltered(json filter)
{
  return this->pImpl->GetDiagnosticFiltered(std::move(filter));
}

void
ifm3d::O3R::Reboot(const boot_mode& mode)
{
  switch (mode)
    {
    case boot_mode::PRODUCTIVE:
      this->pImpl->Reboot();
      break;

    case boot_mode::RECOVERY:
      this->pImpl->RebootToRecovery();
      break;
    }
}

void
ifm3d::O3R::RebootToRecovery()
{
  this->pImpl->RebootToRecovery();
}

ifm3d::Device::swu_version
ifm3d::O3R::SwUpdateVersion()
{
  return ifm3d::Device::swu_version::SWU_V2;
}

void
ifm3d::O3R::DownloadServiceReport(std::string out_file)
{
  this->pImpl->DownloadServiceReport(std::move(out_file));
}

#ifdef BUILD_MODULE_CRYPTO

std::shared_ptr<ifm3d::O3RSealedBox>
ifm3d::O3R::SealedBox()
{
  return std::make_shared<O3RSealedBox>(this->pImpl);
}

ifm3d::O3RSealedBox::O3RSealedBox(std::shared_ptr<O3R::Impl> p_impl)
  : pImpl(std::move(p_impl))
{}

ifm3d::O3RSealedBox::~O3RSealedBox() = default;

void
ifm3d::O3RSealedBox::SetPassword(const std::string& new_password,
                                 std::optional<std::string> old_password)
{
  this->pImpl->SealedBoxSetPassword(new_password, std::move(old_password));
}

bool
ifm3d::O3RSealedBox::IsPasswordProtected()
{
  return this->pImpl->SealedBoxIsPasswordProtected();
}

void
ifm3d::O3RSealedBox::RemovePassword(std::string password)
{
  this->pImpl->SealedBoxRemovePassword(std::move(password));
}

std::vector<uint8_t>
ifm3d::O3RSealedBox::GetPublicKey()
{
  return this->pImpl->SealedBoxGetPublicKey();
}

void
ifm3d::O3RSealedBox::Set(const std::string& password,
                         const json& configuration)
{
  this->pImpl->SealedBoxSet(password, configuration);
}

#endif