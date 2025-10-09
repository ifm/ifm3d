/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/device/device.h>
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
    _impl(new ifm3d::O3R::Impl(x_wrapper()))
{}

ifm3d::O3R::~O3R() = default;

void
ifm3d::O3R::FactoryReset(bool keep_network_settings)
{
  this->_impl->FactoryReset(keep_network_settings);
}

ifm3d::json
ifm3d::O3R::Get(const std::vector<std::string>& path)
{
  return this->_impl->Get(path);
}

ifm3d::json
ifm3d::O3R::ResolveConfig(const json::json_pointer& ptr)
{
  return this->_impl->ResolveConfig(ptr);
}

void
ifm3d::O3R::Set(const json& j)
{
  this->_impl->Set(j.dump());
}

void
ifm3d::O3R::Remove(const std::string& json_pointer)
{
  this->_impl->Remove(json_pointer);
}

void
ifm3d::O3R::Reset(const std::string& json_pointer)
{
  this->_impl->Reset(json_pointer);
}

ifm3d::json
ifm3d::O3R::GetInit()
{
  return this->_impl->GetInit();
}

void
ifm3d::O3R::SaveInit(const std::vector<std::string>& pointers)
{
  this->_impl->SaveInit(pointers);
}

std::string
ifm3d::O3R::GetInitStatus()
{
  return this->_impl->GetInitStatus();
}

ifm3d::json
ifm3d::O3R::GetSchema()
{
  return json::parse(this->_impl->GetSchema());
}

void
ifm3d::O3R::Lock(const std::string& password)
{
  this->_impl->Lock(password);
}

void
ifm3d::O3R::Unlock(const std::string& password)
{
  this->_impl->Unlock(password);
}

ifm3d::PortInfo
ifm3d::O3R::Port(const std::string& port)
{
  return this->_impl->Port(port);
}

std::vector<ifm3d::PortInfo>
ifm3d::O3R::Ports()
{
  return this->_impl->Ports();
}

ifm3d::Device::DeviceFamily
ifm3d::O3R::WhoAmI()
{
  return DeviceFamily::O3R;
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
  return this->_impl->GetDiagnostic();
}

ifm3d::json
ifm3d::O3R::GetDiagnosticFilterSchema()
{
  return this->_impl->GetDiagnosticFilterSchema();
}

ifm3d::json
ifm3d::O3R::GetDiagnosticFiltered(const json& filter)
{
  return this->_impl->GetDiagnosticFiltered(filter);
}

void
ifm3d::O3R::Reboot(const BootMode& mode)
{
  switch (mode)
    {
    case BootMode::PRODUCTIVE:
      this->_impl->Reboot();
      break;

    case BootMode::RECOVERY:
      this->_impl->RebootToRecovery();
      break;
    }
}

void
ifm3d::O3R::RebootToRecovery()
{
  this->_impl->RebootToRecovery();
}

ifm3d::Device::SWUVersion
ifm3d::O3R::SwUpdateVersion()
{
  return ifm3d::Device::SWUVersion::SWU_V2;
}

void
ifm3d::O3R::DownloadServiceReport(const std::string& out_file)
{
  // NOLINTNEXTLINE(clang-analyzer-unix.BlockInCriticalSection)
  this->_impl->DownloadServiceReport(out_file);
}

#ifdef BUILD_MODULE_CRYPTO

std::shared_ptr<ifm3d::O3RSealedBox>
ifm3d::O3R::SealedBox()
{
  return std::make_shared<O3RSealedBox>(this->_impl);
}

ifm3d::O3RSealedBox::O3RSealedBox(std::shared_ptr<O3R::Impl> p_impl)
  : _impl(std::move(p_impl))
{}

ifm3d::O3RSealedBox::~O3RSealedBox() = default;

void
ifm3d::O3RSealedBox::SetPassword(const std::string& new_password,
                                 std::optional<std::string> old_password)
{
  this->_impl->SealedBoxSetPassword(new_password, std::move(old_password));
}

bool
ifm3d::O3RSealedBox::IsPasswordProtected()
{
  return this->_impl->SealedBoxIsPasswordProtected();
}

void
ifm3d::O3RSealedBox::RemovePassword(std::string password)
{
  this->_impl->SealedBoxRemovePassword(std::move(password));
}

std::vector<uint8_t>
ifm3d::O3RSealedBox::GetPublicKey()
{
  return this->_impl->SealedBoxGetPublicKey();
}

void
ifm3d::O3RSealedBox::Set(const std::string& password,
                         const json& configuration)
{
  this->_impl->SealedBoxSet(password, configuration);
}

#endif