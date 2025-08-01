// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_O3R_IMPL_HPP
#define IFM3D_DEVICE_O3R_IMPL_HPP

#include <ctime>
#include <string>
#include <vector>
#include <fstream>
#include <fmt/core.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/err.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/util.h>
#include <xmlrpc.hpp>

namespace ifm3d
{
  const std::string XMLRPC_DIAGNOSTIC = "/api/rpc/v1/com.ifm.diagnostic/";

  class XMLRPC;

  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_NO_EXPORT O3R::Impl
  {
  public:
    explicit Impl(std::shared_ptr<XMLRPC> xwrapper);
    ~Impl();

    json Get(const std::vector<std::string>& path);
    json ResolveConfig(const json::json_pointer& ptr);
    void Set(const std::string& config);
    void Remove(const std::string& jsonPointer);
    void Reset(const std::string& jsonPointer);
    json GetInit();
    void SaveInit(const std::vector<std::string>& pointers);
    std::string GetInitStatus();
    std::string GetSchema();
    void Lock(const std::string& password);
    void Unlock(const std::string& password);
    std::vector<PortInfo> Ports();
    json GetDiagnostic();
    json GetDiagnosticFilterSchema();
    json GetDiagnosticFiltered(json filter);
    PortInfo Port(const std::string& port);

    void FactoryReset(bool keepNetworkSettings);
    void Reboot();
    void RebootToRecovery();
    void DownloadServiceReport(std::string outFile);

#ifdef BUILD_MODULE_CRYPTO
    // SealedBoxImpl
    void SealedBoxSetPassword(
      const std::string& new_password,
      std::optional<std::string> old_password = std::nullopt);
    bool SealedBoxIsPasswordProtected();
    void SealedBoxRemovePassword(std::string password);
    void SealedBoxSet(const std::string& password, const json& configuration);
    std::vector<uint8_t> SealedBoxGetPublicKey();

    std::vector<uint8_t> SealedBoxEncryptMessage(const json& message);
    void SealedBoxSendCommand(const std::string& command,
                              const json& data = json::object());
#endif

  protected:
    std::shared_ptr<XMLRPC> xwrapper_;

  private:
    template <typename... Args>
    XMLRPCValue const
    _XCallDiagnostic(const std::string& method, Args... args)
    {
      return this->xwrapper_->XCall(ifm3d::XMLRPC_DIAGNOSTIC, method, args...);
    }
  }; // end: class O3RCamera::Impl

} // end: namespace ifm3d

ifm3d::O3R::Impl::Impl(std::shared_ptr<XMLRPC> xwrapper)
  : xwrapper_(std::move(xwrapper))
{}

ifm3d::O3R::Impl::~Impl() {}

ifm3d::json
ifm3d::O3R::Impl::Get(const std::vector<std::string>& path)
{
  return json::parse(this->xwrapper_->XCallMain("get", path).AsString());
}

ifm3d::json
ifm3d::O3R::Impl::ResolveConfig(const json::json_pointer& ptr)
{
  return this->Get({ptr.to_string()})[ptr];
}

void
ifm3d::O3R::Impl::Set(const std::string& config)
{
  this->xwrapper_->XCallMainTimeout("set", NET_WAIT_O3R_SET, config);
}

void
ifm3d::O3R::Impl::Remove(const std::string& jsonPointer)
{
  this->xwrapper_->XCallMain("remove", jsonPointer);
}

void
ifm3d::O3R::Impl::Reset(const std::string& jsonPointer)
{
  this->xwrapper_->XCallMain("reset", jsonPointer);
}

ifm3d::json
ifm3d::O3R::Impl::GetInit()
{
  return json::parse(this->xwrapper_->XCallMain("getInit").AsString());
}

void
ifm3d::O3R::Impl::SaveInit(const std::vector<std::string>& pointers)
{
  if (pointers.size() == 0)
    {
      this->xwrapper_->XCallMain("saveInit");
    }
  else
    {
      this->xwrapper_->XCallMain("saveInit", pointers);
    }
}

std::string
ifm3d::O3R::Impl::GetInitStatus()
{
  return this->xwrapper_
    ->XCallMain("getInitStatus", std::vector<std::string>())
    .AsString();
}

std::string
ifm3d::O3R::Impl::GetSchema()
{
  return this->xwrapper_->XCallMain("getSchema").AsString();
}

void
ifm3d::O3R::Impl::Lock(const std::string& password)
{
  this->xwrapper_->XCallMain("lock", password);
}

void
ifm3d::O3R::Impl::Unlock(const std::string& password)
{
  this->xwrapper_->XCallMain("unlock", password);
}

ifm3d::PortInfo
ifm3d::O3R::Impl::Port(const std::string& port)
{
  auto get_port_data = [this](const ifm3d::json::json_pointer basePtr,
                              const std::string& port) {
    auto port_data = ResolveConfig(basePtr / port);

    if (port_data.is_null())
      {
        throw ifm3d::Error(IFM3D_INVALID_PORT);
      }
    return port_data;
  };
  try
    {
      if (port.find("app") == 0)
        {
          auto port_data =
            get_port_data("/applications/instances"_json_pointer, port);
          auto pcicTCPPort = port_data["/data/pcicTCPPort"_json_pointer];
          return {port, pcicTCPPort, "app"};
        }
      if (port.find("port") == 0)
        {
          auto port_data = get_port_data("/ports"_json_pointer, port);
          auto pcicTCPPort = port_data["/data/pcicTCPPort"_json_pointer];
          auto type = port_data["/info/features/type"_json_pointer];
          return {port, pcicTCPPort, type};
        }
      if (port.find("diagnostics") == 0)
        {
          auto json = ResolveConfig({"/device/diagnostic"_json_pointer});
          auto daig_port = json["/data/pcicPort"_json_pointer];
          return {"diagnostics", daig_port, "Diagnostics"};
        }
    }
  catch (const ifm3d::json::exception& ex)
    {
      LOG_WARNING("JSON: {}", ex.what());
      throw ifm3d::Error(IFM3D_JSON_ERROR);
    }
  throw ifm3d::Error(IFM3D_DEVICE_PORT_NOT_SUPPORTED, port);
}

std::vector<ifm3d::PortInfo>
ifm3d::O3R::Impl::Ports()
{
  std::vector<ifm3d::PortInfo> result;
  ifm3d::json device_dump = {};

  try
    {
      device_dump =
        Get({"/ports", "/applications/instances", "/device/diagnostic"});
    }
  catch (ifm3d::Error& error)
    {
      LOG_WARNING("XMLRPC ERRO : {}", error.what());
      device_dump = Get({});
    }
  try
    {
      auto json_ptr = "/ports"_json_pointer;
      if (device_dump.contains(json_ptr))
        {
          auto ports = device_dump[json_ptr];
          for (const auto& port : ports.items())
            {
              auto port_key = port.key();
              auto port_data = port.value();

              result.push_back(
                {port_key,
                 port_data["/data/pcicTCPPort"_json_pointer],
                 port_data["/info/features/type"_json_pointer]});
            }
        }
    }
  catch (const std::exception& ex)
    {
      LOG_WARNING("JSON: {}", ex.what());
    }
  try
    {
      auto json_ptr = "/applications/instances"_json_pointer;
      if (device_dump.contains(json_ptr))
        {
          auto app_ports = device_dump["/applications/instances"_json_pointer];
          for (const auto& port : app_ports.items())
            {
              auto port_key = port.key();
              auto port_data = port.value();

              result.push_back({port_key,
                                port_data["/data/pcicTCPPort"_json_pointer],
                                "app"});
            }
        }
    }
  catch (const std::exception& ex)
    {
      LOG_WARNING("JSON: {}", ex.what());
    }

  try
    {
      auto json_ptr = "/device/diagnostic"_json_pointer;
      if (device_dump.contains(json_ptr))
        {
          auto daig_port =
            device_dump["/device/diagnostic/data/pcicPort"_json_pointer];

          result.push_back({"diagnostics", daig_port, "Diagnostics"});
        }
    }
  catch (const std::exception& ex)
    {
      LOG_WARNING("JSON: {}", ex.what());
    }

  return result;
}

ifm3d::json
ifm3d::O3R::Impl::GetDiagnostic()
{
  return json::parse(this->_XCallDiagnostic("get").AsString());
}

ifm3d::json
ifm3d::O3R::Impl::GetDiagnosticFilterSchema()
{
  return json::parse(this->_XCallDiagnostic("getFilterSchema").AsString());
}

ifm3d::json
ifm3d::O3R::Impl::GetDiagnosticFiltered(json filter)
{
  return json::parse(
    this->_XCallDiagnostic("getFiltered", filter.dump()).AsString());
}

void
ifm3d::O3R::Impl::FactoryReset(bool keepNetworkSettings)
{
  this->xwrapper_->XCallMain("factoryReset", keepNetworkSettings);
}

void
ifm3d::O3R::Impl::Reboot()
{
  this->xwrapper_->XCallMain("reboot");
}

void
ifm3d::O3R::Impl::RebootToRecovery()
{
  this->xwrapper_->XCallMain("rebootToRecovery");
}

size_t
WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
  std::ofstream* ofs = static_cast<std::ofstream*>(userp);
  size_t totalSize = size * nmemb;
  ofs->write(static_cast<char*>(contents), totalSize);
  return totalSize;
}

void
ifm3d::O3R::Impl::DownloadServiceReport(std::string outFile)
{
  httplib::Client cli(this->xwrapper_->IP(), 80);
  std::ofstream ofs("service_report.zip", std::ios::binary);
  auto res =
    cli.Get("/service_report/", [&](const char* data, size_t data_length) {
      ofs.write(data, data_length);
      return true;
    });

  ifm3d::check_http_result(res);
}

#ifdef BUILD_MODULE_CRYPTO

void
ifm3d::O3R::Impl::SealedBoxSetPassword(const std::string& new_password,
                                       std::optional<std::string> old_password)
{
  if (old_password)
    {
      SealedBoxSendCommand(
        "set_password",
        {{"new_password", new_password}, {"password", old_password.value()}});
    }
  else
    {
      SealedBoxSendCommand("set_password", {{"new_password", new_password}});
    }
}

bool
ifm3d::O3R::Impl::SealedBoxIsPasswordProtected()
{
  return this->ResolveConfig(
    "/device/crypto/sealedbox/passwordProtected"_json_pointer);
}

void
ifm3d::O3R::Impl::SealedBoxRemovePassword(std::string password)
{
  SealedBoxSendCommand("remove_password", {{"password", password}});
}

inline std::vector<uint8_t>
ifm3d::O3R::Impl::SealedBoxGetPublicKey()
{
  return base64_decode(
    this->ResolveConfig("/device/crypto/sealedbox/public_key"_json_pointer)
      .get<std::string>());
}

std::vector<uint8_t>
ifm3d::O3R::Impl::SealedBoxEncryptMessage(const json& message)
{
  return ifm3d::SealedBox(SealedBoxGetPublicKey()).Encrypt(message.dump());
}

void
ifm3d::O3R::Impl::SealedBoxSet(const std::string& password,
                               const json& configuration)
{
  SealedBoxSendCommand(
    "set_configuration",
    {{"password", password}, {"configuration", configuration}});
}

void
ifm3d::O3R::Impl::SealedBoxSendCommand(const std::string& command,
                                       const json& additional_data)
{
  json message = additional_data;
  message.update(
    {{"nonce", base64_encode(RandomNonce())}, {"request", command}});

  this->Set(
    json({{"device",
           {{"crypto",
             {{"sealedbox",
               {{"message",
                 base64_encode(SealedBoxEncryptMessage(message))}}}}}}}})
      .dump());
}

#endif

#endif // IFM3D_DEVICE_O3R_IMPL_HPP