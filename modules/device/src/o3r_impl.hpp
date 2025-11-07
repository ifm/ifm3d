// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_O3R_IMPL_HPP
#define IFM3D_DEVICE_O3R_IMPL_HPP

#include <ctime>
#include <fmt/core.h> // NOLINT(*)
#include <fstream>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/err.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/util.h>
#include <string>
#include <utility>
#include <variant>
#include <vector>
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
    Impl(const Impl&) = default;
    Impl(Impl&&) = delete;
    Impl& operator=(const Impl&) = default;
    Impl& operator=(Impl&&) = delete;
    explicit Impl(std::shared_ptr<XMLRPC> xwrapper);
    ~Impl();

    json Get(const std::vector<std::string>& path);
    json ResolveConfig(const json::json_pointer& ptr);
    void Set(const std::string& config);
    void Remove(const std::string& json_pointer);
    void Reset(const std::string& json_pointer);
    json GetInit();
    void SaveInit(const std::vector<std::string>& pointers);
    std::string GetInitStatus();
    std::string GetSchema();
    std::string GetSchema(
      std::variant<std::monostate, std::string, std::vector<std::string>>
        pointers);
    std::string GetSchema(std::initializer_list<std::string> pointers);
    void Lock(const std::string& password);
    void Unlock(const std::string& password);
    std::vector<PortInfo> Ports();
    json GetDiagnostic();
    json GetDiagnosticFilterSchema();
    json GetDiagnosticFiltered(const json& filter);
    PortInfo Port(const std::string& port);

    void FactoryReset(bool keep_network_settings);
    void Reboot();
    void RebootToRecovery();
    void DownloadServiceReport(const std::string& out_file);

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
    std::shared_ptr<XMLRPC> _xwrapper;

  private:
    template <typename... ARGS>
    XMLRPCValue
    x_call_diagnostic(const std::string& method, ARGS... args)
    {
      return this->_xwrapper->XCall(ifm3d::XMLRPC_DIAGNOSTIC,
                                    method,
                                    std::move(args)...);
    }
  }; // end: class O3RCamera::Impl

} // end: namespace ifm3d

inline ifm3d::O3R::Impl::Impl(std::shared_ptr<XMLRPC> xwrapper)
  : _xwrapper(std::move(xwrapper))
{}

// NOLINTNEXTLINE(readability-redundant-inline-specifier)
inline ifm3d::O3R::Impl::~Impl() = default;

inline ifm3d::json
ifm3d::O3R::Impl::Get(const std::vector<std::string>& path)
{
  return json::parse(this->_xwrapper->XCallMain("get", path).AsString());
}

inline ifm3d::json
ifm3d::O3R::Impl::ResolveConfig(const json::json_pointer& ptr)
{
  return this->Get({ptr.to_string()})[ptr];
}

inline void
ifm3d::O3R::Impl::Set(const std::string& config)
{
  this->_xwrapper->XCallMainTimeout("set", NET_WAIT_O3R_SET, config);
}

inline void
ifm3d::O3R::Impl::Remove(const std::string& json_pointer)
{
  this->_xwrapper->XCallMain("remove", json_pointer);
}

inline void
ifm3d::O3R::Impl::Reset(const std::string& json_pointer)
{
  this->_xwrapper->XCallMain("reset", json_pointer);
}

inline ifm3d::json
ifm3d::O3R::Impl::GetInit()
{
  return json::parse(this->_xwrapper->XCallMain("getInit").AsString());
}

inline void
ifm3d::O3R::Impl::SaveInit(const std::vector<std::string>& pointers)
{
  if (pointers.size() == 0)
    {
      this->_xwrapper->XCallMain("saveInit");
    }
  else
    {
      this->_xwrapper->XCallMain("saveInit", pointers);
    }
}

inline std::string
ifm3d::O3R::Impl::GetInitStatus()
{
  return this->_xwrapper
    ->XCallMain("getInitStatus", std::vector<std::string>())
    .AsString();
}

inline std::string
ifm3d::O3R::Impl::GetSchema()
{
  return this->_xwrapper->XCallMain("getSchema").AsString();
}

inline std::string
ifm3d::O3R::Impl::GetSchema(
  std::variant<std::monostate, std::string, std::vector<std::string>> pointers)
{
  return std::visit(
    [this](auto&& value) -> std::string {
      using T = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<T, std::monostate>)
        {
          return this->_xwrapper->XCallMain("getSchema").AsString();
        }
      else if constexpr (std::is_same_v<T, std::string>)
        {
          return this->_xwrapper
            ->XCallMain("getSchema", std::vector<std::string>{value})
            .AsString();
        }
      else
        {
          return this->_xwrapper->XCallMain("getSchema", value).AsString();
        }
    },
    pointers);
}

inline std::string
ifm3d::O3R::Impl::GetSchema(std::initializer_list<std::string> pointers)
{
  return this->_xwrapper->XCallMain("getSchema", pointers).AsString();
}

inline void
ifm3d::O3R::Impl::Lock(const std::string& password)
{
  this->_xwrapper->XCallMain("lock", password);
}

inline void
ifm3d::O3R::Impl::Unlock(const std::string& password)
{
  this->_xwrapper->XCallMain("unlock", password);
}

inline ifm3d::PortInfo
ifm3d::O3R::Impl::Port(const std::string& port)
{
  auto get_port_data = [this](const ifm3d::json::json_pointer base_ptr,
                              const std::string& port) {
    auto port_data = ResolveConfig(base_ptr / port);

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
          auto pcic_tcp_port = port_data["/data/pcicTCPPort"_json_pointer];
          return {port, pcic_tcp_port, "app"};
        }
      if (port.find("port") == 0)
        {
          auto port_data = get_port_data("/ports"_json_pointer, port);
          auto pcic_tcp_port = port_data["/data/pcicTCPPort"_json_pointer];
          auto type = port_data["/info/features/type"_json_pointer];
          return {port, pcic_tcp_port, type};
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

inline std::vector<ifm3d::PortInfo>
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

inline ifm3d::json
ifm3d::O3R::Impl::GetDiagnostic()
{
  return json::parse(this->x_call_diagnostic("get").AsString());
}

inline ifm3d::json
ifm3d::O3R::Impl::GetDiagnosticFilterSchema()
{
  return json::parse(this->x_call_diagnostic("getFilterSchema").AsString());
}

inline ifm3d::json
ifm3d::O3R::Impl::GetDiagnosticFiltered(const json& filter)
{
  return json::parse(
    this->x_call_diagnostic("getFiltered", filter.dump()).AsString());
}

inline void
ifm3d::O3R::Impl::FactoryReset(bool keep_network_settings)
{
  this->_xwrapper->XCallMain("factoryReset", keep_network_settings);
}

inline void
ifm3d::O3R::Impl::Reboot()
{
  this->_xwrapper->XCallMain("reboot");
}

inline void
ifm3d::O3R::Impl::RebootToRecovery()
{
  try
    {
      this->_xwrapper->XCallMain("rebootToRecovery");
    }
  catch (const ifm3d::Error& e)
    {
      // The device will drop the connection without closing when it reboots to
      // recovery
      if (e.code() != IFM3D_CURL_ERROR ||
          std::strstr(e.message(), "Failed to read connection") == nullptr)
        {
          throw;
        }
    }
}

inline size_t
write_callback(void* contents, size_t size, size_t nmemb, void* userp)
{
  auto* ofs = static_cast<std::ofstream*>(userp);
  size_t total_size = size * nmemb;
  ofs->write(static_cast<char*>(contents),
             static_cast<std::streamsize>(total_size));
  return total_size;
}

inline void
ifm3d::O3R::Impl::DownloadServiceReport(const std::string& out_file)
{
  httplib::Client cli(this->_xwrapper->IP(), 80);
  std::ofstream ofs(out_file, std::ios::binary);
  auto res =
    cli.Get("/service_report/", [&](const char* data, size_t data_length) {
      ofs.write(data, static_cast<std::streamsize>(data_length));
      return true;
    });

  ifm3d::check_http_result(res);
}

#ifdef BUILD_MODULE_CRYPTO

inline void
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

inline bool
ifm3d::O3R::Impl::SealedBoxIsPasswordProtected()
{
  return this->ResolveConfig(
    "/device/crypto/sealedbox/passwordProtected"_json_pointer);
}

inline void
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

inline std::vector<uint8_t>
ifm3d::O3R::Impl::SealedBoxEncryptMessage(const json& message)
{
  return ifm3d::SealedBox(SealedBoxGetPublicKey()).Encrypt(message.dump());
}

inline void
ifm3d::O3R::Impl::SealedBoxSet(const std::string& password,
                               const json& configuration)
{
  SealedBoxSendCommand(
    "set_configuration",
    {{"password", password}, {"configuration", configuration}});
}

inline void
ifm3d::O3R::Impl::SealedBoxSendCommand(const std::string& command,
                                       const json& additional_data)
{
  json message = additional_data;
  message.update(
    {{"nonce", base64_encode(random_nonce())}, {"request", command}});

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