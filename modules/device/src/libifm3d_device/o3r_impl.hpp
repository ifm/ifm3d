// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_O3R_IMPL_HPP
#define IFM3D_DEVICE_O3R_IMPL_HPP

#include <chrono>
#include <cstdint>
#include <ctime>
#include <functional>
#include <map>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>
#include <xmlrpc-c/client.hpp>
#include <fmt/core.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/err.h>
#include <ifm3d/common/logging/log.h>
#include <xmlrpc_wrapper.hpp>

namespace ifm3d
{
  const std::string XMLRPC_DIAGNOSTIC = "/api/rpc/v1/com.ifm.diagnostic/";

  class XMLRPCWrapper;

  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_DEVICE_LOCAL O3R::Impl
  {
  public:
    explicit Impl(std::shared_ptr<XMLRPCWrapper> xwrapper);
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

  protected:
    std::shared_ptr<XMLRPCWrapper> xwrapper_;

  private:
    template <typename... Args>
    xmlrpc_c::value const
    _XCallDiagnostic(const std::string& method, Args... args)
    {
      std::string url = this->xwrapper_->XPrefix() + ifm3d::XMLRPC_DIAGNOSTIC;
      return this->xwrapper_->XCall(url, method, args...);
    }
  }; // end: class O3RCamera::Impl

} // end: namespace ifm3d

ifm3d::O3R::Impl::Impl(std::shared_ptr<XMLRPCWrapper> xwrapper)
  : xwrapper_(std::move(xwrapper))
{}

ifm3d::O3R::Impl::~Impl() {}

ifm3d::json
ifm3d::O3R::Impl::Get(const std::vector<std::string>& path)
{
  return json::parse(
    xmlrpc_c::value_string(this->xwrapper_->XCallMain("get", path)).cvalue());
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
  return json::parse(
    xmlrpc_c::value_string(this->xwrapper_->XCallMain("getInit")).cvalue());
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
  return xmlrpc_c::value_string(
           this->xwrapper_->XCallMain("getInitStatus",
                                      std::vector<std::string>()))
    .cvalue();
}

std::string
ifm3d::O3R::Impl::GetSchema()
{
  return xmlrpc_c::value_string(this->xwrapper_->XCallMain("getSchema"))
    .cvalue();
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
  return json::parse(
    xmlrpc_c::value_string(this->_XCallDiagnostic("get")).cvalue());
}

ifm3d::json
ifm3d::O3R::Impl::GetDiagnosticFilterSchema()
{
  return json::parse(
    xmlrpc_c::value_string(this->_XCallDiagnostic("getFilterSchema"))
      .cvalue());
}

ifm3d::json
ifm3d::O3R::Impl::GetDiagnosticFiltered(json filter)
{
  return json::parse(xmlrpc_c::value_string(
                       this->_XCallDiagnostic("getFiltered", filter.dump()))
                       .cvalue());
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

#endif // IFM3D_DEVICE_O3R_IMPL_HPP