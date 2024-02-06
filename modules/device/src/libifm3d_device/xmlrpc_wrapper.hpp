// -*- c++ -*-
/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_XMLRPC_HPP
#define IFM3D_CAMERA_XMLRPC_HPP

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
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/common/logging/log.h>

namespace ifm3d
{
  const int NET_WAIT = std::getenv("IFM3D_NET_WAIT") == nullptr ?
                         3000 :
                         std::stoi(std::getenv("IFM3D_NET_WAIT")); // millis

  const std::string XMLRPC_MAIN = "/api/rpc/v1/com.ifm.efector/";

  class XMLRPCWrapper
  {
  public:
    XMLRPCWrapper(const std::string& ip, const std::uint16_t xmlrpc_port)
      : ip_(ip),
        xmlrpc_port_(xmlrpc_port),
        xmlrpc_url_prefix_("http://" + ip + ":" + std::to_string(xmlrpc_port))
    {}

    // ---------------------------------------------
    // Terminates recursion over the parameter pack
    // in _XSetParams
    // ---------------------------------------------
    void
    XSetParams(xmlrpc_c::paramList& params)
    {}

    // ---------------------------------------------
    // Recursively processes a parameter pack `args'
    // as a list and sets those values into the
    // `params' reference.
    // ---------------------------------------------
    template <typename T, typename... Args>
    void
    XSetParams(xmlrpc_c::paramList& params, T value, Args... args)
    {
      params.addc(value);
      this->XSetParams(params, args...);
    }

    // ---------------------------------------------
    // Encapsulates XMLRPC calls to the sensor and
    // unifies the trapping of comm errors.
    // ---------------------------------------------
    template <typename... Args>
    xmlrpc_c::value const
    XCall(std::string& url, const std::string& method, Args... args)
    {
      return XCallTimeout(url, method, NET_WAIT, args...);
    }

    // ---------------------------------------------
    // Encapsulates XMLRPC calls to the sensor and
    // unifies the trapping of comm errors.
    // ---------------------------------------------
    template <typename... Args>
    xmlrpc_c::value const
    XCallTimeout(std::string& url,
                 const std::string& method,
                 int timeout,
                 Args... args)
    {
      xmlrpc_c::paramList params;
      this->XSetParams(params, args...);
      xmlrpc_c::rpcPtr rpc(method, params);
      xmlrpc_c::carriageParm_curl0 cparam(url);
      xmlrpc_c::client_xml xlcient(
        xmlrpc_c::clientXmlTransportPtr(new xmlrpc_c::clientXmlTransport_curl(
          xmlrpc_c::clientXmlTransport_curl::constrOpt().timeout(timeout))));

      try
        {
          rpc->call(&xlcient, &cparam);
          return rpc->getResult();
        }
      catch (const std::exception& ex)
        {
          LOG_ERROR("{} -> {}: {}", url, method, ex.what());

          if (std::strstr(ex.what(), "HTTP response code is 407, not 200"))
            {
              throw ifm3d::Error(IFM3D_PROXY_AUTH_REQUIRED);
            }
          if (!rpc->isFinished())
            {
              throw ifm3d::Error(IFM3D_XMLRPC_TIMEOUT);
            }
          else if (!rpc->isSuccessful())
            {
              xmlrpc_c::fault f = rpc->getFault();
              throw ifm3d::Error(f.getCode(), f.getDescription());
            }
          else
            {
              throw ifm3d::Error(IFM3D_XMLRPC_FAILURE, ex.what());
            }
        }
    }

    template <typename... Args>
    xmlrpc_c::value const
    XCallMain(const std::string& method, Args... args)
    {
      return this->XCallMainTimeout(method, NET_WAIT, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    XCallMainTimeout(const std::string& method, int timeout, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN;
      return this->XCallTimeout(url, method, timeout, args...);
    }

    // utilities for taking xmlrpc structures to STL structures
    std::unordered_map<std::string, std::string> const
    value_struct_to_map(const xmlrpc_c::value_struct& vs)
    {

      std::map<std::string, xmlrpc_c::value> const resmap(
        static_cast<std::map<std::string, xmlrpc_c::value>>(vs));

      std::unordered_map<std::string, std::string> retval;
      for (auto& kv : resmap)
        {
          retval[kv.first] = std::string(xmlrpc_c::value_string(kv.second));
        }

      return retval;
    }

    std::unordered_map<std::string,
                       std::unordered_map<std::string, std::string>> const
    value_struct_to_map_of_maps(const xmlrpc_c::value_struct& vs)
    {
      std::unordered_map<std::string,
                         std::unordered_map<std::string, std::string>>
        retval;

      std::map<std::string, xmlrpc_c::value> const outter_map(
        static_cast<std::map<std::string, xmlrpc_c::value>>(vs));

      for (auto& kv : outter_map)
        {
          xmlrpc_c::value_struct _vs(kv.second);

          std::map<std::string, xmlrpc_c::value> const inner_map(
            static_cast<std::map<std::string, xmlrpc_c::value>>(_vs));

          std::unordered_map<std::string, std::string> inner_retval;

          for (auto& inner_kv : inner_map)
            {
              inner_retval[inner_kv.first] =
                std::string(xmlrpc_c::value_string(inner_kv.second));
            }

          retval[kv.first] = inner_retval;
        }

      return retval;
    }

    std::string
    XPrefix()
    {
      return this->xmlrpc_url_prefix_;
    }

    std::string
    IP()
    {
      return this->ip_;
    }

    std::uint16_t
    XMLRPCPort()
    {
      return this->xmlrpc_port_;
    }

  private:
    std::string ip_;
    std::uint16_t xmlrpc_port_;
    std::string xmlrpc_url_prefix_;
    xmlrpc_c::clientPtr xclient_;
  };
}

#endif // IFM3D_CAMERA_XMLRPC_HPP