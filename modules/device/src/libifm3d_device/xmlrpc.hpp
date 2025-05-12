// -*- c++ -*-
/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_XMLRPC_HPP
#define IFM3D_CAMERA_XMLRPC_HPP

#include <cstdint>
#include <ctime>
#include <string>
#include <unordered_map>
#include <vector>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/common/logging/log.h>
#include <httplib.h>
#include <tinyxml2.h>
#include <variant>

namespace ifm3d
{
  const int NET_WAIT = std::getenv("IFM3D_NET_WAIT") == nullptr ?
                         3000 :
                         std::stoi(std::getenv("IFM3D_NET_WAIT")); // millis

  const std::string XMLRPC_MAIN = "/api/rpc/v1/com.ifm.efector/";

  class XMLRPCValue
  {
    using value_type =
      std::variant<std::monostate,
                   std::string,
                   int32_t,
                   bool,
                   double,
                   std::vector<XMLRPCValue>,
                   std::unordered_map<std::string, XMLRPCValue>,
                   std::vector<std::uint8_t>>;

  public:
    XMLRPCValue() : value_(std::make_shared<value_type>(std::monostate())){};
    XMLRPCValue(std::string value)
      : value_(std::make_shared<value_type>(value))
    {}
    XMLRPCValue(const char* value)
      : value_(std::make_shared<value_type>(std::string(value)))
    {}
    XMLRPCValue(int32_t value) : value_(std::make_shared<value_type>(value)) {}
    XMLRPCValue(bool value) : value_(std::make_shared<value_type>(value)) {}
    XMLRPCValue(double value) : value_(std::make_shared<value_type>(value)) {}

    template <typename T>
    XMLRPCValue(std::vector<T> value)
    {
      auto vec = std::vector<XMLRPCValue>();
      for (const auto& elem : value)
        {
          vec.push_back(XMLRPCValue(elem));
        }
      value_ = std::make_shared<value_type>(vec);
    }

    template <typename T>
    XMLRPCValue(std::unordered_map<std::string, T> value)
    {
      auto map = std::unordered_map<std::string, XMLRPCValue>();
      for (const auto& elem : value)
        {
          map.emplace(elem.first, XMLRPCValue(elem.second));
        }
      value_ = std::make_shared<value_type>(map);
    }

    bool IsValid() const;

    double AsDouble() const;
    std::string const& AsString() const;
    int32_t AsInt() const;
    bool AsBool() const;
    std::vector<XMLRPCValue> const& AsArray() const;
    std::vector<std::uint8_t> const& AsByteArray() const;
    std::unordered_map<std::string, XMLRPCValue> const& AsMap() const;

    std::unordered_map<std::string, std::string> ToStringMap() const;

    void ToXML(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* parent) const;
    static XMLRPCValue FromXML(tinyxml2::XMLElement* el);

    json ToJson() const;

  private:
    std::shared_ptr<value_type> value_;
  };

  class XMLRPC
  {
  public:
    XMLRPC(std::string ip, const std::uint16_t xmlrpc_port);

    // ---------------------------------------------
    // Terminates recursion over the parameter pack
    // in _XSetParams
    // ---------------------------------------------
    void
    XSetParams(const std::vector<XMLRPCValue>& params)
    {}

    // ---------------------------------------------
    // Recursively processes a parameter pack `args'
    // as a list and sets those values into the
    // `params' reference.
    // ---------------------------------------------
    template <typename T, typename... Args>
    void
    XSetParams(std::vector<XMLRPCValue>& params, T value, Args... args)
    {
      params.push_back(XMLRPCValue(value));
      this->XSetParams(params, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    XCall(const std::string& path, const std::string& method, Args... args)
    {
      return XCallTimeout(path, method, NET_WAIT, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    XCallTimeout(const std::string& path,
                 const std::string& method,
                 int timeout,
                 Args... args)
    {
      std::vector<XMLRPCValue> params;
      this->XSetParams(params, args...);
      return this->DoXMLRPCCall(path, method, timeout, params);
    }

    template <typename... Args>
    XMLRPCValue const
    XCallMain(const std::string& method, Args... args)
    {
      return this->XCallMainTimeout(method, NET_WAIT, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    XCallMainTimeout(const std::string& method, int timeout, Args... args)
    {
      return this->XCallTimeout(ifm3d::XMLRPC_MAIN, method, timeout, args...);
    }

    std::string IP();
    std::uint16_t XMLRPCPort() const;

  private:
    XMLRPCValue DoXMLRPCCall(const std::string& path,
                             const std::string& method,
                             int timeout,
                             std::vector<XMLRPCValue>& params);
    std::string CreateXMLRPCRequest(const std::string& method_name,
                                    const std::vector<XMLRPCValue>& params);
    XMLRPCValue ParseXMLRPCResponse(const std::string& xml_response);

    std::string ip_;
    std::uint16_t xmlrpc_port_;
  };
}

#endif // IFM3D_CAMERA_XMLRPC_HPP