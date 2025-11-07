// -*- c++ -*-
/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_XMLRPC_HPP
#define IFM3D_CAMERA_XMLRPC_HPP

#include <cstdint>
#include <ctime>
#include <httplib.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/legacy_device.h>
#include <initializer_list>
#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <variant>
#include <vector>

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
    XMLRPCValue() : _value(std::make_shared<value_type>(std::monostate())){};
    XMLRPCValue(std::monostate value)
      : _value(std::make_shared<value_type>(value))
    {}
    XMLRPCValue(std::string value)
      : _value(std::make_shared<value_type>(value))
    {}
    XMLRPCValue(const char* value)
      : _value(std::make_shared<value_type>(std::string(value)))
    {}
    XMLRPCValue(int32_t value) : _value(std::make_shared<value_type>(value)) {}
    XMLRPCValue(bool value) : _value(std::make_shared<value_type>(value)) {}
    XMLRPCValue(double value) : _value(std::make_shared<value_type>(value)) {}

    template <typename T>
    XMLRPCValue(std::vector<T> value)
    {
      auto vec = std::vector<XMLRPCValue>();
      for (const auto& elem : value)
        {
          vec.push_back(XMLRPCValue(elem));
        }
      _value = std::make_shared<value_type>(vec);
    }

    template <typename T>
    XMLRPCValue(std::unordered_map<std::string, T> value)
    {
      auto map = std::unordered_map<std::string, XMLRPCValue>();
      for (const auto& elem : value)
        {
          map.emplace(elem.first, XMLRPCValue(elem.second));
        }
      _value = std::make_shared<value_type>(map);
    }

    template <typename... T>
    XMLRPCValue(std::variant<T...>&& value)
    {
      _value = std::visit(
        [](auto&& val) -> std::shared_ptr<value_type> {
          return XMLRPCValue(std::move(val))._value;
        },
        std::move(value));
    }

    XMLRPCValue(std::initializer_list<std::string> list)
    {
      auto vec = std::vector<XMLRPCValue>();
      for (const auto& elem : list)
        {
          vec.emplace_back(elem);
        }
      _value = std::make_shared<value_type>(vec);
    }

    [[nodiscard]] bool IsValid() const;

    [[nodiscard]] double AsDouble() const;
    [[nodiscard]] std::string const& AsString() const;
    [[nodiscard]] int32_t AsInt() const;
    [[nodiscard]] bool AsBool() const;
    [[nodiscard]] std::vector<XMLRPCValue> const& AsArray() const;
    [[nodiscard]] std::vector<std::uint8_t> const& AsByteArray() const;
    [[nodiscard]] std::unordered_map<std::string, XMLRPCValue> const& AsMap()
      const;

    [[nodiscard]] std::unordered_map<std::string, std::string> ToStringMap()
      const;

    void ToXML(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* parent) const;
    static XMLRPCValue FromXML(tinyxml2::XMLElement* el);

    [[nodiscard]] json ToJson() const;

  private:
    std::shared_ptr<value_type> _value;
  };

  class XMLRPC
  {
  public:
    XMLRPC(std::string ip, std::uint16_t xmlrpc_port);

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
    template <typename T, typename... ARGS>
    void
    XSetParams(std::vector<XMLRPCValue>& params, T value, ARGS... args)
    {
      params.push_back(XMLRPCValue(value));
      this->XSetParams(params, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    XCall(const std::string& path, const std::string& method, ARGS... args)
    {
      return XCallTimeout(path, method, NET_WAIT, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    XCallTimeout(const std::string& path,
                 const std::string& method,
                 int timeout,
                 ARGS... args)
    {
      std::vector<XMLRPCValue> params;
      this->XSetParams(params, args...);
      return this->do_xmlrpc_call(path, method, timeout, params);
    }

    template <typename... ARGS>
    XMLRPCValue
    XCallMain(const std::string& method, ARGS... args)
    {
      return this->XCallMainTimeout(method, NET_WAIT, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    XCallMainTimeout(const std::string& method, int timeout, ARGS... args)
    {
      return this->XCallTimeout(ifm3d::XMLRPC_MAIN, method, timeout, args...);
    }

    std::string IP();
    [[nodiscard]] std::uint16_t XMLRPCPort() const;

  private:
    XMLRPCValue do_xmlrpc_call(const std::string& path,
                               const std::string& method,
                               int timeout,
                               std::vector<XMLRPCValue>& params);
    std::string create_xmlrpc_request(const std::string& method_name,
                                      const std::vector<XMLRPCValue>& params);
    XMLRPCValue parse_xmlrpc_response(const std::string& xml_response);

    std::string _ip;
    std::uint16_t _xmlrpc_port;
  };
}

#endif // IFM3D_CAMERA_XMLRPC_HPP