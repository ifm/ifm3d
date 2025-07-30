#include "xmlrpc.hpp"
#include <cstdint>
#include <string>
#include "fmt/core.h"
#include "ifm3d/common/err.h"
#include "ifm3d/common/json_impl.hpp"
#include <httplib.h>
#include <tinyxml2.h>
#include <cstring>
#include <ifm3d/device/util.h>
#include <utility>
#include <vector>
#include <unordered_map>
#include <variant>
#include <type_traits>

namespace ifm3d
{
  double
  XMLRPCValue::AsDouble() const
  {
    return std::get<double>(*value_);
  }

  std::vector<XMLRPCValue> const&
  XMLRPCValue::AsArray() const
  {
    return std::get<std::vector<XMLRPCValue>>(*value_);
  }

  std::vector<std::uint8_t> const&
  XMLRPCValue::AsByteArray() const
  {
    return std::get<std::vector<std::uint8_t>>(*value_);
  }

  std::unordered_map<std::string, XMLRPCValue> const&
  XMLRPCValue::AsMap() const
  {
    return std::get<std::unordered_map<std::string, XMLRPCValue>>(*value_);
  }

  std::unordered_map<std::string, std::string>
  XMLRPCValue::ToStringMap() const
  {
    std::unordered_map<std::string, std::string> map;
    for (const auto& [key, value] : this->AsMap())
      {
        map[key] = value.AsString();
      }
    return map;
  }

  bool
  XMLRPCValue::IsValid() const
  {
    return !std::holds_alternative<std::monostate>(*value_);
  }

  std::string const&
  XMLRPCValue::AsString() const
  {
    return std::get<std::string>(*value_);
  }

  int32_t
  XMLRPCValue::AsInt() const
  {
    return std::get<int32_t>(*value_);
  }

  bool
  XMLRPCValue::AsBool() const
  {
    return std::get<bool>(*value_);
  }

  void
  XMLRPCValue::ToXML(tinyxml2::XMLDocument* doc,
                     tinyxml2::XMLElement* parent) const
  {
    tinyxml2::XMLElement* value_element = doc->NewElement("value");
    std::visit(
      [&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::string>)
          {
            tinyxml2::XMLElement* string_element = doc->NewElement("string");
            string_element->SetText(arg.c_str());
            value_element->InsertEndChild(string_element);
          }
        else if constexpr (std::is_same_v<T, int32_t>)
          {
            tinyxml2::XMLElement* int_element = doc->NewElement("i4");
            int_element->SetText(arg);
            value_element->InsertEndChild(int_element);
          }
        else if constexpr (std::is_same_v<T, bool>)
          {
            tinyxml2::XMLElement* bool_element = doc->NewElement("boolean");
            bool_element->SetText(arg ? "1" : "0");
            value_element->InsertEndChild(bool_element);
          }
        else if constexpr (std::is_same_v<T, double>)
          {
            tinyxml2::XMLElement* double_element = doc->NewElement("double");
            double_element->SetText(arg);
            value_element->InsertEndChild(double_element);
          }
        else if constexpr (std::is_same_v<T, std::vector<uint8_t>>)
          {
            tinyxml2::XMLElement* base64_element = doc->NewElement("base64");
            base64_element->SetText(ifm3d::base64_encode(arg).c_str());
            value_element->InsertEndChild(base64_element);
          }
        else if constexpr (std::is_same_v<T, std::vector<XMLRPCValue>>)
          {
            tinyxml2::XMLElement* array_element = doc->NewElement("array");
            tinyxml2::XMLElement* data_element = doc->NewElement("data");
            for (const auto& arr_elem : arg)
              {
                arr_elem.ToXML(doc, data_element);
              }
            array_element->InsertEndChild(data_element);
            value_element->InsertEndChild(array_element);
          }
      },
      *this->value_);
    parent->InsertEndChild(value_element);
  }

  XMLRPCValue
  XMLRPCValue::FromXML(tinyxml2::XMLElement* el)
  {
    tinyxml2::XMLElement* inner = el->FirstChildElement();

    if (!inner)
      {
        return {};
      }

    static const char* EMPTY_TEXT = "";

    const char* result_type = inner->Name();
    const char* text = inner->GetText() ? inner->GetText() : EMPTY_TEXT;

    if (std::strcmp(result_type, "string") == 0)
      {
        return std::string(text);
      }
    if (std::strcmp(result_type, "i4") == 0 ||
        std::strcmp(result_type, "int") == 0)
      {
        return std::stoi(text);
      }
    if (std::strcmp(result_type, "double") == 0)
      {
        return std::stod(text);
      }
    if (std::strcmp(result_type, "boolean") == 0)
      {
        return std::strcmp(text, "1") == 0;
      }
    if (std::strcmp(result_type, "array") == 0)
      {
        tinyxml2::XMLElement* data = inner->FirstChildElement();
        if (data)
          {
            std::vector<XMLRPCValue> array;
            tinyxml2::XMLElement* value = data->FirstChildElement();
            while (value)
              {
                array.push_back(XMLRPCValue::FromXML(value));
                value = value->NextSiblingElement();
              }
            return array;
          }
      }
    if (std::strcmp(result_type, "struct") == 0)
      {
        std::unordered_map<std::string, XMLRPCValue> map;
        tinyxml2::XMLElement* member = inner->FirstChildElement();
        while (member)
          {
            tinyxml2::XMLElement* name = member->FirstChildElement("name");
            tinyxml2::XMLElement* value = member->FirstChildElement("value");
            map.emplace(name->GetText(), XMLRPCValue::FromXML(value));
            member = member->NextSiblingElement();
          }
        return map;
      }
    if (std::strcmp(result_type, "base64") == 0)
      {
        return ifm3d::base64_decode(inner->GetText());
      }

    throw ifm3d::Error(IFM3D_XMLRPC_FAILURE,
                       fmt::format("Unknown XML-RPC type: {}", result_type));
  }

  json
  XMLRPCValue::ToJson() const
  {
    json j;
    std::visit(
      [&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::string> ||
                      std::is_same_v<T, int32_t> ||
                      std::is_same_v<T, bool> | std::is_same_v<T, double>)
          {
            j = arg;
          }
        else if constexpr (std::is_same_v<T, std::vector<XMLRPCValue>>)
          {
            j = json::array();
            for (const auto& arr_elem : arg)
              {
                j.push_back(arr_elem.ToJson());
              }
          }
        else if constexpr (std::is_same_v<
                             T,
                             std::unordered_map<std::string, XMLRPCValue>>)
          {
            j = json::object();
            for (const auto& [key, value] : arg)
              {
                j[key] = value.ToJson();
              }
          }
      },
      *this->value_);
    return j;
  }

  XMLRPC::XMLRPC(std::string ip, const std::uint16_t xmlrpc_port)
    : ip_(std::move(ip)),
      xmlrpc_port_(xmlrpc_port)
  {}

  std::string
  XMLRPC::IP()
  {
    return this->ip_;
  }

  std::uint16_t
  XMLRPC::XMLRPCPort() const
  {
    return this->xmlrpc_port_;
  }

  XMLRPCValue
  XMLRPC::ParseXMLRPCResponse(const std::string& xml_response)
  {
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError const result = doc.Parse(xml_response.c_str());

    if (result != tinyxml2::XML_SUCCESS)
      {
        throw ifm3d::Error(IFM3D_XMLRPC_FAILURE,
                           "Malformed XML received from server.");
      }

    tinyxml2::XMLElement* method_response =
      doc.FirstChildElement("methodResponse");
    if (!method_response)
      {
        throw ifm3d::Error(
          IFM3D_XMLRPC_FAILURE,
          "Invalid XML-RPC response. <methodResponse> not found.");
      }

    tinyxml2::XMLElement* fault = method_response->FirstChildElement("fault");
    if (fault)
      {
        tinyxml2::XMLElement* value_element =
          fault->FirstChildElement("value");
        if (value_element)
          {
            auto value = XMLRPCValue::FromXML(value_element).AsMap();
            if (value.find("faultCode") != value.end() &&
                value.find("faultString") != value.end())
              {
                throw ifm3d::Error(value["faultCode"].AsInt(),
                                   value["faultString"].AsString());
              }

            throw ifm3d::Error(
              IFM3D_XMLRPC_FAILURE,
              "Invalid XML-RPC response. Invalid fault structure.");
          }
      }

    tinyxml2::XMLElement* params =
      method_response->FirstChildElement("params");
    if (params)
      {
        tinyxml2::XMLElement* param = params->FirstChildElement("param");
        if (param)
          {
            tinyxml2::XMLElement* value = param->FirstChildElement("value");
            if (value)
              {
                return XMLRPCValue::FromXML(value);
              }
          }
      }

    throw ifm3d::Error(IFM3D_XMLRPC_FAILURE,
                       "No valid result found in the response.");
  }

  std::string
  XMLRPC::CreateXMLRPCRequest(const std::string& method_name,
                              const std::vector<XMLRPCValue>& params)
  {
    tinyxml2::XMLDocument doc;

    tinyxml2::XMLDeclaration* decl = doc.NewDeclaration("xml version=\"1.0\"");
    doc.InsertFirstChild(decl);

    tinyxml2::XMLElement* method_call = doc.NewElement("methodCall");
    doc.InsertEndChild(method_call);

    tinyxml2::XMLElement* method_elem = doc.NewElement("methodName");
    method_elem->SetText(method_name.c_str());
    method_call->InsertEndChild(method_elem);

    tinyxml2::XMLElement* params_element = doc.NewElement("params");
    method_call->InsertEndChild(params_element);

    for (const auto& param : params)
      {
        tinyxml2::XMLElement* param_element = doc.NewElement("param");
        params_element->InsertEndChild(param_element);

        param.ToXML(&doc, param_element);
      }

    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);

    return printer.CStr();
  }

  XMLRPCValue
  XMLRPC::DoXMLRPCCall(const std::string& path,
                       const std::string& method,
                       int /*timeout*/,
                       std::vector<XMLRPCValue>& params)
  {
    std::string const xmlrpc_request = CreateXMLRPCRequest(method, params);

    httplib::Client client(this->ip_, this->xmlrpc_port_);

    auto res = client.Post(path, xmlrpc_request, "text/xml");

    auto timeout_sec = NET_WAIT / 1000;
    auto timeout_usec = (NET_WAIT % 1000) * 1000;

    client.set_connection_timeout(timeout_sec, timeout_usec);
    client.set_read_timeout(timeout_sec, timeout_usec);
    client.set_write_timeout(timeout_sec, timeout_usec);

    ifm3d::check_http_result(res);

    return ParseXMLRPCResponse(res->body);
  }
}