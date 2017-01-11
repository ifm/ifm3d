/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ifm3d/camera/camera.h>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <map>
#include <mutex>
#include <regex>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <glog/logging.h>
#include <xmlrpc-c/client.hpp>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/camera/version.h>

//================================================
// Public constants
//================================================
const std::string ifm3d::DEFAULT_PASSWORD = "";
const std::uint16_t ifm3d::DEFAULT_XMLRPC_PORT = 80;
const std::string ifm3d::DEFAULT_IP =
  std::getenv("IFM3D_IP") == nullptr ?
  "192.168.0.69" : std::string(std::getenv("IFM3D_IP"));
const int ifm3d::MAX_HEARTBEAT = 300; // secs

//================================================
// Private constants
//================================================

namespace ifm3d
{
  const int NET_WAIT = 3000; // millis

  const std::string XMLRPC_MAIN = "/api/rpc/v1/com.ifm.efector/";
  const std::string XMLRPC_SESSION = "session_$XXX/";
  const std::string XMLRPC_EDIT = "edit/";
  const std::string XMLRPC_DEVICE = "device/";
  const std::string XMLRPC_NET = "network/";
  const std::string XMLRPC_APP = "application/";
  const std::string XMLRPC_IMAGER = "imager_001/";
  const std::string XMLRPC_SPATIALFILTER = "spatialfilter";
  const std::string XMLRPC_TEMPORALFILTER = "temporalfilter";

  using app_entry_t = struct {
    int index;
    int id;
    std::string name;
    std::string description;
  };
}

//================================================
// Camera::Impl class - the private interface
//================================================

class ifm3d::Camera::Impl
{
private:
  std::string ip_;
  std::uint16_t xmlrpc_port_;
  std::string password_;
  std::string xmlrpc_url_prefix_;
  xmlrpc_c::clientPtr xclient_;
  std::mutex xclient_mutex_;
  std::string session_;
  std::mutex session_mutex_;

public:
  std::string XPrefix()
  {
    return this->xmlrpc_url_prefix_;
  }

  std::string IP()
  {
    return this->ip_;
  }

  std::uint16_t XMLRPCPort()
  {
    return this->xmlrpc_port_;
  }

  std::string Password()
  {
    return this->password_;
  }

  std::string SessionID()
  {
    std::lock_guard<std::mutex> lock(this->session_mutex_);
    return this->session_;
  }

  void SetSessionID(const std::string& id)
  {
    std::lock_guard<std::mutex> lock(this->session_mutex_);
    this->session_ = id;
  }

private:
  // ---------------------------------------------
  // Conversions from XMLRPC data types to
  // "normal" C++ data types
  // ---------------------------------------------
  std::unordered_map<std::string, std::string> const
  value_struct_to_map(const xmlrpc_c::value_struct& vs)
  {
    std::map<std::string, xmlrpc_c::value> const
      resmap(static_cast<std::map<std::string, xmlrpc_c::value> >(vs));

    std::unordered_map<std::string, std::string> retval;
    for (auto& kv : resmap)
      {
        retval[kv.first] = std::string(xmlrpc_c::value_string(kv.second));
      }

    return retval;
  }

  std::unordered_map<std::string,
                     std::unordered_map<std::string, std::string> > const
  value_struct_to_map_of_maps(const xmlrpc_c::value_struct& vs)
  {
    std::unordered_map<std::string,
                       std::unordered_map<std::string, std::string> >
      retval;

    std::map<std::string, xmlrpc_c::value> const
      outter_map(static_cast<std::map<std::string, xmlrpc_c::value> >
                 (vs));

    for (auto& kv : outter_map)
      {
        xmlrpc_c::value_struct _vs(kv.second);

        std::map<std::string, xmlrpc_c::value> const
          inner_map(static_cast<std::map<std::string, xmlrpc_c::value> >
                    (_vs));

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

  // ---------------------------------------------
  // Terminates recursion over the parameter pack
  // in _XSetParams
  // ---------------------------------------------
  void _XSetParams(xmlrpc_c::paramList& params) { }

  // ---------------------------------------------
  // Recursively processes a parameter pack `args'
  // as a list and sets those values into the
  // `params' reference.
  // ---------------------------------------------
  template <typename T, typename... Args>
  void _XSetParams(xmlrpc_c::paramList& params, T value, Args... args)
  {
    params.addc(value);
    this->_XSetParams(params, args...);
  }

  // ---------------------------------------------
  // Encapsulates XMLRPC calls to the sensor and
  // unifies the trapping of comm errors.
  // ---------------------------------------------
  template <typename... Args>
  xmlrpc_c::value const
  _XCall(std::string& url, const std::string& method, Args... args)
  {
    xmlrpc_c::paramList params;
    this->_XSetParams(params, args...);
    xmlrpc_c::rpcPtr rpc(method, params);

    url = std::regex_replace(url, std::regex("\\$XXX"), this->SessionID());
    xmlrpc_c::carriageParm_curl0 cparam(url);

    std::lock_guard<std::mutex> lock(this->xclient_mutex_);
    try
      {
        rpc->call(this->xclient_.get(), &cparam);
        return rpc->getResult();
      }
    catch (const std::exception& ex)
      {
        LOG(ERROR) << url << "->" << method << ":" << ex.what();

        if (! rpc->isFinished())
          {
            throw ifm3d::error_t(IFM3D_XMLRPC_TIMEOUT);
          }
        else if (! rpc->isSuccessful())
          {
            xmlrpc_c::fault f = rpc->getFault();
            throw ifm3d::error_t(f.getCode());
          }
        else
          {
            throw ifm3d::error_t(IFM3D_XMLRPC_FAILURE);
          }
      }
  }

  // ---------------------------------------------
  // _XCall wrappers
  // ---------------------------------------------
  template <typename... Args>
  xmlrpc_c::value const
  _XCallMain(const std::string& method, Args... args)
  {
    std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN;
    return this->_XCall(url, method, args...);
  }

  template <typename... Args>
  xmlrpc_c::value const
  _XCallSession(const std::string& method, Args... args)
  {
    std::string url =
      this->XPrefix() + ifm3d::XMLRPC_MAIN + ifm3d::XMLRPC_SESSION;
    return this->_XCall(url, method, args...);
  }

public:
  int Heartbeat(int hb)
  {
    xmlrpc_c::value_int v_int(this->_XCallSession("heartbeat", hb));
    return v_int.cvalue();
  }

  std::string RequestSession()
  {
    xmlrpc_c::value_string val_str(
      this->_XCallMain("requestSession",
                       this->Password().c_str(),
                       std::string("")));

    this->SetSessionID(static_cast<std::string>(val_str));
    this->Heartbeat(ifm3d::MAX_HEARTBEAT);
    return this->SessionID();
  }

  bool CancelSession()
  {
    if (this->SessionID() == "")
      {
        return true;
      }

    bool retval = true;

    try
      {
        this->_XCallSession("cancelSession");
        this->SetSessionID("");
      }
    catch (const ifm3d::error_t& ex)
      {
        LOG(ERROR) << "Failed to cancel session: "
                   << this->SessionID() << " -> "
                   << ex.what();
        retval = false;
      }

    return retval;
  }

  //
  // ctor
  //
  Impl(const std::string& ip,
       const std::uint16_t xmlrpc_port,
       const std::string& password)
    : ip_(ip),
      xmlrpc_port_(xmlrpc_port),
      password_(password),
      xmlrpc_url_prefix_("http://" + ip + ":" + std::to_string(xmlrpc_port)),
      xclient_(new xmlrpc_c::client_xml(
                 xmlrpc_c::clientXmlTransportPtr(
                   new xmlrpc_c::clientXmlTransport_curl(
                     xmlrpc_c::clientXmlTransport_curl::constrOpt().
                     timeout(ifm3d::NET_WAIT))))),
      session_("")
  {
    VLOG(IFM3D_TRACE) << "Initializing Camera: ip="
                      << this->IP()
                      << ", xmlrpc_port=" << this->XMLRPCPort()
                      << ", password=" << this->Password();
    VLOG(IFM3D_TRACE) << "XMLRPC URL Prefix=" << this->xmlrpc_url_prefix_;
  }

  //
  // dtor
  //
  ~Impl()
  {
    VLOG(IFM3D_TRACE) << "Dtor...";
    this->CancelSession();
  }

  std::unordered_map<std::string, std::string> HWInfo()
  {
    return this->value_struct_to_map(this->_XCallMain("getHWInfo"));
  }

  std::unordered_map<std::string, std::string> SWVersion()
  {
    return this->value_struct_to_map(this->_XCallMain("getSWVersion"));
  }

  std::unordered_map<std::string, std::string> DeviceInfo()
  {
    return this->value_struct_to_map(this->_XCallMain("getAllParameters"));
  }

  void Reboot(int mode)
  {
    this->_XCallMain("reboot", mode);
  }

  std::vector<ifm3d::app_entry_t> ApplicationList()
  {
    xmlrpc_c::value_array result(this->_XCallMain("getApplicationList"));
    std::vector<xmlrpc_c::value> const res_vec(result.vectorValueValue());

    std::vector<ifm3d::app_entry_t> retval;
    for (auto& entry : res_vec)
      {
        xmlrpc_c::value_struct const entry_st(entry);
        std::map<std::string, xmlrpc_c::value>
          entry_map(static_cast<std::map<std::string, xmlrpc_c::value> >
                    (entry_st));

        ifm3d::app_entry_t app;
        app.index = xmlrpc_c::value_int(entry_map["Index"]).cvalue();
        app.id = xmlrpc_c::value_int(entry_map["Id"]).cvalue();
        app.name = xmlrpc_c::value_string(entry_map["Name"]).cvalue();
        app.description =
          xmlrpc_c::value_string(entry_map["Description"]).cvalue();

        retval.push_back(app);
      }
    return retval;
  }

}; // end: class ifm3d::Camera::Impl

//================================================
// Camera class - the public interface
//================================================

ifm3d::Camera::Camera(const std::string& ip,
                      const std::uint16_t xmlrpc_port,
                      const std::string& password)
  : pImpl(new ifm3d::Camera::Impl(ip, xmlrpc_port, password))
{ }

ifm3d::Camera::~Camera() = default;

std::string
ifm3d::Camera::IP()
{
  return this->pImpl->IP();
}

std::uint16_t
ifm3d::Camera::XMLRPCPort()
{
  return this->pImpl->XMLRPCPort();
}

std::string
ifm3d::Camera::Password()
{
  return this->pImpl->Password();
}

std::string
ifm3d::Camera::SessionID()
{
  return this->pImpl->SessionID();
}

std::string
ifm3d::Camera::RequestSession()
{
  return this->pImpl->RequestSession();
}

bool
ifm3d::Camera::CancelSession()
{
  return this->pImpl->CancelSession();
}

int
ifm3d::Camera::Heartbeat(int hb)
{
  return this->pImpl->Heartbeat(hb);
}

void
ifm3d::Camera::Reboot(const ifm3d::Camera::boot_mode& mode)
{
  this->pImpl->Reboot(static_cast<int>(mode));
}

int
ifm3d::Camera::ActiveApplication()
{
  int active = -1;
  json jdev(this->pImpl->DeviceInfo());

  try
    {
      active = std::stoi(jdev["ActiveApplication"].get<std::string>());
    }
  catch (const std::exception& ex)
    {
      LOG(ERROR) << "Could not extract 'ActiveApplication' from JSON";
      LOG(ERROR) << ex.what();
      LOG(ERROR) << jdev.dump();

      throw ifm3d::error_t(IFM3D_JSON_ERROR);
    }

  return active;
}

json
ifm3d::Camera::ApplicationList()
{
  json retval; // list

  int active = this->ActiveApplication();
  std::vector<ifm3d::app_entry_t> apps = this->pImpl->ApplicationList();

  for (auto& app : apps)
    {
      json dict =
        {
          {"Index", app.index},
          {"Id", app.id},
          {"Name", app.name},
          {"Description", app.description},
          {"Active", app.index == active ? true : false}
        };

      retval.push_back(dict);
    }

  return retval;
}

json
ifm3d::Camera::ToJSON()
{
  auto t =
    std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::ostringstream time_buf;
  time_buf << std::ctime(&t);
  std::string time_s = time_buf.str();
  boost::algorithm::trim(time_s);

  json j =
    {
      {
       "ifm3d",
       {
         {std::string(IFM3D_LIBRARY_NAME) + "_version", IFM3D_VERSION},
         {"Date", time_s},
         {"HWInfo", json(this->pImpl->HWInfo())},
         {"SWVersion", json(this->pImpl->SWVersion())},
         {"Device", json(this->pImpl->DeviceInfo())}
       }
      }
    };

  return j;
}

std::string
ifm3d::Camera::ToJSONStr()
{
  return this->ToJSON().dump(2);
}
