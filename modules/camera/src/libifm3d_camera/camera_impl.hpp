// -*- c++ -*-
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

#ifndef __IFM3D_CAMERA_CAMERA_IMPL_HPP__
#define __IFM3D_CAMERA_CAMERA_IMPL_HPP__

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
#include <glog/logging.h>
#include <xmlrpc-c/client.hpp>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>

namespace ifm3d
{
  const int NET_WAIT = 3000; // millis

  const std::string XMLRPC_MAIN = "/api/rpc/v1/com.ifm.efector/";
  const std::string XMLRPC_SESSION = "session_$XXX/";
  const std::string XMLRPC_EDIT = "edit/";
  const std::string XMLRPC_DEVICE = "device/";
  const std::string XMLRPC_NET = "network/";
  const std::string XMLRPC_TIME = "time/";
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

  //============================================================
  // Impl interface
  //============================================================
  class Camera::Impl
  {
  public:
    Impl(const std::string& ip, const std::uint16_t xmlrpc_port,
         const std::string& password);
    ~Impl();

    // accessor/mutators
    std::string XPrefix();
    std::string IP();
    std::uint16_t XMLRPCPort();
    std::string Password();
    std::string SessionID();
    void SetSessionID(const std::string& id);

    //
    // public xmlrpc interface methods
    //

    // Main
    std::unordered_map<std::string, std::string> SWVersion();
    std::unordered_map<std::string, std::string> HWInfo();
    std::unordered_map<std::string, std::string> DeviceInfo();
    std::string DeviceParameter(const std::string& param);
    std::vector<std::string> TraceLogs(int count);
    void Reboot(int mode);
    std::vector<ifm3d::app_entry_t> ApplicationList();
    std::string RequestSession(
      const std::string& sid = ifm3d::DEFAULT_SESSION_ID);
    std::vector<std::uint8_t> UnitVectors();
    void ForceTrigger();

    // Session
    bool CancelSession();
    bool CancelSession(const std::string& sid);
    int Heartbeat(int hb);
    void SetOperatingMode(const ifm3d::Camera::operating_mode& mode);
    void SetTemporaryApplicationParameters(const std::unordered_map<std::string,
                                           std::string>& params);
    std::vector<std::uint8_t> ExportIFMConfig();
    std::vector<std::uint8_t> ExportIFMApp(int idx);
    void ImportIFMConfig(const std::vector<std::uint8_t>& bytes,
                         std::uint16_t flags);
    int ImportIFMApp(const std::vector<std::uint8_t>& bytes);

    // Edit Mode
    std::vector<std::string> ApplicationTypes();
    int CopyApplication(int idx);
    void DeleteApplication(int idx);
    int CreateApplication(const std::string& type);
    void FactoryReset();
    void EditApplication(int idx);
    void StopEditingApplication();

    // Device
    void SetDeviceParameter(const std::string& param, const std::string& val);
    void SaveDevice();
    void ActivatePassword(const std::string& password = "");
    void DisablePassword();

    // Network
    std::unordered_map<std::string, std::string> NetInfo();
    std::string NetParameter(const std::string& param);
    void SetNetParameter(const std::string& param, const std::string& val);
    void SaveNet();

    // Time
    std::unordered_map<std::string, std::string> TimeInfo();
    std::string TimeParameter(const std::string& param);
    void SetTimeParameter(const std::string& param, const std::string& val);
    void SaveTime();
    // A value less than 0 (the default) will set the time to "now"
    void SetCurrentTime(int epoch_seconds = -1);

    // Application
    std::unordered_map<std::string, std::string> AppInfo();
    std::string AppParameter(const std::string& param);
    void SetAppParameter(const std::string& param, const std::string& val);
    void SaveApp();

    // Imager
    std::unordered_map<std::string, std::string> ImagerInfo();
    std::string ImagerParameter(const std::string& param);
    void SetImagerParameter(const std::string& param, const std::string& val);
    std::vector<std::string> ImagerTypes();
    void ChangeImagerType(const std::string& type);

    // Spatial Filter
    std::unordered_map<std::string, std::string> SpatialFilterInfo();
    std::string SpatialFilterParameter(const std::string& param);
    void SetSpatialFilterParameter(const std::string& param,
                                   const std::string& val);

    // Temporal Filter
    std::unordered_map<std::string, std::string> TemporalFilterInfo();
    std::string TemporalFilterParameter(const std::string& param);
    void SetTemporalFilterParameter(const std::string& param,
                                    const std::string& val);

    // ---------------------------------------------
    // Session wrappers
    // ---------------------------------------------
    template <typename T>
    T WrapInEditSession(std::function<T()> f)
    {
      T retval;
      try
        {
          this->RequestSession();
          this->SetOperatingMode(ifm3d::Camera::operating_mode::EDIT);
          retval = f();
        }
      catch (const ifm3d::error_t& ex)
        {
          LOG(ERROR) << ex.what();
          this->CancelSession();
          throw;
        }
      this->CancelSession();
      return retval;
    }

    void WrapInEditSession(std::function<void()> f)
    {
      try
        {
          this->RequestSession();
          this->SetOperatingMode(ifm3d::Camera::operating_mode::EDIT);
          f();
        }
      catch (const ifm3d::error_t& ex)
        {
          LOG(ERROR) << ex.what();
          this->CancelSession();
          throw;
        }
        this->CancelSession();
    }

  private:
    std::string ip_;
    std::uint16_t xmlrpc_port_;
    std::string password_;
    std::string xmlrpc_url_prefix_;
    xmlrpc_c::clientPtr xclient_;
    std::mutex xclient_mutex_;
    std::string session_;
    std::mutex session_mutex_;

    // utilities for taking xmlrpc structures to STL structures
    std::unordered_map<std::string, std::string> const
    value_struct_to_map(const xmlrpc_c::value_struct& vs);

    std::unordered_map<std::string,
                       std::unordered_map<std::string, std::string> > const
    value_struct_to_map_of_maps(const xmlrpc_c::value_struct& vs);

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

    template <typename... Args>
    xmlrpc_c::value const
    _XCallEdit(const std::string& method, Args... args)
    {
      std::string url =
        this->XPrefix() + ifm3d::XMLRPC_MAIN + ifm3d::XMLRPC_SESSION +
        ifm3d::XMLRPC_EDIT;
      return this->_XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallDevice(const std::string& method, Args... args)
    {
      std::string url =
        this->XPrefix() + ifm3d::XMLRPC_MAIN + ifm3d::XMLRPC_SESSION +
        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE;
      return this->_XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallNet(const std::string& method, Args... args)
    {
      std::string url =
        this->XPrefix() + ifm3d::XMLRPC_MAIN + ifm3d::XMLRPC_SESSION +
        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE + ifm3d::XMLRPC_NET;
      return this->_XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallTime(const std::string& method, Args... args)
    {
      std::string url =
        this->XPrefix() + ifm3d::XMLRPC_MAIN + ifm3d::XMLRPC_SESSION +
        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE + ifm3d::XMLRPC_TIME;
      return this->_XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallApp(const std::string& method, Args... args)
    {
      std::string url =
        this->XPrefix() + ifm3d::XMLRPC_MAIN + ifm3d::XMLRPC_SESSION +
        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP;
      return this->_XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallImager(const std::string& method, Args... args)
    {
      std::string url =
        this->XPrefix() + ifm3d::XMLRPC_MAIN + ifm3d::XMLRPC_SESSION +
        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP + ifm3d::XMLRPC_IMAGER;
      return this->_XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallSpatialFilter(const std::string& method, Args... args)
    {
      std::string url =
        this->XPrefix() + ifm3d::XMLRPC_MAIN + ifm3d::XMLRPC_SESSION +
        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP + ifm3d::XMLRPC_IMAGER +
        ifm3d::XMLRPC_SPATIALFILTER;
      return this->_XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallTemporalFilter(const std::string& method, Args... args)
    {
      std::string url =
        this->XPrefix() + ifm3d::XMLRPC_MAIN + ifm3d::XMLRPC_SESSION +
        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP + ifm3d::XMLRPC_IMAGER +
        ifm3d::XMLRPC_TEMPORALFILTER;
      return this->_XCall(url, method, args...);
    }

  }; // end: class Camera::Impl

} // end: namespace ifm3d

//============================================================
// Impl - Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------

ifm3d::Camera::Impl::Impl(const std::string& ip,
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
  // Needed for fetching unit vectors over xmlrpc for O3X
  VLOG(IFM3D_TRACE) << "Increasing XML-RPC response size limit...";
  xmlrpc_limit_set(XMLRPC_XML_SIZE_LIMIT_ID,
                   XMLRPC_XML_SIZE_LIMIT_DEFAULT * 2);

  VLOG(IFM3D_TRACE) << "Initializing Camera: ip="
                    << this->IP()
                    << ", xmlrpc_port=" << this->XMLRPCPort()
                    << ", password=" << this->Password();
  VLOG(IFM3D_TRACE) << "XMLRPC URL Prefix=" << this->xmlrpc_url_prefix_;
}

ifm3d::Camera::Impl::~Impl()
{
  VLOG(IFM3D_TRACE) << "Dtor...";
  this->CancelSession();
}

//-------------------------------------
// Accessor/mutators
//-------------------------------------

std::string
ifm3d::Camera::Impl::XPrefix()
{
  return this->xmlrpc_url_prefix_;
}

std::string
ifm3d::Camera::Impl::IP()
{
  return this->ip_;
}

std::uint16_t
ifm3d::Camera::Impl::XMLRPCPort()
{
  return this->xmlrpc_port_;
}

std::string
ifm3d::Camera::Impl::Password()
{
  return this->password_;
}

std::string
ifm3d::Camera::Impl::SessionID()
{
  std::lock_guard<std::mutex> lock(this->session_mutex_);
  return this->session_;
}

void
ifm3d::Camera::Impl::SetSessionID(const std::string& id)
{
  std::lock_guard<std::mutex> lock(this->session_mutex_);
  this->session_ = id;
}

// ---------------------------------------------
// Conversions from XMLRPC data structures to
// STL data structures
// ---------------------------------------------
std::unordered_map<std::string, std::string> const
ifm3d::Camera::Impl::value_struct_to_map(const xmlrpc_c::value_struct& vs)
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
ifm3d::Camera::Impl::value_struct_to_map_of_maps(
  const xmlrpc_c::value_struct& vs)
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

// =============================================
// Public XMLRPC interface - worker methods
// =============================================

// ---------------------------------------------
// Main (no edit session necessary)
// ---------------------------------------------

std::string
ifm3d::Camera::Impl::DeviceParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallMain("getParameter", param.c_str())).cvalue();
}

std::vector<std::string>
ifm3d::Camera::Impl::TraceLogs(int count)
{
  xmlrpc_c::value_array result(this->_XCallMain("getTraceLogs", count));
  std::vector<xmlrpc_c::value> const res_vec(result.vectorValueValue());

  std::vector<std::string> retval;
  for (auto& entry : res_vec)
    {
      xmlrpc_c::value_string const entry_str(entry);
      retval.push_back(static_cast<std::string>(entry_str));
    }
  return retval;
}

std::unordered_map<std::string, std::string>
ifm3d::Camera::Impl::HWInfo()
{
  return this->value_struct_to_map(this->_XCallMain("getHWInfo"));
}

std::unordered_map<std::string, std::string>
ifm3d::Camera::Impl::SWVersion()
{
  return this->value_struct_to_map(this->_XCallMain("getSWVersion"));
}

std::unordered_map<std::string, std::string>
ifm3d::Camera::Impl::DeviceInfo()
{
  return this->value_struct_to_map(this->_XCallMain("getAllParameters"));
}

void
ifm3d::Camera::Impl::Reboot(int mode)
{
  this->_XCallMain("reboot", mode);
}

std::vector<ifm3d::app_entry_t>
ifm3d::Camera::Impl::ApplicationList()
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

std::string
ifm3d::Camera::Impl::RequestSession(const std::string& sid)
{
  xmlrpc_c::value_string val_str(
    this->_XCallMain("requestSession",
                     this->Password().c_str(), sid));

  this->SetSessionID(static_cast<std::string>(val_str));
  this->Heartbeat(ifm3d::MAX_HEARTBEAT);
  return this->SessionID();
}

std::vector<std::uint8_t>
ifm3d::Camera::Impl::UnitVectors()
{
  const xmlrpc_c::value_bytestring v_bytes =
    this->_XCallMain("getUnitVectors");

  return v_bytes.vectorUcharValue();
}

void
ifm3d::Camera::Impl::ForceTrigger()
{
  this->_XCallMain("trigger");
}

// ---------------------------------------------
// Session
// ---------------------------------------------
bool
ifm3d::Camera::Impl::CancelSession(const std::string& sid)
{
  if (sid == this->SessionID())
    {
      return this->CancelSession();
    }

  std::string old_sid = this->SessionID();
  this->SetSessionID(sid);
  bool retval = this->CancelSession();
  this->SetSessionID(old_sid);
  return retval;
}

bool
ifm3d::Camera::Impl::CancelSession()
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
      LOG(WARNING) << "Failed to cancel session: "
                   << this->SessionID() << " -> "
                   << ex.what();

      if (ex.code() == IFM3D_XMLRPC_OBJ_NOT_FOUND)
        {
          // this code path gets tickled when a session
          // is closed implicitly as a result of applying
          // new network setting on the sensor.
          this->SetSessionID("");
          retval = true;
        }
      else
        {
          retval = false;
        }
    }

  return retval;
}

int
ifm3d::Camera::Impl::Heartbeat(int hb)
{
  xmlrpc_c::value_int v_int(this->_XCallSession("heartbeat", hb));
  return v_int.cvalue();
}

void
ifm3d::Camera::Impl::SetOperatingMode(const ifm3d::Camera::operating_mode& mode)
{
  this->_XCallSession("setOperatingMode", static_cast<int>(mode));
}

void
ifm3d::Camera::Impl::SetTemporaryApplicationParameters(
  const std::unordered_map<std::string, std::string>& params)
{
  std::map<std::string, xmlrpc_c::value> param_map;

  for (const auto& kv : params)
    {
      std::pair<std::string, xmlrpc_c::value> member;

      if ((kv.first == "imager_001/ExposureTime") ||
          (kv.first == "imager_001/ExposureTimeRatio") ||
          (kv.first == "imager_001/Channel"))
        {
          member =
            std::make_pair(kv.first, xmlrpc_c::value_int(std::stoi(kv.second)));
          param_map.insert(member);
        }
      else
        {
        throw(ifm3d::error_t(IFM3D_INVALID_PARAM));
        }
    }

  xmlrpc_c::value_struct const params_st(param_map);
  this->_XCallSession("setTemporaryApplicationParameters",
                      params_st);
}

std::vector<std::uint8_t>
ifm3d::Camera::Impl::ExportIFMConfig()
{
  const xmlrpc_c::value_bytestring v_bytes =
    this->_XCallSession("exportConfig");

  return v_bytes.vectorUcharValue();
}

std::vector<std::uint8_t>
ifm3d::Camera::Impl::ExportIFMApp(int idx)
{
  const xmlrpc_c::value_bytestring v_bytes =
    this->_XCallSession("exportApplication", idx);

  return v_bytes.vectorUcharValue();
}

void
ifm3d::Camera::Impl::ImportIFMConfig(const std::vector<std::uint8_t>& bytes,
                                     std::uint16_t flags)
{
  this->_XCallSession("importConfig", bytes, flags);
}

int
ifm3d::Camera::Impl::ImportIFMApp(const std::vector<std::uint8_t>& bytes)
{
  xmlrpc_c::value_int v_int(this->_XCallSession("importApplication", bytes));
  return v_int.cvalue();
}

// ---------------------------------------------
// Edit Mode
// ---------------------------------------------

std::vector<std::string>
ifm3d::Camera::Impl::ApplicationTypes()
{
  xmlrpc_c::value_array a = this->_XCallEdit("availableApplicationTypes");

  std::vector<xmlrpc_c::value> v = a.vectorValueValue();
  std::vector<std::string> retval;
  for (auto& vs : v)
    {
      retval.push_back(static_cast<std::string>(xmlrpc_c::value_string(vs)));
    }

  return retval;
}

int
ifm3d::Camera::Impl::CopyApplication(int idx)
{
  xmlrpc_c::value_int v_int(this->_XCallEdit("copyApplication", idx));
  return v_int.cvalue();
}

void
ifm3d::Camera::Impl::DeleteApplication(int idx)
{
  this->_XCallEdit("deleteApplication", idx);
}

int
ifm3d::Camera::Impl::CreateApplication(const std::string& type)
{
  xmlrpc_c::value_int
    v_int(this->_XCallEdit("createApplication", type.c_str()));
  return v_int.cvalue();
}

void
ifm3d::Camera::Impl::FactoryReset()
{
  this->_XCallEdit("factoryReset");
}

void
ifm3d::Camera::Impl::EditApplication(int idx)
{
  this->_XCallEdit("editApplication", idx);
}

void
ifm3d::Camera::Impl::StopEditingApplication()
{
  this->_XCallEdit("stopEditingApplication");
}

// ---------------------------------------------
// Device
// ---------------------------------------------

void
ifm3d::Camera::Impl::SetDeviceParameter(const std::string& param,
                                        const std::string& val)
{
  this->_XCallDevice("setParameter", param.c_str(), val.c_str());
}

void
ifm3d::Camera::Impl::SaveDevice()
{
  this->_XCallDevice("save");
}

void
ifm3d::Camera::Impl::ActivatePassword(const std::string& password)
{
  this->_XCallDevice("activatePassword", password.c_str());
}

void
ifm3d::Camera::Impl::DisablePassword()
{
  this->_XCallDevice("disablePassword");
}

// ---------------------------------------------
// Network
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::Camera::Impl::NetInfo()
{
  return this->value_struct_to_map(this->_XCallNet("getAllParameters"));
}

std::string
ifm3d::Camera::Impl::NetParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallNet("getParameter", param.c_str())).cvalue();
}

void
ifm3d::Camera::Impl::SetNetParameter(const std::string& param,
                                     const std::string& val)
{
  this->_XCallNet("setParameter", param.c_str(), val.c_str());
}

void
ifm3d::Camera::Impl::SaveNet()
{
  this->_XCallNet("saveAndActivateConfig");
}

// ---------------------------------------------
// Time
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::Camera::Impl::TimeInfo()
{
  return this->value_struct_to_map(this->_XCallTime("getAllParameters"));
}

std::string
ifm3d::Camera::Impl::TimeParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallTime("getParameter", param.c_str())).cvalue();
}

void
ifm3d::Camera::Impl::SetTimeParameter(const std::string& param,
                                      const std::string& val)
{
  this->_XCallTime("setParameter", param.c_str(), val.c_str());
}

void ifm3d::Camera::Impl::SaveTime()
{
  this->_XCallTime("saveAndActivateConfig");
}


void
ifm3d::Camera::Impl::SetCurrentTime(int epoch_seconds)
{
  if (epoch_seconds < 0)
    {
      epoch_seconds = static_cast<int>(
                std::chrono::seconds(std::time(nullptr)).count());
    }

  this->_XCallTime("setCurrentTime", epoch_seconds);
}


// ---------------------------------------------
// Application
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::Camera::Impl::AppInfo()
{
  return this->value_struct_to_map(this->_XCallApp("getAllParameters"));
}

std::string
ifm3d::Camera::Impl::AppParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallApp("getParameter", param.c_str())).cvalue();
}

void
ifm3d::Camera::Impl::SetAppParameter(const std::string& param,
                                     const std::string& val)
{
  this->_XCallApp("setParameter", param.c_str(), val.c_str());
}

void
ifm3d::Camera::Impl::SaveApp()
{
  this->_XCallApp("save");
}

// ---------------------------------------------
// Imager
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::Camera::Impl::ImagerInfo()
{
  return this->value_struct_to_map(this->_XCallImager("getAllParameters"));
}

std::string
ifm3d::Camera::Impl::ImagerParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallImager("getParameter", param.c_str())).cvalue();
}

void
ifm3d::Camera::Impl::SetImagerParameter(const std::string& param,
                                        const std::string& val)
{
  this->_XCallImager("setParameter", param.c_str(), val.c_str());
}

std::vector<std::string>
ifm3d::Camera::Impl::ImagerTypes()
{
  xmlrpc_c::value_array a = this->_XCallImager("availableTypes");

  std::vector<xmlrpc_c::value> v = a.vectorValueValue();
  std::vector<std::string> retval;
  for (auto& vs : v)
    {
      retval.push_back(static_cast<std::string>(xmlrpc_c::value_string(vs)));
    }

  return retval;
}

void
ifm3d::Camera::Impl::ChangeImagerType(const std::string& type)
{
  this->_XCallImager("changeType", type.c_str());
}

// ---------------------------------------------
// Spatial Filter
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::Camera::Impl::SpatialFilterInfo()
{
  return this->value_struct_to_map(
           this->_XCallSpatialFilter("getAllParameters"));
}

std::string
ifm3d::Camera::Impl::SpatialFilterParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallSpatialFilter("getParameter", param.c_str())).cvalue();
}

void
ifm3d::Camera::Impl::SetSpatialFilterParameter(const std::string& param,
                                               const std::string& val)
{
  this->_XCallSpatialFilter("setParameter", param.c_str(), val.c_str());
}

// ---------------------------------------------
// Temporal Filter
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::Camera::Impl::TemporalFilterInfo()
{
  return this->value_struct_to_map(
           this->_XCallTemporalFilter("getAllParameters"));
}

std::string
ifm3d::Camera::Impl::TemporalFilterParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallTemporalFilter("getParameter", param.c_str())).cvalue();
}

void
ifm3d::Camera::Impl::SetTemporalFilterParameter(const std::string& param,
                                               const std::string& val)
{
  this->_XCallTemporalFilter("setParameter", param.c_str(), val.c_str());
}

#endif // __IFM3D_CAMERA_CAMERA_IMPL_HPP__
