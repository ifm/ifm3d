// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_LEGACY_DEVICE_IMPL_HPP
#define IFM3D_LEGACY_DEVICE_IMPL_HPP

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
#include <xmlrpc_wrapper.hpp>

namespace ifm3d
{
  const std::string XMLRPC_SESSION = "session_$XXX/";
  const std::string XMLRPC_EDIT = "edit/";
  const std::string XMLRPC_DEVICE = "device/";
  const std::string XMLRPC_NET = "network/";
  const std::string XMLRPC_TIME = "time/";
  const std::string XMLRPC_APP = "application/";
  const std::string XMLRPC_IMAGER = "imager_001/";
  const std::string XMLRPC_SPATIALFILTER = "spatialfilter";
  const std::string XMLRPC_TEMPORALFILTER = "temporalfilter";

  using app_entry_t = struct
  {
    int index;
    int id;
    std::string name;
    std::string description;
  };

  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_DEVICE_LOCAL LegacyDevice::Impl
  {
  public:
    Impl(std::shared_ptr<XMLRPCWrapper> xwrapper, const std::string& password);
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
    std::vector<ifm3d::app_entry_t> ApplicationList();
    std::string RequestSession(
      const std::string& sid = ifm3d::DEFAULT_SESSION_ID);
    std::vector<std::uint8_t> UnitVectors();
    void ForceTrigger();

    // Session
    bool CancelSession();
    bool CancelSession(const std::string& sid);
    int Heartbeat(int hb);
    void SetOperatingMode(const ifm3d::LegacyDevice::operating_mode& mode);
    void SetTemporaryApplicationParameters(
      const std::unordered_map<std::string, std::string>& params);
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
    T
    WrapInEditSession(std::function<T()> f)
    {
      T retval;
      try
        {
          this->RequestSession();
          this->SetOperatingMode(ifm3d::LegacyDevice::operating_mode::EDIT);
          retval = f();
        }
      catch (const ifm3d::Error& ex)
        {
          LOG_ERROR(ex.what());
          this->CancelSession();
          throw;
        }
      this->CancelSession();
      return retval;
    }

    void
    WrapInEditSession(std::function<void()> f)
    {
      try
        {
          this->RequestSession();
          this->SetOperatingMode(ifm3d::LegacyDevice::operating_mode::EDIT);
          f();
        }
      catch (const ifm3d::Error& ex)
        {
          LOG_ERROR(ex.what());
          this->CancelSession();
          throw;
        }
      this->CancelSession();
    }

  private:
    std::shared_ptr<XMLRPCWrapper> xwrapper_;
    std::string password_;
    std::string session_;
    std::mutex session_mutex_;

    // ---------------------------------------------
    // _XCall wrappers
    // ---------------------------------------------

    std::string
    _XSession()
    {
      return std::regex_replace(ifm3d::XMLRPC_SESSION,
                                std::regex("\\$XXX"),
                                this->SessionID());
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallSession(const std::string& method, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN + _XSession();
      return this->xwrapper_->XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallEdit(const std::string& method, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN + _XSession() +
                        ifm3d::XMLRPC_EDIT;
      return this->xwrapper_->XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallDevice(const std::string& method, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN + _XSession() +
                        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE;
      return this->xwrapper_->XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallNet(const std::string& method, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN + _XSession() +
                        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE +
                        ifm3d::XMLRPC_NET;
      return this->xwrapper_->XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallTime(const std::string& method, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN + _XSession() +
                        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE +
                        ifm3d::XMLRPC_TIME;
      return this->xwrapper_->XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallApp(const std::string& method, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN + _XSession() +
                        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP;
      return this->xwrapper_->XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallImager(const std::string& method, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN + _XSession() +
                        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP +
                        ifm3d::XMLRPC_IMAGER;
      return this->xwrapper_->XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallSpatialFilter(const std::string& method, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN + _XSession() +
                        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP +
                        ifm3d::XMLRPC_IMAGER + ifm3d::XMLRPC_SPATIALFILTER;
      return this->xwrapper_->XCall(url, method, args...);
    }

    template <typename... Args>
    xmlrpc_c::value const
    _XCallTemporalFilter(const std::string& method, Args... args)
    {
      std::string url = this->XPrefix() + ifm3d::XMLRPC_MAIN + _XSession() +
                        ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP +
                        ifm3d::XMLRPC_IMAGER + ifm3d::XMLRPC_TEMPORALFILTER;
      return this->xwrapper_->XCall(url, method, args...);
    }

  }; // end: class Camera::Impl

} // end: namespace ifm3d

//============================================================
// Impl - Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------

ifm3d::LegacyDevice::Impl::Impl(std::shared_ptr<XMLRPCWrapper> xwrapper,
                                const std::string& password)
  : xwrapper_(std::move(xwrapper)),
    password_(password),
    session_("")
{
  // Needed for fetching unit vectors over xmlrpc for O3X
  LOG_VERBOSE("Increasing XML-RPC response size limit...");
  xmlrpc_limit_set(XMLRPC_XML_SIZE_LIMIT_ID,
                   XMLRPC_XML_SIZE_LIMIT_DEFAULT * 2);
}

ifm3d::LegacyDevice::Impl::~Impl()
{
  LOG_VERBOSE("Dtor...");
  this->CancelSession();
}

//-------------------------------------
// Accessor/mutators
//-------------------------------------

std::string
ifm3d::LegacyDevice::Impl::XPrefix()
{
  return this->xwrapper_->XPrefix();
}

std::string
ifm3d::LegacyDevice::Impl::IP()
{
  return this->xwrapper_->IP();
}

std::uint16_t
ifm3d::LegacyDevice::Impl::XMLRPCPort()
{
  return this->xwrapper_->XMLRPCPort();
}

std::string
ifm3d::LegacyDevice::Impl::Password()
{
  return this->password_;
}

std::string
ifm3d::LegacyDevice::Impl::SessionID()
{
  std::lock_guard<std::mutex> lock(this->session_mutex_);
  return this->session_;
}

void
ifm3d::LegacyDevice::Impl::SetSessionID(const std::string& id)
{
  std::lock_guard<std::mutex> lock(this->session_mutex_);
  this->session_ = id;
}

// =============================================
// Public XMLRPC interface - worker methods
// =============================================

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::HWInfo()
{
  return this->xwrapper_->value_struct_to_map(
    this->xwrapper_->XCallMain("getHWInfo"));
}

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::SWVersion()
{
  return this->xwrapper_->value_struct_to_map(
    this->xwrapper_->XCallMain("getSWVersion"));
}

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::DeviceInfo()
{
  return this->xwrapper_->value_struct_to_map(
    this->xwrapper_->XCallMain("getAllParameters"));
}

std::vector<ifm3d::app_entry_t>
ifm3d::LegacyDevice::Impl::ApplicationList()
{
  xmlrpc_c::value_array result(
    this->xwrapper_->XCallMain("getApplicationList"));
  std::vector<xmlrpc_c::value> const res_vec(result.vectorValueValue());

  std::vector<ifm3d::app_entry_t> retval;
  for (auto& entry : res_vec)
    {
      xmlrpc_c::value_struct const entry_st(entry);
      std::map<std::string, xmlrpc_c::value> entry_map(
        static_cast<std::map<std::string, xmlrpc_c::value>>(entry_st));

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
ifm3d::LegacyDevice::Impl::RequestSession(const std::string& sid)
{
  xmlrpc_c::value_string val_str(
    this->xwrapper_->XCallMain("requestSession",
                               this->Password().c_str(),
                               sid));

  this->SetSessionID(static_cast<std::string>(val_str));
  this->Heartbeat(ifm3d::MAX_HEARTBEAT);
  return this->SessionID();
}

std::vector<std::uint8_t>
ifm3d::LegacyDevice::Impl::UnitVectors()
{
  const xmlrpc_c::value_bytestring v_bytes =
    this->xwrapper_->XCallMain("getUnitVectors");

  return v_bytes.vectorUcharValue();
}

void
ifm3d::LegacyDevice::Impl::ForceTrigger()
{
  this->xwrapper_->XCallMain("trigger");
}

// ---------------------------------------------
// Session
// ---------------------------------------------
bool
ifm3d::LegacyDevice::Impl::CancelSession(const std::string& sid)
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
ifm3d::LegacyDevice::Impl::CancelSession()
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
  catch (const ifm3d::Error& ex)
    {
      LOG_WARNING("Failed to cancel session: {} -> {}",
                  this->SessionID(),
                  ex.what());

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
ifm3d::LegacyDevice::Impl::Heartbeat(int hb)
{
  xmlrpc_c::value_int v_int(this->_XCallSession("heartbeat", hb));
  return v_int.cvalue();
}

void
ifm3d::LegacyDevice::Impl::SetOperatingMode(
  const ifm3d::LegacyDevice::operating_mode& mode)
{
  this->_XCallSession("setOperatingMode", static_cast<int>(mode));
}

void
ifm3d::LegacyDevice::Impl::SetTemporaryApplicationParameters(
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
          member = std::make_pair(kv.first,
                                  xmlrpc_c::value_int(std::stoi(kv.second)));
          param_map.insert(member);
        }
      else
        {
          throw(ifm3d::Error(IFM3D_INVALID_PARAM));
        }
    }

  xmlrpc_c::value_struct const params_st(param_map);
  this->_XCallSession("setTemporaryApplicationParameters", params_st);
}

std::vector<std::uint8_t>
ifm3d::LegacyDevice::Impl::ExportIFMConfig()
{
  const xmlrpc_c::value_bytestring v_bytes =
    this->_XCallSession("exportConfig");

  return v_bytes.vectorUcharValue();
}

std::vector<std::uint8_t>
ifm3d::LegacyDevice::Impl::ExportIFMApp(int idx)
{
  const xmlrpc_c::value_bytestring v_bytes =
    this->_XCallSession("exportApplication", idx);

  return v_bytes.vectorUcharValue();
}

void
ifm3d::LegacyDevice::Impl::ImportIFMConfig(
  const std::vector<std::uint8_t>& bytes,
  std::uint16_t flags)
{
  this->_XCallSession("importConfig", bytes, flags);
}

int
ifm3d::LegacyDevice::Impl::ImportIFMApp(const std::vector<std::uint8_t>& bytes)
{
  xmlrpc_c::value_int v_int(this->_XCallSession("importApplication", bytes));
  return v_int.cvalue();
}

// ---------------------------------------------
// Edit Mode
// ---------------------------------------------

std::vector<std::string>
ifm3d::LegacyDevice::Impl::ApplicationTypes()
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
ifm3d::LegacyDevice::Impl::CopyApplication(int idx)
{
  xmlrpc_c::value_int v_int(this->_XCallEdit("copyApplication", idx));
  return v_int.cvalue();
}

void
ifm3d::LegacyDevice::Impl::DeleteApplication(int idx)
{
  this->_XCallEdit("deleteApplication", idx);
}

int
ifm3d::LegacyDevice::Impl::CreateApplication(const std::string& type)
{
  xmlrpc_c::value_int v_int(
    this->_XCallEdit("createApplication", type.c_str()));
  return v_int.cvalue();
}

void
ifm3d::LegacyDevice::Impl::FactoryReset()
{
  this->_XCallEdit("factoryReset");
}

void
ifm3d::LegacyDevice::Impl::EditApplication(int idx)
{
  this->_XCallEdit("editApplication", idx);
}

void
ifm3d::LegacyDevice::Impl::StopEditingApplication()
{
  this->_XCallEdit("stopEditingApplication");
}

// ---------------------------------------------
// Device
// ---------------------------------------------

void
ifm3d::LegacyDevice::Impl::SetDeviceParameter(const std::string& param,
                                              const std::string& val)
{
  this->_XCallDevice("setParameter", param.c_str(), val.c_str());
}

void
ifm3d::LegacyDevice::Impl::SaveDevice()
{
  this->_XCallDevice("save");
}

void
ifm3d::LegacyDevice::Impl::ActivatePassword(const std::string& password)
{
  this->_XCallDevice("activatePassword", password.c_str());
}

void
ifm3d::LegacyDevice::Impl::DisablePassword()
{
  this->_XCallDevice("disablePassword");
}

// ---------------------------------------------
// Network
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::NetInfo()
{
  return this->xwrapper_->value_struct_to_map(
    this->_XCallNet("getAllParameters"));
}

std::string
ifm3d::LegacyDevice::Impl::NetParameter(const std::string& param)
{
  return xmlrpc_c::value_string(this->_XCallNet("getParameter", param.c_str()))
    .cvalue();
}

void
ifm3d::LegacyDevice::Impl::SetNetParameter(const std::string& param,
                                           const std::string& val)
{
  this->_XCallNet("setParameter", param.c_str(), val.c_str());
}

void
ifm3d::LegacyDevice::Impl::SaveNet()
{
  this->_XCallNet("saveAndActivateConfig");
}

// ---------------------------------------------
// Time
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::TimeInfo()
{
  return this->xwrapper_->value_struct_to_map(
    this->_XCallTime("getAllParameters"));
}

std::string
ifm3d::LegacyDevice::Impl::TimeParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallTime("getParameter", param.c_str()))
    .cvalue();
}

void
ifm3d::LegacyDevice::Impl::SetTimeParameter(const std::string& param,
                                            const std::string& val)
{
  this->_XCallTime("setParameter", param.c_str(), val.c_str());
}

void
ifm3d::LegacyDevice::Impl::SaveTime()
{
  this->_XCallTime("saveAndActivateConfig");
}

void
ifm3d::LegacyDevice::Impl::SetCurrentTime(int epoch_seconds)
{
  if (epoch_seconds < 0)
    {
      epoch_seconds =
        static_cast<int>(std::chrono::seconds(std::time(nullptr)).count());
    }

  this->_XCallTime("setCurrentTime", epoch_seconds);
}

// ---------------------------------------------
// Application
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::AppInfo()
{
  return this->xwrapper_->value_struct_to_map(
    this->_XCallApp("getAllParameters"));
}

std::string
ifm3d::LegacyDevice::Impl::AppParameter(const std::string& param)
{
  return xmlrpc_c::value_string(this->_XCallApp("getParameter", param.c_str()))
    .cvalue();
}

void
ifm3d::LegacyDevice::Impl::SetAppParameter(const std::string& param,
                                           const std::string& val)
{
  this->_XCallApp("setParameter", param.c_str(), val.c_str());
}

void
ifm3d::LegacyDevice::Impl::SaveApp()
{
  this->_XCallApp("save");
}

// ---------------------------------------------
// Imager
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::ImagerInfo()
{
  return this->xwrapper_->value_struct_to_map(
    this->_XCallImager("getAllParameters"));
}

std::string
ifm3d::LegacyDevice::Impl::ImagerParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallImager("getParameter", param.c_str()))
    .cvalue();
}

void
ifm3d::LegacyDevice::Impl::SetImagerParameter(const std::string& param,
                                              const std::string& val)
{
  this->_XCallImager("setParameter", param.c_str(), val.c_str());
}

std::vector<std::string>
ifm3d::LegacyDevice::Impl::ImagerTypes()
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
ifm3d::LegacyDevice::Impl::ChangeImagerType(const std::string& type)
{
  this->_XCallImager("changeType", type.c_str());
}

// ---------------------------------------------
// Spatial Filter
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::SpatialFilterInfo()
{
  return this->xwrapper_->value_struct_to_map(
    this->_XCallSpatialFilter("getAllParameters"));
}

std::string
ifm3d::LegacyDevice::Impl::SpatialFilterParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallSpatialFilter("getParameter", param.c_str()))
    .cvalue();
}

void
ifm3d::LegacyDevice::Impl::SetSpatialFilterParameter(const std::string& param,
                                                     const std::string& val)
{
  this->_XCallSpatialFilter("setParameter", param.c_str(), val.c_str());
}

// ---------------------------------------------
// Temporal Filter
// ---------------------------------------------

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::TemporalFilterInfo()
{
  return this->xwrapper_->value_struct_to_map(
    this->_XCallTemporalFilter("getAllParameters"));
}

std::string
ifm3d::LegacyDevice::Impl::TemporalFilterParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->_XCallTemporalFilter("getParameter", param.c_str()))
    .cvalue();
}

void
ifm3d::LegacyDevice::Impl::SetTemporalFilterParameter(const std::string& param,
                                                      const std::string& val)
{
  this->_XCallTemporalFilter("setParameter", param.c_str(), val.c_str());
}

#endif // IFM3D_LEGACY_DEVICE_IMPL_HPP