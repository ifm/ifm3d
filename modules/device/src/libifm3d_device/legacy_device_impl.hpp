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
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/common/logging/log.h>
#include <xmlrpc.hpp>

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
  class IFM3D_NO_EXPORT LegacyDevice::Impl
  {
  public:
    Impl(std::shared_ptr<XMLRPC> xwrapper, const std::string& password);
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
    std::shared_ptr<XMLRPC> xwrapper_;
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
    XMLRPCValue const
    _XCallSession(const std::string& method, Args... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + _XSession();
      return this->xwrapper_->XCall(path, method, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    _XCallEdit(const std::string& method, Args... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + _XSession() + ifm3d::XMLRPC_EDIT;
      return this->xwrapper_->XCall(path, method, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    _XCallDevice(const std::string& method, Args... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + _XSession() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE;
      return this->xwrapper_->XCall(path, method, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    _XCallNet(const std::string& method, Args... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + _XSession() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE +
                         ifm3d::XMLRPC_NET;
      return this->xwrapper_->XCall(path, method, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    _XCallTime(const std::string& method, Args... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + _XSession() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE +
                         ifm3d::XMLRPC_TIME;
      return this->xwrapper_->XCall(path, method, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    _XCallApp(const std::string& method, Args... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + _XSession() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP;
      return this->xwrapper_->XCall(path, method, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    _XCallImager(const std::string& method, Args... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + _XSession() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP +
                         ifm3d::XMLRPC_IMAGER;
      return this->xwrapper_->XCall(path, method, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    _XCallSpatialFilter(const std::string& method, Args... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + _XSession() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP +
                         ifm3d::XMLRPC_IMAGER + ifm3d::XMLRPC_SPATIALFILTER;
      return this->xwrapper_->XCall(path, method, args...);
    }

    template <typename... Args>
    XMLRPCValue const
    _XCallTemporalFilter(const std::string& method, Args... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + _XSession() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP +
                         ifm3d::XMLRPC_IMAGER + ifm3d::XMLRPC_TEMPORALFILTER;
      return this->xwrapper_->XCall(path, method, args...);
    }

  }; // end: class Camera::Impl

} // end: namespace ifm3d

//============================================================
// Impl - Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------

ifm3d::LegacyDevice::Impl::Impl(std::shared_ptr<XMLRPC> xwrapper,
                                const std::string& password)
  : xwrapper_(std::move(xwrapper)),
    password_(password),
    session_("")
{
  // Needed for fetching unit vectors over xmlrpc for O3X
  LOG_VERBOSE("Increasing XML-RPC response size limit...");
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
  return this->xwrapper_->XCallMain("getHWInfo").ToStringMap();
}

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::SWVersion()
{
  return this->xwrapper_->XCallMain("getSWVersion").ToStringMap();
}

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::DeviceInfo()
{
  return this->xwrapper_->XCallMain("getAllParameters").ToStringMap();
}

std::vector<ifm3d::app_entry_t>
ifm3d::LegacyDevice::Impl::ApplicationList()
{
  auto result(this->xwrapper_->XCallMain("getApplicationList").AsArray());

  std::vector<ifm3d::app_entry_t> retval;
  for (auto& entry : result)
    {
      auto entry_map = entry.AsMap();

      ifm3d::app_entry_t app;
      app.index = entry_map["Index"].AsInt();
      app.id = entry_map["Id"].AsInt();
      app.name = entry_map["Name"].AsString();
      app.description = entry_map["Description"].AsString();

      retval.push_back(app);
    }
  return retval;
}

std::string
ifm3d::LegacyDevice::Impl::RequestSession(const std::string& sid)
{
  auto result =
    this->xwrapper_->XCallMain("requestSession", this->Password().c_str(), sid)
      .AsString();

  this->SetSessionID(static_cast<std::string>(result));
  this->Heartbeat(ifm3d::MAX_HEARTBEAT);
  return this->SessionID();
}

std::vector<std::uint8_t>
ifm3d::LegacyDevice::Impl::UnitVectors()
{
  return this->xwrapper_->XCallMain("getUnitVectors").AsByteArray();
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
  return this->_XCallSession("heartbeat", hb).AsInt();
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
  std::unordered_map<std::string, XMLRPCValue> param_map;

  for (const auto& kv : params)
    {
      std::pair<std::string, XMLRPCValue> member;

      if ((kv.first == "imager_001/ExposureTime") ||
          (kv.first == "imager_001/ExposureTimeRatio") ||
          (kv.first == "imager_001/Channel"))
        {
          param_map.emplace(kv.first, XMLRPCValue(std::stoi(kv.second)));
        }
      else
        {
          throw(ifm3d::Error(IFM3D_INVALID_PARAM));
        }
    }

  this->_XCallSession("setTemporaryApplicationParameters",
                      XMLRPCValue(param_map));
}

std::vector<std::uint8_t>
ifm3d::LegacyDevice::Impl::ExportIFMConfig()
{
  return this->_XCallSession("exportConfig").AsByteArray();
}

std::vector<std::uint8_t>
ifm3d::LegacyDevice::Impl::ExportIFMApp(int idx)
{
  return this->_XCallSession("exportApplication", idx).AsByteArray();
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
  return this->_XCallSession("importApplication", bytes).AsInt();
}

// ---------------------------------------------
// Edit Mode
// ---------------------------------------------

std::vector<std::string>
ifm3d::LegacyDevice::Impl::ApplicationTypes()
{
  auto result = this->_XCallEdit("availableApplicationTypes").AsArray();

  std::vector<std::string> retval;
  for (auto& entry : result)
    {
      retval.push_back(entry.AsString());
    }

  return retval;
}

int
ifm3d::LegacyDevice::Impl::CopyApplication(int idx)
{
  return this->_XCallEdit("copyApplication", idx).AsInt();
}

void
ifm3d::LegacyDevice::Impl::DeleteApplication(int idx)
{
  this->_XCallEdit("deleteApplication", idx);
}

int
ifm3d::LegacyDevice::Impl::CreateApplication(const std::string& type)
{
  return this->_XCallEdit("createApplication", type.c_str()).AsInt();
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
  return this->_XCallNet("getAllParameters").ToStringMap();
}

std::string
ifm3d::LegacyDevice::Impl::NetParameter(const std::string& param)
{
  return this->_XCallNet("getParameter", param.c_str()).AsString();
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
  return this->_XCallTime("getAllParameters").ToStringMap();
}

std::string
ifm3d::LegacyDevice::Impl::TimeParameter(const std::string& param)
{
  return this->_XCallTime("getParameter", param.c_str()).AsString();
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
  return this->_XCallApp("getAllParameters").ToStringMap();
}

std::string
ifm3d::LegacyDevice::Impl::AppParameter(const std::string& param)
{
  return this->_XCallApp("getParameter", param.c_str()).AsString();
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
  return this->_XCallImager("getAllParameters").ToStringMap();
}

std::string
ifm3d::LegacyDevice::Impl::ImagerParameter(const std::string& param)
{
  return this->_XCallImager("getParameter", param.c_str()).AsString();
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
  auto result = this->_XCallImager("availableTypes").AsArray();

  std::vector<std::string> retval;
  for (auto& entry : result)
    {
      retval.push_back(entry.AsString());
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
  return this->_XCallSpatialFilter("getAllParameters").ToStringMap();
}

std::string
ifm3d::LegacyDevice::Impl::SpatialFilterParameter(const std::string& param)
{
  return this->_XCallSpatialFilter("getParameter", param.c_str()).AsString();
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
  return this->_XCallTemporalFilter("getAllParameters").ToStringMap();
}

std::string
ifm3d::LegacyDevice::Impl::TemporalFilterParameter(const std::string& param)
{
  return this->_XCallTemporalFilter("getParameter", param.c_str()).AsString();
}

void
ifm3d::LegacyDevice::Impl::SetTemporalFilterParameter(const std::string& param,
                                                      const std::string& val)
{
  this->_XCallTemporalFilter("setParameter", param.c_str(), val.c_str());
}

#endif // IFM3D_LEGACY_DEVICE_IMPL_HPP