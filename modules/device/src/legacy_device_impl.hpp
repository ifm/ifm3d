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
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/err.h>
#include <ifm3d/device/legacy_device.h>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
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

  struct AppEntry
  {
    int index{};
    int id{};
    std::string name;
    std::string description;
  };

  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_NO_EXPORT LegacyDevice::Impl
  {
  public:
    Impl(std::shared_ptr<XMLRPC> xwrapper, std::string password);
    ~Impl();

    // Delete copy and move constructors and assignment operators
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

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
    std::vector<ifm3d::AppEntry> ApplicationList();
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
    WrapInEditSession(const std::function<void()>& f)
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
    std::shared_ptr<XMLRPC> _xwrapper;
    std::string _password;
    std::string _session;
    std::mutex _session_mutex;

    // ---------------------------------------------
    // _XCall wrappers
    // ---------------------------------------------

    std::string
    xmlrpc_session()
    {
      return std::regex_replace(ifm3d::XMLRPC_SESSION,
                                std::regex("\\$XXX"),
                                this->SessionID());
    }

    template <typename... ARGS>
    XMLRPCValue
    xmlrpc_call_main(const std::string& method, ARGS... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + xmlrpc_session();
      return this->_xwrapper->XCall(path, method, std::move(args)...);
    }

    template <typename... ARGS>
    XMLRPCValue
    xmlrpc_call_edit(const std::string& method, ARGS... args)
    {
      std::string path =
        ifm3d::XMLRPC_MAIN + xmlrpc_session() + ifm3d::XMLRPC_EDIT;
      return this->_xwrapper->XCall(path, method, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    xmlrpc_call_device(const std::string& method, ARGS... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + xmlrpc_session() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE;
      return this->_xwrapper->XCall(path, method, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    xmlrpc_call_net(const std::string& method, ARGS... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + xmlrpc_session() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE +
                         ifm3d::XMLRPC_NET;
      return this->_xwrapper->XCall(path, method, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    xmlrpc_call_time(const std::string& method, ARGS... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + xmlrpc_session() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_DEVICE +
                         ifm3d::XMLRPC_TIME;
      return this->_xwrapper->XCall(path, method, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    xmlrpc_call_app(const std::string& method, ARGS... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + xmlrpc_session() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP;
      return this->_xwrapper->XCall(path, method, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    xmlrpc_call_imager(const std::string& method, ARGS... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + xmlrpc_session() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP +
                         ifm3d::XMLRPC_IMAGER;
      return this->_xwrapper->XCall(path, method, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    xmlrpc_call_spatial_filter(const std::string& method, ARGS... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + xmlrpc_session() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP +
                         ifm3d::XMLRPC_IMAGER + ifm3d::XMLRPC_SPATIALFILTER;
      return this->_xwrapper->XCall(path, method, args...);
    }

    template <typename... ARGS>
    XMLRPCValue
    xmlrpc_call_temporal_filter(const std::string& method, ARGS... args)
    {
      std::string path = ifm3d::XMLRPC_MAIN + xmlrpc_session() +
                         ifm3d::XMLRPC_EDIT + ifm3d::XMLRPC_APP +
                         ifm3d::XMLRPC_IMAGER + ifm3d::XMLRPC_TEMPORALFILTER;
      return this->_xwrapper->XCall(path, method, args...);
    }

  }; // end: class Camera::Impl

} // end: namespace ifm3d

//============================================================
// Impl - Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------

inline ifm3d::LegacyDevice::Impl::Impl(std::shared_ptr<XMLRPC> xwrapper,
                                       std::string password)
  : _xwrapper(std::move(xwrapper)),
    _password(std::move(password))
{
  // Needed for fetching unit vectors over xmlrpc for O3X
  LOG_VERBOSE("Increasing XML-RPC response size limit...");
}

inline ifm3d::LegacyDevice::Impl::~Impl()
{
  LOG_VERBOSE("Dtor...");
  this->CancelSession();
}

//-------------------------------------
// Accessor/mutators
//-------------------------------------

inline std::string
ifm3d::LegacyDevice::Impl::IP()
{
  return this->_xwrapper->IP();
}

inline std::uint16_t
ifm3d::LegacyDevice::Impl::XMLRPCPort()
{
  return this->_xwrapper->XMLRPCPort();
}

inline std::string
ifm3d::LegacyDevice::Impl::Password()
{
  return this->_password;
}

inline std::string
ifm3d::LegacyDevice::Impl::SessionID()
{
  std::lock_guard<std::mutex> lock(this->_session_mutex);
  return this->_session;
}

inline void
ifm3d::LegacyDevice::Impl::SetSessionID(const std::string& id)
{
  std::lock_guard<std::mutex> lock(this->_session_mutex);
  this->_session = id;
}

// =============================================
// Public XMLRPC interface - worker methods
// =============================================

inline std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::HWInfo()
{
  return this->_xwrapper->XCallMain("getHWInfo").ToStringMap();
}

inline std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::SWVersion()
{
  return this->_xwrapper->XCallMain("getSWVersion").ToStringMap();
}

inline std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::DeviceInfo()
{
  return this->_xwrapper->XCallMain("getAllParameters").ToStringMap();
}

inline std::vector<ifm3d::AppEntry>
ifm3d::LegacyDevice::Impl::ApplicationList()
{
  auto result(this->_xwrapper->XCallMain("getApplicationList").AsArray());

  std::vector<ifm3d::AppEntry> retval;
  for (auto& entry : result)
    {
      auto entry_map = entry.AsMap();

      ifm3d::AppEntry app;
      app.index = entry_map["Index"].AsInt();
      app.id = entry_map["Id"].AsInt();
      app.name = entry_map["Name"].AsString();
      app.description = entry_map["Description"].AsString();

      retval.push_back(app);
    }
  return retval;
}

inline std::string
ifm3d::LegacyDevice::Impl::RequestSession(const std::string& sid)
{
  auto result =
    this->_xwrapper->XCallMain("requestSession", this->Password().c_str(), sid)
      .AsString();

  this->SetSessionID(static_cast<std::string>(result));
  this->Heartbeat(ifm3d::MAX_HEARTBEAT);
  return this->SessionID();
}

inline std::vector<std::uint8_t>
ifm3d::LegacyDevice::Impl::UnitVectors()
{
  return this->_xwrapper->XCallMain("getUnitVectors").AsByteArray();
}

inline void
ifm3d::LegacyDevice::Impl::ForceTrigger()
{
  this->_xwrapper->XCallMain("trigger");
}

// ---------------------------------------------
// Session
// ---------------------------------------------
inline bool
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

inline bool
ifm3d::LegacyDevice::Impl::CancelSession()
{
  if (this->SessionID() == "")
    {
      return true;
    }

  bool retval = true;

  try
    {
      this->xmlrpc_call_main("cancelSession");
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

inline int
ifm3d::LegacyDevice::Impl::Heartbeat(int hb)
{
  return this->xmlrpc_call_main("heartbeat", hb).AsInt();
}

inline void
ifm3d::LegacyDevice::Impl::SetOperatingMode(
  const ifm3d::LegacyDevice::operating_mode& mode)
{
  this->xmlrpc_call_main("setOperatingMode", static_cast<int>(mode));
}

inline void
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

  this->xmlrpc_call_main("setTemporaryApplicationParameters",
                         XMLRPCValue(param_map));
}

inline std::vector<std::uint8_t>
ifm3d::LegacyDevice::Impl::ExportIFMConfig()
{
  return this->xmlrpc_call_main("exportConfig").AsByteArray();
}

inline std::vector<std::uint8_t>
ifm3d::LegacyDevice::Impl::ExportIFMApp(int idx)
{
  return this->xmlrpc_call_main("exportApplication", idx).AsByteArray();
}

inline void
ifm3d::LegacyDevice::Impl::ImportIFMConfig(
  const std::vector<std::uint8_t>& bytes,
  std::uint16_t flags)
{
  this->xmlrpc_call_main("importConfig", bytes, flags);
}

inline int
ifm3d::LegacyDevice::Impl::ImportIFMApp(const std::vector<std::uint8_t>& bytes)
{
  return this->xmlrpc_call_main("importApplication", bytes).AsInt();
}

// ---------------------------------------------
// Edit Mode
// ---------------------------------------------

inline std::vector<std::string>
ifm3d::LegacyDevice::Impl::ApplicationTypes()
{
  auto result = this->xmlrpc_call_edit("availableApplicationTypes").AsArray();

  std::vector<std::string> retval;
  retval.reserve(result.size());
  for (auto& entry : result)
    {
      retval.push_back(entry.AsString());
    }

  return retval;
}

inline int
ifm3d::LegacyDevice::Impl::CopyApplication(int idx)
{
  return this->xmlrpc_call_edit("copyApplication", idx).AsInt();
}

inline void
ifm3d::LegacyDevice::Impl::DeleteApplication(int idx)
{
  this->xmlrpc_call_edit("deleteApplication", idx);
}

inline int
ifm3d::LegacyDevice::Impl::CreateApplication(const std::string& type)
{
  return this->xmlrpc_call_edit("createApplication", type.c_str()).AsInt();
}

inline void
ifm3d::LegacyDevice::Impl::FactoryReset()
{
  this->xmlrpc_call_edit("factoryReset");
}

inline void
ifm3d::LegacyDevice::Impl::EditApplication(int idx)
{
  this->xmlrpc_call_edit("editApplication", idx);
}

inline void
ifm3d::LegacyDevice::Impl::StopEditingApplication()
{
  this->xmlrpc_call_edit("stopEditingApplication");
}

// ---------------------------------------------
// Device
// ---------------------------------------------

inline void
ifm3d::LegacyDevice::Impl::SetDeviceParameter(const std::string& param,
                                              const std::string& val)
{
  this->xmlrpc_call_device("setParameter", param.c_str(), val.c_str());
}

inline void
ifm3d::LegacyDevice::Impl::SaveDevice()
{
  this->xmlrpc_call_device("save");
}

inline void
ifm3d::LegacyDevice::Impl::ActivatePassword(const std::string& password)
{
  this->xmlrpc_call_device("activatePassword", password.c_str());
}

inline void
ifm3d::LegacyDevice::Impl::DisablePassword()
{
  this->xmlrpc_call_device("disablePassword");
}

// ---------------------------------------------
// Network
// ---------------------------------------------

inline std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::NetInfo()
{
  return this->xmlrpc_call_net("getAllParameters").ToStringMap();
}

inline std::string
ifm3d::LegacyDevice::Impl::NetParameter(const std::string& param)
{
  return this->xmlrpc_call_net("getParameter", param.c_str()).AsString();
}

inline void
ifm3d::LegacyDevice::Impl::SetNetParameter(const std::string& param,
                                           const std::string& val)
{
  this->xmlrpc_call_net("setParameter", param.c_str(), val.c_str());
}

inline void
ifm3d::LegacyDevice::Impl::SaveNet()
{
  this->xmlrpc_call_net("saveAndActivateConfig");
}

// ---------------------------------------------
// Time
// ---------------------------------------------

inline std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::TimeInfo()
{
  return this->xmlrpc_call_time("getAllParameters").ToStringMap();
}

inline std::string
ifm3d::LegacyDevice::Impl::TimeParameter(const std::string& param)
{
  return this->xmlrpc_call_time("getParameter", param.c_str()).AsString();
}

inline void
ifm3d::LegacyDevice::Impl::SetTimeParameter(const std::string& param,
                                            const std::string& val)
{
  this->xmlrpc_call_time("setParameter", param.c_str(), val.c_str());
}

inline void
ifm3d::LegacyDevice::Impl::SaveTime()
{
  this->xmlrpc_call_time("saveAndActivateConfig");
}

inline void
ifm3d::LegacyDevice::Impl::SetCurrentTime(int epoch_seconds)
{
  if (epoch_seconds < 0)
    {
      epoch_seconds =
        static_cast<int>(std::chrono::seconds(std::time(nullptr)).count());
    }

  this->xmlrpc_call_time("setCurrentTime", epoch_seconds);
}

// ---------------------------------------------
// Application
// ---------------------------------------------

inline std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::AppInfo()
{
  return this->xmlrpc_call_app("getAllParameters").ToStringMap();
}

inline std::string
ifm3d::LegacyDevice::Impl::AppParameter(const std::string& param)
{
  return this->xmlrpc_call_app("getParameter", param.c_str()).AsString();
}

inline void
ifm3d::LegacyDevice::Impl::SetAppParameter(const std::string& param,
                                           const std::string& val)
{
  this->xmlrpc_call_app("setParameter", param.c_str(), val.c_str());
}

inline void
ifm3d::LegacyDevice::Impl::SaveApp()
{
  this->xmlrpc_call_app("save");
}

// ---------------------------------------------
// Imager
// ---------------------------------------------

inline std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::ImagerInfo()
{
  return this->xmlrpc_call_imager("getAllParameters").ToStringMap();
}

inline std::string
ifm3d::LegacyDevice::Impl::ImagerParameter(const std::string& param)
{
  return this->xmlrpc_call_imager("getParameter", param.c_str()).AsString();
}

inline void
ifm3d::LegacyDevice::Impl::SetImagerParameter(const std::string& param,
                                              const std::string& val)
{
  this->xmlrpc_call_imager("setParameter", param.c_str(), val.c_str());
}

inline std::vector<std::string>
ifm3d::LegacyDevice::Impl::ImagerTypes()
{
  auto result = this->xmlrpc_call_imager("availableTypes").AsArray();

  std::vector<std::string> retval;
  retval.reserve(result.size());
  for (auto& entry : result)
    {
      retval.push_back(entry.AsString());
    }

  return retval;
}

inline void
ifm3d::LegacyDevice::Impl::ChangeImagerType(const std::string& type)
{
  this->xmlrpc_call_imager("changeType", type.c_str());
}

// ---------------------------------------------
// Spatial Filter
// ---------------------------------------------

inline std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::SpatialFilterInfo()
{
  return this->xmlrpc_call_spatial_filter("getAllParameters").ToStringMap();
}

inline std::string
ifm3d::LegacyDevice::Impl::SpatialFilterParameter(const std::string& param)
{
  return this->xmlrpc_call_spatial_filter("getParameter", param.c_str())
    .AsString();
}

inline void
ifm3d::LegacyDevice::Impl::SetSpatialFilterParameter(const std::string& param,
                                                     const std::string& val)
{
  this->xmlrpc_call_spatial_filter("setParameter", param.c_str(), val.c_str());
}

// ---------------------------------------------
// Temporal Filter
// ---------------------------------------------

inline std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::Impl::TemporalFilterInfo()
{
  return this->xmlrpc_call_temporal_filter("getAllParameters").ToStringMap();
}

inline std::string
ifm3d::LegacyDevice::Impl::TemporalFilterParameter(const std::string& param)
{
  return this->xmlrpc_call_temporal_filter("getParameter", param.c_str())
    .AsString();
}

inline void
ifm3d::LegacyDevice::Impl::SetTemporalFilterParameter(const std::string& param,
                                                      const std::string& val)
{
  this->xmlrpc_call_temporal_filter("setParameter",
                                    param.c_str(),
                                    val.c_str());
}

#endif // IFM3D_LEGACY_DEVICE_IMPL_HPP