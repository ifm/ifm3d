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
#include <functional>
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

  void SetOperatingMode(const ifm3d::Camera::operating_mode& mode)
  {
    this->_XCallSession("setOperatingMode", static_cast<int>(mode));
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

  std::string DeviceParameter(const std::string& param)
  {
    return xmlrpc_c::value_string(
             this->_XCallMain("getParameter", param.c_str())).cvalue();
  }

  void SetDeviceParameter(const std::string& param, const std::string& val)
  {
    this->_XCallDevice("setParameter", param.c_str(), val.c_str());
  }

  void SaveDevice()
  {
    this->_XCallDevice("save");
  }

  std::unordered_map<std::string, std::string> NetInfo()
  {
    return this->value_struct_to_map(this->_XCallNet("getAllParameters"));
  }

  std::string NetParameter(const std::string& param)
  {
    return xmlrpc_c::value_string(
             this->_XCallNet("getParameter", param.c_str())).cvalue();
  }

  void SetNetParameter(const std::string& param, const std::string& val)
  {
    this->_XCallNet("setParameter", param.c_str(), val.c_str());
  }

  void SaveNet()
  {
    this->_XCallNet("saveAndActivateConfig");
  }

  std::unordered_map<std::string, std::string> AppInfo()
  {
    return this->value_struct_to_map(this->_XCallApp("getAllParameters"));
  }

  std::string AppParameter(const std::string& param)
  {
    return xmlrpc_c::value_string(
             this->_XCallApp("getParameter", param.c_str())).cvalue();
  }

  void SetAppParameter(const std::string& param, const std::string& val)
  {
    this->_XCallApp("setParameter", param.c_str(), val.c_str());
  }

  void SaveApp()
  {
    this->_XCallApp("save");
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

  std::vector<std::string> ApplicationTypes()
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

  int CopyApplication(int idx)
  {
    xmlrpc_c::value_int v_int(this->_XCallEdit("copyApplication", idx));
    return v_int.cvalue();
  }

  void DeleteApplication(int idx)
  {
    this->_XCallEdit("deleteApplication", idx);
  }

  int CreateApplication(const std::string& type)
  {
    xmlrpc_c::value_int
      v_int(this->_XCallEdit("createApplication", type.c_str()));
    return v_int.cvalue();
  }

  void FactoryReset()
  {
    this->_XCallEdit("factoryReset");
  }

  std::vector<std::uint8_t> ExportIFMConfig()
  {
    const xmlrpc_c::value_bytestring v_bytes =
      this->_XCallSession("exportConfig");

    return v_bytes.vectorUcharValue();
  }

  std::vector<std::uint8_t> ExportIFMApp(int idx)
  {
    const xmlrpc_c::value_bytestring v_bytes =
      this->_XCallSession("exportApplication", idx);

    return v_bytes.vectorUcharValue();
  }

  void ImportIFMConfig(const std::vector<std::uint8_t>& bytes,
                       std::uint16_t flags)
  {
    this->_XCallSession("importConfig", bytes, flags);
  }

  int ImportIFMApp(const std::vector<std::uint8_t>& bytes)
  {
    xmlrpc_c::value_int v_int(this->_XCallSession("importApplication", bytes));
    return v_int.cvalue();
  }

  void EditApplication(int idx)
  {
    this->_XCallEdit("editApplication", idx);
  }

  void StopEditingApplication()
  {
    this->_XCallEdit("stopEditingApplication");
  }

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

std::string
ifm3d::Camera::ArticleNumber()
{
  return this->pImpl->DeviceParameter("ArticleNumber");
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

std::vector<std::string>
ifm3d::Camera::ApplicationTypes()
{
  return this->pImpl->WrapInEditSession<std::vector<std::string> >(
    [this]()->std::vector<std::string>
    { return this->pImpl->ApplicationTypes(); });
}

int
ifm3d::Camera::CreateApplication(const std::string& type)
{
  return this->pImpl->WrapInEditSession<int>(
    [this,&type]()->int { return this->pImpl->CreateApplication(type); });
}

int
ifm3d::Camera::CopyApplication(int idx)
{
  return this->pImpl->WrapInEditSession<int>(
    [this,idx]()->int { return this->pImpl->CopyApplication(idx); });
}

void
ifm3d::Camera::DeleteApplication(int idx)
{
  this->pImpl->WrapInEditSession(
     [this,idx]() { this->pImpl->DeleteApplication(idx); });
}

void
ifm3d::Camera::FactoryReset()
{
  this->pImpl->WrapInEditSession([this]() { this->pImpl->FactoryReset(); });
}

std::vector<std::uint8_t>
ifm3d::Camera::ExportIFMConfig()
{
  return this->pImpl->WrapInEditSession<std::vector<std::uint8_t> >(
    [this]()->std::vector<std::uint8_t>
    { return this->pImpl->ExportIFMConfig(); });
}

std::vector<std::uint8_t>
ifm3d::Camera::ExportIFMApp(int idx)
{
  return this->pImpl->WrapInEditSession<std::vector<std::uint8_t> >(
    [this,idx]()->std::vector<std::uint8_t>
    { return this->pImpl->ExportIFMApp(idx); });
}

void
ifm3d::Camera::ImportIFMConfig(const std::vector<std::uint8_t>& bytes,
                               std::uint16_t flags)
{
  return this->pImpl->WrapInEditSession(
    [this,&bytes,flags]() { this->pImpl->ImportIFMConfig(bytes, flags); });
}

int
ifm3d::Camera::ImportIFMApp(const std::vector<std::uint8_t>& bytes)
{
  return this->pImpl->WrapInEditSession<int>(
    [this,&bytes]()->int { return this->pImpl->ImportIFMApp(bytes); });
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

  json app_list = this->ApplicationList();
  json net_info, app_info;

  this->pImpl->WrapInEditSession(
    [this,&net_info,&app_info,&app_list]()
    {
      net_info = json(this->pImpl->NetInfo());
      app_info = json::parse("[]");

      for (auto& app : app_list)
        {
          int idx = app["Index"].get<int>();
          this->pImpl->EditApplication(idx);

          json app_json = json(this->pImpl->AppInfo());
          app_json["LogicGraph"] =
            app_json["LogicGraph"].is_null() ?
            json::parse("{}") :
            json::parse(app_json["LogicGraph"].get<std::string>());
          app_json["PcicEipResultSchema"] =
            app_json["PcicEipResultSchema"].is_null() ?
            json::parse("{}") :
            json::parse(app_json["PcicEipResultSchema"].get<std::string>());
          app_json["PcicPnioResultSchema"] =
            app_json["PcicPnioResultSchema"].is_null() ?
            json::parse("{}") :
            json::parse(app_json["PcicPnioResultSchema"].get<std::string>());
          app_json["PcicTcpResultSchema"] =
            app_json["PcicTcpResultSchema"].is_null() ?
            json::parse("{}") :
            json::parse(app_json["PcicTcpResultSchema"].get<std::string>());
          app_json["Index"] = std::to_string(idx);
          app_json["Id"] = std::to_string(app["Id"].get<int>());
          app_info.push_back(app_json);

          this->pImpl->StopEditingApplication();
        }
    });

  json j =
    {
      {
       "ifm3d",
       {
         {std::string(IFM3D_LIBRARY_NAME) + "_version", IFM3D_VERSION},
         {"Date", time_s},
         {"HWInfo", json(this->pImpl->HWInfo())},
         {"SWVersion", json(this->pImpl->SWVersion())},
         {"Device", json(this->pImpl->DeviceInfo())},
         {"Net", net_info},
         {"Apps", app_info}
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

void
ifm3d::Camera::FromJSON_(const json& j_curr,
                         const json& j_new,
                         std::function<void(const std::string&,
                                            const std::string&)> SetFunc,
                         std::function<void()> SaveFunc,
                         const std::string& name)
{
  VLOG(IFM3D_TRACE) << "Setting " << name << " parameters...";
  if (! j_new.is_object())
    {
      LOG(ERROR) << "The passed in " << name << " json should be an object!";
      VLOG(IFM3D_TRACE) << "Invalid JSON was: " << j_new.dump();

      throw ifm3d::error_t(IFM3D_JSON_ERROR);
    }

  this->pImpl->WrapInEditSession (
    [&name,&j_curr,&j_new,&SetFunc,&SaveFunc]()
    {
      bool do_save = false;
      for (auto it = j_new.begin(); it != j_new.end(); ++it)
        {
          std::string key = it.key();
          std::string val = j_new[key].get<std::string>();
          if (j_curr[key].get<std::string>() != val)
            {
              try
                {
                  VLOG(IFM3D_TRACE) << "Setting " << name << " parameter: "
                                    << key << "=" << val;
                  SetFunc(key, val);
                  do_save = true;
                }
              catch (const ifm3d::error_t& ex)
                {
                  if (ex.code() != IFM3D_READONLY_PARAM)
                    {
                      throw;
                    }
                }
            }
        }

      if (do_save)
        {
          SaveFunc();
        }
    });
}

void
ifm3d::Camera::FromJSON(const json& j)
{
  if (! j.is_object())
    {
      LOG(ERROR) << "The passed in json should be an object!";
      VLOG(IFM3D_TRACE) << "Invalid JSON was: " << j.dump();

      throw ifm3d::error_t(IFM3D_JSON_ERROR);
    }

  // we use this to lessen the number of overall network calls
  json current = this->ToJSON();

  // make the `ifm3d` root element optional
  json root = j.count("ifm3d") ? j["ifm3d"] : j;

  // Device
  json j_dev = root["Device"];
  if (! j_dev.is_null())
    {
      this->FromJSON_(current["ifm3d"]["Device"], j_dev,
                      [this](const std::string& k,
                             const std::string& v)
                      { this->pImpl->SetDeviceParameter(k,v); },
                      [this](){ this->pImpl->SaveDevice(); },
                      "Device");
    }

  // Apps - requires careful/special treatment
  json j_apps = root["Apps"];
  if (! j_apps.is_null())
    {
      if (! j_apps.is_array())
        {
          LOG(ERROR) << "The `Apps` element should be an array!";
          VLOG(IFM3D_TRACE) << "Invalid JSON was: " << j_apps.dump();

          throw ifm3d::error_t(IFM3D_JSON_ERROR);
        }

      for (auto& j_app : j_apps)
        {
          //
          // XXX: Need to do application processing here
          //
        }
    }

  // Network - we do this last intentionally!
  json j_net = root["Net"];
  if (! j_net.is_null())
    {
      this->FromJSON_(current["ifm3d"]["Net"], j_net,
                      [this](const std::string& k,
                             const std::string& v)
                      { this->pImpl->SetNetParameter(k,v); },
                      [this](){ this->pImpl->SaveNet(); },
                      "Net");
    }
}

void
ifm3d::Camera::FromJSONStr(const std::string& jstr)
{
  // We make this a bit verbose for better error reporting
  json j;

  try
    {
      j = json::parse(jstr);
    }
  catch (const std::exception& ex)
    {
      LOG(ERROR) << "JSON: " << ex.what();
      throw ifm3d::error_t(IFM3D_JSON_ERROR);
    }

  this->FromJSON(j);
}
