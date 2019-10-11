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
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <glog/logging.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/camera/version.h>
#include <ifm3d/camera/util.h>
#include <camera_impl.hpp>

//================================================
// Public constants
//================================================
const std::string ifm3d::DEFAULT_PASSWORD = "";
const std::uint16_t ifm3d::DEFAULT_XMLRPC_PORT = 80;
const int ifm3d::DEFAULT_PCIC_PORT = 50010;
const std::string ifm3d::DEFAULT_IP =
  std::getenv("IFM3D_IP") == nullptr ?
  "192.168.0.69" : std::string(std::getenv("IFM3D_IP"));
const int ifm3d::MAX_HEARTBEAT = 300; // secs
const std::size_t ifm3d::SESSION_ID_SZ = 32;
const std::string ifm3d::DEFAULT_APPLICATION_TYPE = "Camera";

auto __ifm3d_session_id__ = []() -> std::string
{
  std::string sid;

  try
    {
      if (std::getenv("IFM3D_SESSION_ID") == nullptr)
        {
          sid = "";
        }
      else
        {
          sid = std::string(std::getenv("IFM3D_SESSION_ID"));
          if (! ((sid.size() == ifm3d::SESSION_ID_SZ) &&
                 (sid.find_first_not_of("0123456789abcdefABCDEF") ==
                  std::string::npos)))
            {
              LOG(WARNING) << "Invalid session id: " << sid;
              sid = "";
            }
          else
            {
              LOG(INFO) << "Default session id: " << sid;
            }
        }
    }
  catch (const std::exception& ex)
    {
      LOG(WARNING) << "When trying to set default session id: "
                   << ex.what();

      sid = "";
    }

  return sid;
};

const std::string ifm3d::DEFAULT_SESSION_ID = __ifm3d_session_id__();

const int ifm3d::DEV_O3D_MIN = 1;
const int ifm3d::DEV_O3D_MAX = 255;
const int ifm3d::DEV_O3X_MIN = 512;
const int ifm3d::DEV_O3X_MAX = 767;

const std::string ifm3d::ASSUME_DEVICE =
  std::getenv("IFM3D_DEVICE") == nullptr ?
  "" : std::string(std::getenv("IFM3D_DEVICE"));

const unsigned int ifm3d::O3D_TIME_SUPPORT_MAJOR = 1;
const unsigned int ifm3d::O3D_TIME_SUPPORT_MINOR = 20;
const unsigned int ifm3d::O3D_TIME_SUPPORT_PATCH = 790;

const unsigned int ifm3d::O3D_TMP_PARAMS_SUPPORT_MAJOR = 1;
const unsigned int ifm3d::O3D_TMP_PARAMS_SUPPORT_MINOR = 20;
const unsigned int ifm3d::O3D_TMP_PARAMS_SUPPORT_PATCH = 0;

const unsigned int ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MAJOR = 1;
const unsigned int ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MINOR = 23;
const unsigned int ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_PATCH = 0;

const unsigned int ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR = 1;
const unsigned int ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR = 30;
const unsigned int ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH = 4123;

//================================================
// A lookup table listing the read-only camera
// parameters
//================================================
std::unordered_map<std::string,
                   std::unordered_map<std::string, bool> >
RO_LUT=
  {
    {"Device",
     {
       {"IPAddressConfig", true},
       {"PasswordActivated", true},
       {"OperatingMode", true},
       {"DeviceType", true},
       {"ArticleNumber", true},
       {"ArticleStatus", true},
       {"UpTime", true},
       {"ImageTimestampReference", true},
       {"TemperatureFront1", true},
       {"TemperatureFront2", true},
       {"TemperatureIMX6", true},
       {"TemperatureIllu", true},
     }
    },

    {"App",
     {
       {"Id", true},
     }
    },

    {"Net",
     {
       {"MACAddress", true},
     }
    },

    {"Time",
     {
       {"StartingSynchronization", true},
       {"Syncing", true},
       {"CurrentTime", true},
       {"Stats", true},
     }
    },

    {"Imager",
     {
       // Do not uncomment this! We treat the "Type"
       // key special to make it easy to change imager types --
       // i.e., while technically it is a RO param on the camera
       // we use it as a trigger to call ChangeType(...)
       // {"Type", true},
       {"ClippingLeft", true},
       {"ClippingTop", true},
       {"ClippingRight", true},
       {"ClippingBottom", true},
       {"ExposureTimeList", true},
       {"MaxAllowedLEDFrameRate", true},
     }
    },

    {"SpatialFilter",
     {
       {}
     }
    },

    {"TemporalFilter",
     {
      {}
     }
    }
  };

//================================================
// Factory function for making cameras
//================================================
ifm3d::Camera::Ptr
ifm3d::Camera::MakeShared(const std::string& ip,
                          const std::uint16_t xmlrpc_port,
                          const std::string& password)
{
  auto base = std::make_shared<ifm3d::Camera>(ip, xmlrpc_port, password);
  try
    {
      //
      // It is an optimization to return the specialized subclass
      //
      if (base->IsO3X())
        {
          VLOG(IFM3D_TRACE) << "Instantiating O3X...";
          return std::make_shared<ifm3d::O3XCamera>(ip, xmlrpc_port, password);
        }
      else if (base->IsO3D())
        {
          VLOG(IFM3D_TRACE) << "Instantiating O3D...";
          return std::make_shared<ifm3d::O3DCamera>(ip, xmlrpc_port, password);
        }
      else
        {
          LOG(WARNING) << "Unexpected camera device type: "
                       << base->DeviceType();
        }
    }
  catch (const ifm3d::error_t& ex)
    {
      if (ex.code() == IFM3D_XMLRPC_TIMEOUT)
        {
          LOG(WARNING) << "Could not probe device type: "
                       << ex.what();
        }
      else
        {
          LOG(ERROR) << "While trying to instantiate camera: "
                     << ex.what();
          //
          // XXX: For now, we re-throw. I am not sure what else would
          // go wrong here except for a network time out. To that end,
          // I'd like this to "fail loudly" so I can get to the root
          // of the issue
          //
          throw;
        }
    }

  //
  // worst case: we return the "non-optimized" base class
  //
  LOG(WARNING) << "Returning instance of base camera class!";
  return base;
}

//================================================
// Camera class - the public interface
//================================================

ifm3d::Camera::Camera(const std::string& ip,
                      const std::uint16_t xmlrpc_port,
                      const std::string& password)
  : pImpl(new ifm3d::Camera::Impl(ip, xmlrpc_port, password)),
    device_type_("")
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

bool
ifm3d::Camera::CancelSession(const std::string& sid)
{
  return this->pImpl->CancelSession(sid);
}

int
ifm3d::Camera::Heartbeat(int hb)
{
  return this->pImpl->Heartbeat(hb);
}

void
ifm3d::Camera::SetTemporaryApplicationParameters(
  const std::unordered_map<std::string, std::string>& params)
{
  //
  // NOTE: we "fail" silently here. Assumption is a closed loop check by the
  // user to see if/when the temp params take effect.
  //
  if (this->IsO3D() &&
      (! this->CheckMinimumFirmwareVersion(ifm3d::O3D_TMP_PARAMS_SUPPORT_MAJOR,
                                     ifm3d::O3D_TMP_PARAMS_SUPPORT_MINOR,
                                     ifm3d::O3D_TMP_PARAMS_SUPPORT_PATCH)))
    {
      LOG(WARNING) << "Setting temp params not supported by this device!";
      return;
    }

  this->pImpl->SetTemporaryApplicationParameters(params);
}

void
ifm3d::Camera::ForceTrigger()
{
  if (! this->IsO3X())
    {
      return;
    }

  return this->pImpl->ForceTrigger();
}

std::vector<std::uint8_t>
ifm3d::Camera::UnitVectors()
{
  if (this->IsO3X())
    {
      return this->pImpl->UnitVectors();
    }

  LOG(ERROR) << "The device does not support the XMLRPC UnitVectors accessor";
  throw ifm3d::error_t(IFM3D_UNSUPPORTED_OP);
}

void
ifm3d::Camera::Reboot(const ifm3d::Camera::boot_mode& mode)
{
  this->pImpl->Reboot(static_cast<int>(mode));
}

std::string
ifm3d::Camera::DeviceParameter(const std::string& key)
{
  return this->pImpl->DeviceParameter(key);
}

std::vector<std::string>
ifm3d::Camera::TraceLogs(int count)
{
  return this->pImpl->TraceLogs(count);
}

std::string
ifm3d::Camera::DeviceType(bool use_cached)
{
  if (ifm3d::ASSUME_DEVICE != "")
    {
      LOG(WARNING) << "Returning assumed device type: "
                   << ifm3d::ASSUME_DEVICE;
      return ifm3d::ASSUME_DEVICE;
    }

  if (this->device_type_ != "")
    {
      if (use_cached)
        {
          return this->device_type_;
        }
    }

  this->device_type_ = this->pImpl->DeviceParameter("DeviceType");
  return this->device_type_;
}

bool
ifm3d::Camera::IsO3X()
{
  int devid = 0;
  std::string dt = this->DeviceType();
  std::string::size_type n = dt.find(":");
  if (n != std::string::npos)
    {
      try
        {
          devid = std::atoi(dt.substr(n+1).c_str());
        }
      catch (std::out_of_range& ex)
        {
          LOG(WARNING) << ex.what();
        }
    }

  if ((devid >= ifm3d::DEV_O3X_MIN) &&
      (devid <= ifm3d::DEV_O3X_MAX))
    {
      return true;
    }

  return false;
}

bool
ifm3d::Camera::IsO3D()
{
  int devid = 0;
  std::string dt = this->DeviceType();
  std::string::size_type n = dt.find(":");
  if (n != std::string::npos)
    {
      try
        {
          devid = std::atoi(dt.substr(n+1).c_str());
        }
      catch (std::out_of_range& ex)
        {
          LOG(WARNING) << ex.what();
        }
    }

  if ((devid >= ifm3d::DEV_O3D_MIN) &&
      (devid <= ifm3d::DEV_O3D_MAX))
    {
      return true;
    }

  return false;
}

int
ifm3d::Camera::ActiveApplication()
{
  if (this->IsO3X())
    {
      return 1;
    }

  return std::stoi(this->pImpl->DeviceParameter("ActiveApplication"));
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

std::vector<std::string>
ifm3d::Camera::ImagerTypes()
{
  return this->pImpl->WrapInEditSession<std::vector<std::string> >(
    [this]()->std::vector<std::string>
    {
      if (! this->IsO3X())
        {
          this->pImpl->EditApplication(this->ActiveApplication());
        }
      return this->pImpl->ImagerTypes();
    });
}

int
ifm3d::Camera::CreateApplication(const std::string& type)
{
  if (this->IsO3X())
    {
      LOG(ERROR) << "O3X only supports a single app, create not supported";
      throw ifm3d::error_t(IFM3D_UNSUPPORTED_OP);
    }

  return this->pImpl->WrapInEditSession<int>(
    [this,&type]()->int { return this->pImpl->CreateApplication(type); });
}

int
ifm3d::Camera::CopyApplication(int idx)
{
  if (this->IsO3X())
    {
      LOG(ERROR) << "O3X only supports a single app, copy not supported";
      throw ifm3d::error_t(IFM3D_UNSUPPORTED_OP);
    }

  return this->pImpl->WrapInEditSession<int>(
    [this,idx]()->int { return this->pImpl->CopyApplication(idx); });
}

void
ifm3d::Camera::DeleteApplication(int idx)
{
  if (this->IsO3X())
    {
      LOG(ERROR) << "O3X only supports a single app, delete not supported";
      throw ifm3d::error_t(IFM3D_UNSUPPORTED_OP);
    }

  this->pImpl->WrapInEditSession(
     [this,idx]() { this->pImpl->DeleteApplication(idx); });
}

void
ifm3d::Camera::FactoryReset()
{
  this->pImpl->WrapInEditSession([this]() { this->pImpl->FactoryReset(); });
}

void
ifm3d::Camera::SetCurrentTime(int epoch_secs)
{
  this->pImpl->WrapInEditSession(
    [this,epoch_secs]() { this->pImpl->SetCurrentTime(epoch_secs); });
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

bool
ifm3d::Camera::CheckMinimumFirmwareVersion(unsigned int major,
                                     unsigned int minor, unsigned int patch)
{

  auto data = this->pImpl->SWVersion();
  const std::string swversion = data["IFM_Software"];
  std::istringstream str(swversion);
  std::vector<std::string> strings;
  std::string token;
  while (getline(str, token, '.'))
    {
      strings.push_back(token);
    }
  const auto cmajor = std::stoi(strings[0],nullptr);
  const auto cminor = std::stoi(strings[1],nullptr);
  const auto cpatch = std::stoi(strings[2],nullptr);
  auto res = false;
  if(cmajor > major)
    {
      res = true;
    }
  else if (cmajor == major)
    {
      if(cminor > minor)
        {
          res = true;
        }
      else if (cminor == minor)
        {
          if(cpatch > patch)
            {
              res = true;
            }
          else if(cpatch == patch)
            {
              res = true;
            }
        }
    }

  return res;
}

json
ifm3d::Camera::ToJSON_(const bool open_session)
{
  auto t =
    std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::ostringstream time_buf;
  time_buf << std::ctime(&t);
  std::string time_s = time_buf.str();
  ifm3d::trim(time_s);

  json app_list = this->ApplicationList();
  json net_info, app_info;
  json time_info = json::parse("{}");

  auto exec_toJSON = [this,&net_info,&time_info,&app_info,&app_list]()
    {
      net_info = json(this->pImpl->NetInfo());
      if (this->IsO3X() ||
          (this->IsO3D() &&
           this->CheckMinimumFirmwareVersion(ifm3d::O3D_TIME_SUPPORT_MAJOR,
                                       ifm3d::O3D_TIME_SUPPORT_MINOR,
                                       ifm3d::O3D_TIME_SUPPORT_PATCH)))
        {
          time_info = json(this->pImpl->TimeInfo());
        }
      app_info = json::parse("[]");

      for (auto& app : app_list)
        {
          int idx = app["Index"].get<int>();
          if (! this->IsO3X())
            {
              this->pImpl->EditApplication(idx);
            }

          json app_json = json(this->pImpl->AppInfo());
          app_json["Index"] = std::to_string(idx);
          app_json["Id"] = std::to_string(app["Id"].get<int>());

          json imager_json = json(this->pImpl->ImagerInfo());

	  /* Initialize the imager_json filters with defult values for
	     compatibility with o3x which does not support dedicated xmlrpc
             filter objects since there are no config options for the o3x filter */
	  std::unordered_map<std::string, std::string> spatialFil = {{"MaskSize", "0"}};
          std::unordered_map<std::string, std::string> tmpFil = {{}}; // empty map
          imager_json["SpatialFilter"] = json(spatialFil);
          imager_json["TemporalFilter"] = json(tmpFil);

          if (! this->IsO3X())
            {
		json sfilt_json = json(this->pImpl->SpatialFilterInfo());
                imager_json["SpatialFilter"] = sfilt_json;

                json tfilt_json = json(this->pImpl->TemporalFilterInfo());
                imager_json["TemporalFilter"] = tfilt_json;
            }

          app_json["Imager"] = imager_json;

          app_info.push_back(app_json);

          if (! this->IsO3X())
            {
              this->pImpl->StopEditingApplication();
            }
        }
    };
  if(open_session)
    {
        this->pImpl->WrapInEditSession(exec_toJSON);
    }
  else
    {
        exec_toJSON();
    }

  json j =
    {
      {
       "ifm3d",
       {
         {"_",
          {
            {std::string(IFM3D_LIBRARY_NAME) + "_version", IFM3D_VERSION},
            {"Date", time_s},
            {"HWInfo", json(this->pImpl->HWInfo())},
            {"SWVersion", json(this->pImpl->SWVersion())}
          }
         },
         {"Device", json(this->pImpl->DeviceInfo())},
         {"Net", net_info},
         {"Time", time_info},
         {"Apps", app_info}
       }
      }
    };

  return j;
}

json
ifm3d::Camera::ToJSON()
{
      return ToJSON_();
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
                         const std::string& name,
                         int idx)
{
  VLOG(IFM3D_TRACE) << "Setting " << name << " parameters";
  if (! j_new.is_object())
    {
      LOG(ERROR) << "The passed in " << name << " json should be an object!";
      VLOG(IFM3D_TRACE) << "Invalid JSON was: " << j_new.dump();

      throw ifm3d::error_t(IFM3D_JSON_ERROR);
    }

  if (idx > 0)
    {
      if (! this->IsO3X())
        {
          VLOG(IFM3D_TRACE) << "Editing app at idx=" << idx;
          this->pImpl->EditApplication(idx);
        }
    }

  bool do_save = false;
  for (auto it = j_new.begin(); it != j_new.end(); ++it)
    {
      std::string key = it.key();
      VLOG(IFM3D_TRACE) << "Processing key="
                        << key << ", with val="
                        << j_new[key].dump(2);
      if (it.value().is_null())
        {
          LOG(WARNING)
            << "Skipping " <<  key << ", null value -- should be string!";
          continue;
        }
      std::string val = j_new[key].get<std::string>();
      if (j_curr[key].get<std::string>() != val)
        {
          try
            {
              try
                {
                  if (RO_LUT.at(name).at(key))
                    {
                      VLOG(IFM3D_TRACE)
                        << "Skipping read-only " << name
                        << " param: " << key;
                      continue;
                    }
                }
              catch (const std::out_of_range& /*ex*/)
                {
                  // just swallow the error -- we are setting a
                  // r/w parameter
                }

              VLOG(IFM3D_TRACE) << "Setting " << name << " parameter: "
                                << key << "=" << val;
              SetFunc(key, val);
              do_save = true;
            }
          catch (const ifm3d::error_t& ex)
            {
              if (ex.code() == IFM3D_READONLY_PARAM)
                {
                  LOG(WARNING)
                    << "Tried to set read-only " << name << " param: "
                    << key;
                }
              else
                {
                  throw;
                }
            }
        }
      else
        {
          VLOG(IFM3D_TRACE)
            << "Skipping " << key << ", no change in value";
        }
    }

  if (do_save)
    {
      SaveFunc();
    }
  if(idx > 0)
    {
    if(!this->IsO3X())
      {
        VLOG(IFM3D_TRACE) << "Stop editing app at idx=" << idx;
        this->pImpl->StopEditingApplication();
      }
    }
}

void
ifm3d::Camera::FromJSON(const json& j)
{
  VLOG(IFM3D_TRACE) << "Checking if passed in JSON is an object";
  if (! j.is_object())
    {
      LOG(ERROR) << "The passed in json should be an object!";
      VLOG(IFM3D_TRACE) << "Invalid JSON was: " << j.dump();

      throw ifm3d::error_t(IFM3D_JSON_ERROR);
    }

  // we use this to lessen the number of overall network calls
  VLOG(IFM3D_TRACE) << "Caching current camera dump";
  json current = this->ToJSON();

  // make the `ifm3d` root element optional
  VLOG(IFM3D_TRACE) << "Extracing root element";
  json root = j.count("ifm3d") ? j["ifm3d"] : j;

  // Ensure we cancel the session when leaving this method
  this->pImpl->WrapInEditSession([this,&root,&j,&current]()
    {
      // Device
      json j_dev = root["Device"];
      if (! j_dev.is_null())
      {
        this->FromJSON_(current["ifm3d"]["Device"], j_dev,
            [this](const std::string& k, const std::string& v)
            {
              this->pImpl->SetDeviceParameter(k,v);
            },
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

          VLOG(IFM3D_TRACE) << "Looping over applications";
          for (auto& j_app : j_apps)
            {
              if (! j_app.is_object())
              {
                LOG(ERROR)
                  << "All 'Apps' must be a JSON object!";
                VLOG(IFM3D_TRACE)
                  << "Invalid JSON was: " << j_app.dump();
                throw ifm3d::error_t(IFM3D_JSON_ERROR);
              }

              // First we determine if we are editing an existing application or if
              // we are creating a new one. If no index is specified, we create a
              // new application.
              int idx = -1;
              if (j_app["Index"].is_null())
                {
                  if (! this->IsO3X())
                    {
                      VLOG(IFM3D_TRACE) << "Creating new application";
                      idx = j_app["Type"].is_null() ?
                      this->pImpl->CreateApplication(
                          DEFAULT_APPLICATION_TYPE) :
                          this->pImpl->CreateApplication(
                            j_app["Type"].get<std::string>());

                      VLOG(IFM3D_TRACE)
                        << "Created new app, updating our dump";
                      current = this->ToJSON_(false);
                    }
                    else
                    {
                      VLOG(IFM3D_TRACE)
                        << "O3X only has a single app, assuming idx=1";
                      idx = 1;
                    }
              }
              else
              {
                VLOG(IFM3D_TRACE)
                  << "Getting index of existing application";
                idx = std::stoi(j_app["Index"].get<std::string>());
              }

              VLOG(IFM3D_TRACE) << "Application of interest is at index="
                << idx;

              // now in `current` (which is a whole camera dump)
              // we need to find the application at index `idx`.
              json curr_app = json({});
              json curr_apps = current["ifm3d"]["Apps"];
              bool app_found = false;
              for (auto& a : curr_apps)
                {
                  if (std::stoi(a["Index"].get<std::string>()) == idx)
                    {
                      curr_app = a;
                      app_found = true;
                      break;
                    }
                }

              if (! app_found)
                {
                  LOG(ERROR) << "Could not find an application at index="
                  << idx;
                  throw ifm3d::error_t(IFM3D_JSON_ERROR);
                }

              // at this point both the new and current
              // should have the same index and type and
              // we want to make sure of that.
              j_app["Index"] = curr_app["Index"];
              j_app["Type"] = curr_app["Type"];

              // pull out the imager sub-tree (we treat that separately)
              json j_im = j_app["Imager"];
              if (! j_im.is_null())
              {
                j_app.erase("Imager");
              }

              this->FromJSON_(curr_app, j_app,
                [this](const std::string& k, const std::string& v)
                  {
                    this->pImpl->SetAppParameter(k,v);
                  },
                [this](){ this->pImpl->SaveApp(); },
                "App", idx);

              json s_filt = j_im["SpatialFilter"];
              if (! s_filt.is_null())
                {
                  j_im.erase("SpatialFilter");
                }

              json t_filt = j_im["TemporalFilter"];
              if (! t_filt.is_null())
                {
                  j_im.erase("TemporalFilter");
                }

              this->FromJSON_(curr_app["Imager"], j_im,
                [this](const std::string& k, const std::string& v)
                  {
                    if (k == "Type")
                      {
                        this->pImpl->ChangeImagerType(v);
                      }
                    else
                      {
                        this->pImpl->SetImagerParameter(k,v);
                      }
                  },
                [this](){ this->pImpl->SaveApp(); },
                "Imager", idx);

              if (! this->IsO3X())
                {

                  if (! s_filt.is_null())
                    {
                      this->FromJSON_(curr_app["Imager"]["SpatialFilter"],
                          s_filt,
                          [this](const std::string& k, const std::string& v)
                            { this->pImpl->SetSpatialFilterParameter(k,v); },
                          [this](){ this->pImpl->SaveApp(); },
                          "SpatialFilter", idx);
                    }

                  if (! t_filt.is_null())
                    {
                      this->FromJSON_(curr_app["Imager"]["TemporalFilter"],
                          t_filt,
                          [this](const std::string& k, const std::string& v)
                            {
                              this->pImpl->SetTemporalFilterParameter(k,v);
                            },
                          [this](){ this->pImpl->SaveApp(); },
                          "TemporalFilter", idx);
                    }
                }
            }
        }

        // Time
        if (this->IsO3X() ||
            (this->IsO3D() &&
             this->CheckMinimumFirmwareVersion(ifm3d::O3D_TIME_SUPPORT_MAJOR,
               ifm3d::O3D_TIME_SUPPORT_MINOR,
              ifm3d::O3D_TIME_SUPPORT_PATCH)))
          {
            json j_time = root["Time"];
            if (! j_time.is_null())
              {
                this->FromJSON_(current["ifm3d"]["Time"], j_time,
                  [this](const std::string& k, const std::string& v)
                    { this->pImpl->SetTimeParameter(k,v); },
                  [this](){ this->pImpl->SaveTime(); },
                  "Time");
              }
          }

        // Network - we do this last intentionally!
        json j_net = root["Net"];
        if (! j_net.is_null())
          {
            this->FromJSON_(current["ifm3d"]["Net"], j_net,
              [this](const std::string& k, const std::string& v)
                { this->pImpl->SetNetParameter(k,v); },
              [this]()
                { // we are changing network parameters,
                  // we expect a timeout!
                try
                  {
                    this->pImpl->SaveNet();
                   }
                catch (const ifm3d::error_t& ex)
                  {
                    if (ex.code() == IFM3D_XMLRPC_TIMEOUT)
                    {
                      LOG(WARNING)
                        << "XML-RPC timeout saving net params, "
                        << "this is expected";
                    }
                    else
                    {
                      throw;
                    }
                  }
                },
              "Net");
          }
      });
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

void
ifm3d::Camera::SetPassword(std::string password)
{
  this->pImpl->WrapInEditSession(
	[this, password]() { password == "" ?
	this->pImpl->DisablePassword() : this->pImpl->ActivatePassword(password);
	this->pImpl->SaveDevice(); });

}

//================================================
// O3DCamera class - the public interface
//================================================

ifm3d::O3DCamera::O3DCamera(const std::string& ip,
                            const std::uint16_t xmlrpc_port,
                            const std::string& password)
  : ifm3d::Camera::Camera(ip, xmlrpc_port, password)
{ }

ifm3d::O3DCamera::~O3DCamera() = default;

bool
ifm3d::O3DCamera::IsO3X()
{
  return false;
}

bool
ifm3d::O3DCamera::IsO3D()
{
  return true;
}

//================================================
// O3XCamera class - the public interface
//================================================

ifm3d::O3XCamera::O3XCamera(const std::string& ip,
                            const std::uint16_t xmlrpc_port,
                            const std::string& password)
  : ifm3d::Camera::Camera(ip, xmlrpc_port, password)
{ }

ifm3d::O3XCamera::~O3XCamera() = default;

bool
ifm3d::O3XCamera::IsO3X()
{
  return true;
}

bool
ifm3d::O3XCamera::IsO3D()
{
  return false;
}
