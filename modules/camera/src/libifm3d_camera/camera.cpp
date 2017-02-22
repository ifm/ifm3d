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
#include <vector>
#include <boost/algorithm/string.hpp>
#include <glog/logging.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/camera/version.h>
#include <camera_impl.hpp>

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

std::vector<std::string>
ifm3d::Camera::ImagerTypes()
{
  return this->pImpl->WrapInEditSession<std::vector<std::string> >(
    [this]()->std::vector<std::string>
    {
      int idx = std::stoi(this->pImpl->DeviceParameter("ActiveApplication"));
      this->pImpl->EditApplication(idx);
      return this->pImpl->ImagerTypes();
    });
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
          app_json["Index"] = std::to_string(idx);
          app_json["Id"] = std::to_string(app["Id"].get<int>());

          json imager_json = json(this->pImpl->ImagerInfo());

          json sfilt_json = json(this->pImpl->SpatialFilterInfo());
          imager_json["SpatialFilter"] = sfilt_json;

          json tfilt_json = json(this->pImpl->TemporalFilterInfo());
          imager_json["TemporalFilter"] = tfilt_json;

          app_json["Imager"] = imager_json;

          app_info.push_back(app_json);

          this->pImpl->StopEditingApplication();
        }
    });

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

  this->pImpl->WrapInEditSession (
    [this,&name,idx,&j_curr,&j_new,&SetFunc,&SaveFunc]()
    {
      if (idx > 0)
        {
          VLOG(IFM3D_TRACE) << "Editing app at idx=" << idx;
          this->pImpl->EditApplication(idx);
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
    });
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

      VLOG(IFM3D_TRACE) << "Looping over applications";
      for (auto& j_app : j_apps)
        {
          if (! j_app.is_object())
            {
              LOG(ERROR) << "All 'Apps' must be a JSON object!";
              VLOG(IFM3D_TRACE) << "Invalid JSON was: " << j_app.dump();
              throw ifm3d::error_t(IFM3D_JSON_ERROR);
            }

          // First we determine if we are editing an existing application or if
          // we are creating a new one. If no index is specified, we create a
          // new application.
          int idx = -1;
          if (j_app["Index"].is_null())
            {
              VLOG(IFM3D_TRACE) << "Creating new application";
              idx = j_app["Type"].is_null() ?
                this->CreateApplication() :
                this->CreateApplication(j_app["Type"].get<std::string>());

              VLOG(IFM3D_TRACE) << "Created new app, updating our dump";
              current = this->ToJSON();
            }
          else
            {
              VLOG(IFM3D_TRACE) << "Getting index of existing application";
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
                          [this](const std::string& k,
                                 const std::string& v)
                          { this->pImpl->SetAppParameter(k,v); },
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
                          [this](const std::string& k,
                                 const std::string& v)
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

          if (! s_filt.is_null())
            {
              this->FromJSON_(curr_app["Imager"]["SpatialFilter"], s_filt,
                              [this](const std::string& k,
                                     const std::string& v)
                              { this->pImpl->SetSpatialFilterParameter(k,v); },
                              [this](){ this->pImpl->SaveApp(); },
                              "SpatialFilter", idx);
            }

          if (! t_filt.is_null())
            {
              this->FromJSON_(curr_app["Imager"]["TemporalFilter"], t_filt,
                              [this](const std::string& k,
                                     const std::string& v)
                              { this->pImpl->SetTemporalFilterParameter(k,v); },
                              [this](){ this->pImpl->SaveApp(); },
                              "TemporalFilter", idx);
            }
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
