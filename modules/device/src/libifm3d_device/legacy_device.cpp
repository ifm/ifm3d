/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdint>
#include "ifm3d/common/err.h"
#include "ifm3d/device/device.h"
#include "ifm3d/common/logging/log.h"
#include "ifm3d/common/json_impl.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <ctime>
#include <sstream>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/device/util.h>
#include <ifm3d/device/version.h>
#include <legacy_device_impl.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <tuple>
#include <stdexcept>
//================================================
// A lookup table listing the read-only camera
// parameters
//================================================
// clang-format off
static std::unordered_map<std::string,
                   std::unordered_map<std::string, bool>>
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
// clang-format on

//================================================
// Factory function for making cameras
//================================================
ifm3d::LegacyDevice::Ptr
ifm3d::LegacyDevice::MakeShared(const std::string& ip,
                                const std::uint16_t xmlrpc_port,
                                const std::string& password)
{
  auto base = std::dynamic_pointer_cast<ifm3d::LegacyDevice>(
    ifm3d::Device::MakeShared(ip, xmlrpc_port, password));

  if (base == nullptr)
    {
      LOG_ERROR("Incompatible device");
      throw ifm3d::Error(IFM3D_UNSUPPORTED_DEVICE);
    }

  return base;
}

ifm3d::LegacyDevice::LegacyDevice(const std::string& ip,
                                  const std::uint16_t xmlrpc_port,
                                  const std::string& password)
  : ifm3d::Device::Device(ip, xmlrpc_port),
    pImpl(std::make_unique<LegacyDevice::Impl>(XWrapper(), password))
{}

ifm3d::LegacyDevice::~LegacyDevice() = default;

std::string
ifm3d::LegacyDevice::Password()
{
  return this->pImpl->Password();
}

std::string
ifm3d::LegacyDevice::SessionID()
{
  return this->pImpl->SessionID();
}

std::string
ifm3d::LegacyDevice::RequestSession()
{
  return this->pImpl->RequestSession();
}

bool
ifm3d::LegacyDevice::CancelSession()
{
  return this->pImpl->CancelSession();
}

bool
ifm3d::LegacyDevice::CancelSession(const std::string& sid)
{
  return this->pImpl->CancelSession(sid);
}

int
ifm3d::LegacyDevice::Heartbeat(int hb)
{
  return this->pImpl->Heartbeat(hb);
}

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::NetInfo()
{
  return this->pImpl->NetInfo();
}

std::unordered_map<std::string, std::string>
ifm3d::LegacyDevice::TimeInfo()
{
  // http://192.168.0.69:80/api/rpc/v1/com.ifm.efector/session_$XXX/edit/device/time/
  return json(this->pImpl->TimeInfo());
}

void
ifm3d::LegacyDevice::SetTemporaryApplicationParameters(
  const std::unordered_map<std::string, std::string>& params)
{
  //
  // NOTE: we "fail" silently here. Assumption is a closed loop check by the
  // user to see if/when the temp params take effect.
  //
  if (this->AmI(device_family::O3D) && (!this->CheckMinimumFirmwareVersion(
                                         ifm3d::O3D_TMP_PARAMS_SUPPORT_MAJOR,
                                         ifm3d::O3D_TMP_PARAMS_SUPPORT_MINOR,
                                         ifm3d::O3D_TMP_PARAMS_SUPPORT_PATCH)))
    {
      LOG_WARNING("Setting temp params not supported by this device!");
      return;
    }

  this->pImpl->SetTemporaryApplicationParameters(params);
}

std::vector<std::uint8_t>
ifm3d::LegacyDevice::UnitVectors()
{
  if (this->AmI(device_family::O3X))
    {
      return this->pImpl->UnitVectors();
    }

  LOG_ERROR("The device does not support the XMLRPC UnitVectors accessor");
  throw ifm3d::Error(IFM3D_UNSUPPORTED_OP);
}

int
ifm3d::LegacyDevice::ActiveApplication()
{
  if (this->AmI(device_family::O3X))
    {
      return 1;
    }

  return std::stoi(this->DeviceParameter("ActiveApplication"));
}

ifm3d::json
ifm3d::LegacyDevice::ApplicationList()
{
  json retval; // list

  int const active = this->ActiveApplication();
  std::vector<ifm3d::app_entry_t> apps = this->pImpl->ApplicationList();
  if (apps.empty())
    {
      return ifm3d::json::array();
    }
  for (auto& app : apps)
    {
      json const dict = {{"Index", app.index},
                         {"Id", app.id},
                         {"Name", app.name},
                         {"Description", app.description},
                         {"Active", app.index == active}};

      retval.push_back(dict);
    }

  return retval;
}

std::vector<std::string>
ifm3d::LegacyDevice::ApplicationTypes()
{
  return this->pImpl->WrapInEditSession<std::vector<std::string>>(
    [this]() -> std::vector<std::string> {
      return this->pImpl->ApplicationTypes();
    });
}

std::vector<std::string>
ifm3d::LegacyDevice::ImagerTypes()
{
  return this->pImpl->WrapInEditSession<std::vector<std::string>>(
    [this]() -> std::vector<std::string> {
      if (!this->AmI(device_family::O3X))
        {
          this->pImpl->EditApplication(this->ActiveApplication());
        }
      return this->pImpl->ImagerTypes();
    });
}

int
ifm3d::LegacyDevice::CreateApplication(const std::string& type)
{
  if (this->AmI(device_family::O3X))
    {
      LOG_ERROR("O3X only supports a single app, create not supported");
      throw ifm3d::Error(IFM3D_UNSUPPORTED_OP);
    }

  return this->pImpl->WrapInEditSession<int>(
    [this, &type]() -> int { return this->pImpl->CreateApplication(type); });
}

int
ifm3d::LegacyDevice::CopyApplication(int idx)
{
  if (this->AmI(device_family::O3X))
    {
      LOG_ERROR("O3X only supports a single app, copy not supported");
      throw ifm3d::Error(IFM3D_UNSUPPORTED_OP);
    }

  return this->pImpl->WrapInEditSession<int>(
    [this, idx]() -> int { return this->pImpl->CopyApplication(idx); });
}

void
ifm3d::LegacyDevice::DeleteApplication(int idx)
{
  if (this->AmI(device_family::O3X))
    {
      LOG_ERROR("O3X only supports a single app, delete not supported");
      throw ifm3d::Error(IFM3D_UNSUPPORTED_OP);
    }

  this->pImpl->WrapInEditSession(
    [this, idx]() { this->pImpl->DeleteApplication(idx); });
}

void
ifm3d::LegacyDevice::FactoryReset()
{
  this->pImpl->WrapInEditSession([this]() { this->pImpl->FactoryReset(); });
}

void
ifm3d::LegacyDevice::SetCurrentTime(int epoch_secs)
{
  this->pImpl->WrapInEditSession(
    [this, epoch_secs]() { this->pImpl->SetCurrentTime(epoch_secs); });
}

std::vector<std::uint8_t>
ifm3d::LegacyDevice::ExportIFMConfig()
{
  return this->pImpl->WrapInEditSession<std::vector<std::uint8_t>>(
    [this]() -> std::vector<std::uint8_t> {
      return this->pImpl->ExportIFMConfig();
    });
}

std::vector<std::uint8_t>
ifm3d::LegacyDevice::ExportIFMApp(int idx)
{
  return this->pImpl->WrapInEditSession<std::vector<std::uint8_t>>(
    [this, idx]() -> std::vector<std::uint8_t> {
      return this->pImpl->ExportIFMApp(idx);
    });
}

void
ifm3d::LegacyDevice::ImportIFMConfig(const std::vector<std::uint8_t>& bytes,
                                     std::uint16_t flags)
{
  this->pImpl->WrapInEditSession(
    [this, &bytes, flags]() { this->pImpl->ImportIFMConfig(bytes, flags); });
}

int
ifm3d::LegacyDevice::ImportIFMApp(const std::vector<std::uint8_t>& bytes)
{
  return this->pImpl->WrapInEditSession<int>(
    [this, &bytes]() -> int { return this->pImpl->ImportIFMApp(bytes); });
}

ifm3d::json
ifm3d::LegacyDevice::getApplicationInfosToJSON()
{
  auto app_info = json::parse("[]");
  auto app_list = this->ApplicationList();

  for (auto& app : app_list)
    {
      auto idx = app["Index"].get<int>();
      if (!this->AmI(device_family::O3X))
        {
          this->pImpl->EditApplication(idx);
        }

      auto app_json = json(this->pImpl->AppInfo());
      app_json["Index"] = std::to_string(idx);
      app_json["Id"] = std::to_string(app["Id"].get<int>());

      auto imager_json = json(this->pImpl->ImagerInfo());

      /* Initialize the imager_json filters with default values for
         compatibility with o3x which does not support dedicated xmlrpc
         filter objects since there are no config options for the o3x filter
       */
      std::unordered_map<std::string, std::string> const spatial_fil = {
        {"MaskSize", "0"}};
      std::unordered_map<std::string, std::string> const tmp_fil = {
        {}}; // empty map
      imager_json["SpatialFilter"] = json(spatial_fil);
      imager_json["TemporalFilter"] = json(tmp_fil);

      if (this->AmI(device_family::O3D))
        {
          auto sfilt_json = json(this->pImpl->SpatialFilterInfo());
          imager_json["SpatialFilter"] = sfilt_json;

          auto tfilt_json = json(this->pImpl->TemporalFilterInfo());
          imager_json["TemporalFilter"] = tfilt_json;
        }

      app_json["Imager"] = imager_json;

      app_info.push_back(app_json);

      if (!this->AmI(device_family::O3X))
        {
          this->pImpl->StopEditingApplication();
        }
    }
  return app_info;
}

ifm3d::json
ifm3d::LegacyDevice::ToJSON_(const bool open_session)
{
  auto time_now =
    std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::ostringstream time_buf;
  time_buf << std::ctime(&time_now);
  auto time_s = time_buf.str();
  ifm3d::trim(time_s);

  auto exec_to_json = [this]() {
    return std::make_tuple(this->getApplicationInfosToJSON(),
                           json(this->NetInfo()),
                           this->TimeInfo());
  };

  json app_info;
  json net_info;
  json time_info;
  if (open_session)
    {
      std::tie(app_info, net_info, time_info) =
        this->pImpl->WrapInEditSession<std::tuple<json, json, json>>(
          exec_to_json);
    }
  else
    {
      std::tie(app_info, net_info, time_info) = exec_to_json();
    }

  // clang-format off
  json j =
    {
      {
       "ifm3d",
       {
         {"_",
          {
            {std::string(IFM3D_LIBRARY_NAME) + "_version", IFM3D_VERSION},
            {"Date", time_s},
            {"HWInfo", json(pImpl->HWInfo())},
            {"SWVersion", json(pImpl->SWVersion())}
          }
         },
         {"Device", json(pImpl->DeviceInfo())},
         {"Net", net_info},
         {"Time", time_info},
         {"Apps", app_info}
       }
      }
    };
  // clang-format on

  return j;
}

ifm3d::json
ifm3d::LegacyDevice::ToJSON()
{
  return ToJSON_();
}

void
ifm3d::LegacyDevice::FromJSON_(
  const json& j_curr,
  const json& j_new,
  const std::function<void(const std::string&, const std::string&)>& set_func,
  const std::function<void()>& save_func,
  const std::string& name,
  int idx)
{
  LOG_VERBOSE("Setting {} parameters", name);
  if (!j_new.is_object())
    {
      LOG_ERROR("The passed in {} json should be an object!", name);
      LOG_VERBOSE("Invalid JSON was: {}", j_new.dump());

      throw ifm3d::Error(IFM3D_JSON_ERROR);
    }

  if (idx > 0)
    {
      if (!this->AmI(device_family::O3X))
        {
          LOG_VERBOSE("Editing app at idx={}", idx);
          this->pImpl->EditApplication(idx);
        }
    }

  bool do_save = false;
  for (auto it = j_new.begin(); it != j_new.end(); ++it)
    {
      std::string const& key = it.key();
      LOG_VERBOSE("Processing key={} with val={}", key, j_new[key].dump(2));
      if (it.value().is_null())
        {
          LOG_WARNING("Skipping {}, null value -- should be string!", key);
          continue;
        }
      std::string const val = j_new[key].get<std::string>();
      if (j_curr[key].is_null())
        {
          const auto msg =
            key + std::string(" parameter is not supported in firmware");
          throw std::runtime_error(msg);
        }
      if (j_curr[key].get<std::string>() != val)
        {
          try
            {
              try
                {
                  if (RO_LUT.at(name).at(key))
                    {
                      LOG_VERBOSE("Skipping read-only {} param: {}",
                                  name,
                                  key);
                      continue;
                    }
                }
              catch (const std::out_of_range& /*ex*/)
                {
                  // IGNORE: just swallow the error -- we are setting a
                  // r/w parameter
                }

              LOG_VERBOSE("Setting {} parameter: {}={}", name, key, val);
              set_func(key, val);
              do_save = true;
            }
          catch (const ifm3d::Error& ex)
            {
              if (ex.code() == IFM3D_READONLY_PARAM)
                {
                  LOG_WARNING("Tried to set read-only {} param: {}",
                              name,
                              key);
                }
              else
                {
                  throw;
                }
            }
        }
      else
        {
          LOG_VERBOSE("Skipping {}, no change in value", val);
        }
    }

  if (do_save)
    {
      save_func();
    }
  if (idx > 0)
    {
      if (!this->AmI(device_family::O3X))
        {
          LOG_VERBOSE("Stop editing app at idx={}", idx);
          this->pImpl->StopEditingApplication();
        }
    }
}

bool
ifm3d::LegacyDevice::getAppJSON(int index, const json& j, json& app) /*static*/
{
  bool app_found = false;
  app = json({});

  json curr_apps = j["ifm3d"]["Apps"];

  // Just find application with current index
  for (auto& a : curr_apps)
    {
      if (std::stoi(a["Index"].get<std::string>()) == index)
        {
          app = a;
          app_found = true;
          break;
        }
    }

  return (app_found);
}

void
ifm3d::LegacyDevice::FromJSON(const json& j)
{
  LOG_VERBOSE("Checking if passed in JSON is an object");
  if (!j.is_object())
    {
      LOG_ERROR("The passed in json should be an object!");
      LOG_VERBOSE("Invalid JSON was: {}", j.dump());

      throw ifm3d::Error(IFM3D_JSON_ERROR);
    }

  // we use this to lessen the number of overall network calls
  LOG_VERBOSE("Caching current camera dump");
  json current = this->ToJSON();

  // make the `ifm3d` root element optional
  LOG_VERBOSE("Extracing root element");
  json root = j.count("ifm3d") ? j["ifm3d"] : j;

  // Ensure we cancel the session when leaving this method
  this->pImpl->WrapInEditSession([this, &root, &j, &current]() {
    // Device
    json const j_dev = root["Device"];
    if (!j_dev.is_null())
      {
        this->FromJSON_(
          current["ifm3d"]["Device"],
          j_dev,
          [this](const std::string& k, const std::string& v) {
            this->pImpl->SetDeviceParameter(k, v);
          },
          [this]() { this->pImpl->SaveDevice(); },
          "Device");
      }

    // Apps - requires careful/special treatment
    json j_apps = root["Apps"];
    if (!j_apps.is_null())
      {
        if (!j_apps.is_array())
          {
            LOG_ERROR("The `Apps` element should be an array!");
            LOG_VERBOSE("Invalid JSON was: {}", j_apps.dump());

            throw ifm3d::Error(IFM3D_JSON_ERROR);
          }

        LOG_VERBOSE("Looping over applications");
        for (auto& j_app : j_apps)
          {
            if (!j_app.is_object())
              {
                LOG_ERROR("All 'Apps' must be a JSON object!");
                LOG_VERBOSE("Invalid JSON was: {}", j_app.dump());
                throw ifm3d::Error(IFM3D_JSON_ERROR);
              }

            // First we determine if we are editing an existing application or
            // if we are creating a new one. If no index is specified, we
            // create a new application.
            int idx = -1;
            if (j_app["Index"].is_null())
              {
                if (!this->AmI(device_family::O3X))
                  {
                    LOG_VERBOSE("Creating new application");
                    idx = j_app["Type"].is_null() ?
                            this->pImpl->CreateApplication(
                              DEFAULT_APPLICATION_TYPE) :
                            this->pImpl->CreateApplication(
                              j_app["Type"].get<std::string>());

                    LOG_VERBOSE("Created new app, updating our dump");
                    current = this->ToJSON_(false);
                  }
                else
                  {
                    LOG_VERBOSE("O3X only has a single app, assuming idx=1");
                    idx = 1;
                  }
              }
            else
              {
                LOG_VERBOSE("Getting index of existing application");
                idx = std::stoi(j_app["Index"].get<std::string>());
              }

            LOG_VERBOSE("Application of interest is at index={}", idx);

            // now in `current` (which is a whole camera dump)
            // we need to find the application at index `idx`.
            json curr_app = json({});
            bool app_found = getAppJSON(idx, current, curr_app);
            if (!app_found)
              {
                LOG_ERROR("Could not find an application at index={}", idx);
                throw ifm3d::Error(IFM3D_JSON_ERROR);
              }

            // at this point both the new and current
            // should have the same index and type and
            // we want to make sure of that.
            j_app["Index"] = curr_app["Index"];
            j_app["Type"] = curr_app["Type"];

            // pull out the imager sub-tree (we treat that separately)
            json j_im = j_app["Imager"];
            if (!j_im.is_null())
              {
                j_app.erase("Imager");
              }

            this->FromJSON_(
              curr_app,
              j_app,
              [this](const std::string& k, const std::string& v) {
                this->pImpl->SetAppParameter(k, v);
              },
              [this]() { this->pImpl->SaveApp(); },
              "App",
              idx);

            json const s_filt = j_im["SpatialFilter"];
            if (!s_filt.is_null())
              {
                j_im.erase("SpatialFilter");
              }

            json const t_filt = j_im["TemporalFilter"];
            if (!t_filt.is_null())
              {
                j_im.erase("TemporalFilter");
              }

            // When the TemporalFilterType is set for example, there are
            // additional parameters. They do not exist in the basic
            // application. When trying to import them, ifm3d will fail. In
            // case an temporal filter is set, we have to reload the
            // application.
            bool const reload_app =
              (!j_im["TemporalFilterType"].is_null() &&
               std::stoi(j_im["TemporalFilterType"].get<std::string>()) != 0);

            this->FromJSON_(
              curr_app["Imager"],
              j_im,
              [this](const std::string& k, const std::string& v) {
                if (k == "Type")
                  {
                    this->pImpl->ChangeImagerType(v);
                  }
                else
                  {
                    this->pImpl->SetImagerParameter(k, v);
                  }
              },
              [this, j_im]() { this->pImpl->SaveApp(); },
              "Imager",
              idx);

            if (reload_app)
              {
                current = this->ToJSON_(false);
                app_found = getAppJSON(idx, current, curr_app);
              }

            if (!this->AmI(device_family::O3X))
              {

                if (!s_filt.is_null())
                  {
                    this->FromJSON_(
                      curr_app["Imager"]["SpatialFilter"],
                      s_filt,
                      [this](const std::string& k, const std::string& v) {
                        this->pImpl->SetSpatialFilterParameter(k, v);
                      },
                      [this]() { this->pImpl->SaveApp(); },
                      "SpatialFilter",
                      idx);
                  }

                if (!t_filt.is_null())
                  {
                    this->FromJSON_(
                      curr_app["Imager"]["TemporalFilter"],
                      t_filt,
                      [this](const std::string& k, const std::string& v) {
                        this->pImpl->SetTemporalFilterParameter(k, v);
                      },
                      [this]() { this->pImpl->SaveApp(); },
                      "TemporalFilter",
                      idx);
                  }
              }
          }
      }

    // Time
    if (this->AmI(device_family::O3X) ||
        (this->AmI(device_family::O3D) &&
         this->CheckMinimumFirmwareVersion(ifm3d::O3D_TIME_SUPPORT_MAJOR,
                                           ifm3d::O3D_TIME_SUPPORT_MINOR,
                                           ifm3d::O3D_TIME_SUPPORT_PATCH)))
      {
        json const j_time = root["Time"];
        if (!j_time.is_null())
          {
            this->FromJSON_(
              current["ifm3d"]["Time"],
              j_time,
              [this](const std::string& k, const std::string& v) {
                this->pImpl->SetTimeParameter(k, v);
              },
              [this]() { this->pImpl->SaveTime(); },
              "Time");
          }
      }

    // Network - we do this last intentionally!
    json const j_net = root["Net"];
    if (!j_net.is_null())
      {
        this->FromJSON_(
          current["ifm3d"]["Net"],
          j_net,
          [this](const std::string& k, const std::string& v) {
            this->pImpl->SetNetParameter(k, v);
          },
          [this]() { // we are changing network parameters,
                     // we expect a timeout!
            try
              {
                this->pImpl->SaveNet();
              }
            catch (const ifm3d::Error& ex)
              {
                if (ex.code() == IFM3D_XMLRPC_TIMEOUT)
                  {
                    LOG_WARNING(
                      "XML-RPC timeout saving net params, this is expecte");
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
ifm3d::LegacyDevice::SetPassword(const std::string& password)
{
  this->pImpl->WrapInEditSession([this, password]() {
    password.empty() ? this->pImpl->DisablePassword() :
                       this->pImpl->ActivatePassword(password);
    this->pImpl->SaveDevice();
  });
}
void
ifm3d::LegacyDevice::ForceTrigger()
{
  if (this->AmI(device_family::O3X))
    {
      this->pImpl->ForceTrigger();
      return;
    }
}
