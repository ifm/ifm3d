/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <memory>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/util.h>
#include <ifm3d/camera/version.h>
#include <camera_impl.hpp>

//================================================
// A lookup table listing the read-only camera
// parameters
//================================================
// clang-format off
std::unordered_map<std::string,
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
ifm3d::Camera::Ptr
ifm3d::Camera::MakeShared(const std::string& ip,
                          const std::uint16_t xmlrpc_port,
                          const std::string& password)
{
  auto base = std::dynamic_pointer_cast<ifm3d::Camera>(
    ifm3d::CameraBase::MakeShared(ip, xmlrpc_port, password));

  if (base == nullptr)
    {
      LOG(ERROR) << "Incompatible device";
      throw ifm3d::error_t(IFM3D_UNSUPPORTED_DEVICE);
    }

  return base;
}

ifm3d::Camera::Camera(const std::string& ip,
                      const std::uint16_t xmlrpc_port,
                      const std::string& password)
  : ifm3d::CameraBase::CameraBase(ip, xmlrpc_port),
    pImpl(std::make_unique<Camera::Impl>(XWrapper(), password))
{}

ifm3d::Camera::~Camera() = default;

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

std::unordered_map<std::string, std::string>
ifm3d::Camera::NetInfo()
{
  return this->pImpl->NetInfo();
}

std::unordered_map<std::string, std::string>
ifm3d::Camera::TimeInfo()
{
  // http://192.168.0.69:80/api/rpc/v1/com.ifm.efector/session_$XXX/edit/device/time/
  return json(this->pImpl->TimeInfo());
}

void
ifm3d::Camera::SetTemporaryApplicationParameters(
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
      LOG(WARNING) << "Setting temp params not supported by this device!";
      return;
    }

  this->pImpl->SetTemporaryApplicationParameters(params);
}

std::vector<std::uint8_t>
ifm3d::Camera::UnitVectors()
{
  if (this->AmI(device_family::O3X))
    {
      return this->pImpl->UnitVectors();
    }

  LOG(ERROR) << "The device does not support the XMLRPC UnitVectors accessor";
  throw ifm3d::error_t(IFM3D_UNSUPPORTED_OP);
}

int
ifm3d::Camera::ActiveApplication()
{
  if (this->AmI(device_family::O3X))
    {
      return 1;
    }

  return std::stoi(this->DeviceParameter("ActiveApplication"));
}

json
ifm3d::Camera::ApplicationList()
{
  json retval; // list

  int active = this->ActiveApplication();
  std::vector<ifm3d::app_entry_t> apps = this->pImpl->ApplicationList();

  for (auto& app : apps)
    {
      json dict = {{"Index", app.index},
                   {"Id", app.id},
                   {"Name", app.name},
                   {"Description", app.description},
                   {"Active", app.index == active}};

      retval.push_back(dict);
    }

  return retval;
}

std::vector<std::string>
ifm3d::Camera::ApplicationTypes()
{
  return this->pImpl->WrapInEditSession<std::vector<std::string>>(
    [this]() -> std::vector<std::string> {
      return this->pImpl->ApplicationTypes();
    });
}

std::vector<std::string>
ifm3d::Camera::ImagerTypes()
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
ifm3d::Camera::CreateApplication(const std::string& type)
{
  if (this->AmI(device_family::O3X))
    {
      LOG(ERROR) << "O3X only supports a single app, create not supported";
      throw ifm3d::error_t(IFM3D_UNSUPPORTED_OP);
    }

  return this->pImpl->WrapInEditSession<int>(
    [this, &type]() -> int { return this->pImpl->CreateApplication(type); });
}

int
ifm3d::Camera::CopyApplication(int idx)
{
  if (this->AmI(device_family::O3X))
    {
      LOG(ERROR) << "O3X only supports a single app, copy not supported";
      throw ifm3d::error_t(IFM3D_UNSUPPORTED_OP);
    }

  return this->pImpl->WrapInEditSession<int>(
    [this, idx]() -> int { return this->pImpl->CopyApplication(idx); });
}

void
ifm3d::Camera::DeleteApplication(int idx)
{
  if (this->AmI(device_family::O3X))
    {
      LOG(ERROR) << "O3X only supports a single app, delete not supported";
      throw ifm3d::error_t(IFM3D_UNSUPPORTED_OP);
    }

  this->pImpl->WrapInEditSession(
    [this, idx]() { this->pImpl->DeleteApplication(idx); });
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
    [this, epoch_secs]() { this->pImpl->SetCurrentTime(epoch_secs); });
}

std::vector<std::uint8_t>
ifm3d::Camera::ExportIFMConfig()
{
  return this->pImpl->WrapInEditSession<std::vector<std::uint8_t>>(
    [this]() -> std::vector<std::uint8_t> {
      return this->pImpl->ExportIFMConfig();
    });
}

std::vector<std::uint8_t>
ifm3d::Camera::ExportIFMApp(int idx)
{
  return this->pImpl->WrapInEditSession<std::vector<std::uint8_t>>(
    [this, idx]() -> std::vector<std::uint8_t> {
      return this->pImpl->ExportIFMApp(idx);
    });
}

void
ifm3d::Camera::ImportIFMConfig(const std::vector<std::uint8_t>& bytes,
                               std::uint16_t flags)
{
  return this->pImpl->WrapInEditSession(
    [this, &bytes, flags]() { this->pImpl->ImportIFMConfig(bytes, flags); });
}

int
ifm3d::Camera::ImportIFMApp(const std::vector<std::uint8_t>& bytes)
{
  return this->pImpl->WrapInEditSession<int>(
    [this, &bytes]() -> int { return this->pImpl->ImportIFMApp(bytes); });
}

json
ifm3d::Camera::getApplicationInfosToJSON()
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
      std::unordered_map<std::string, std::string> spatialFil = {
        {"MaskSize", "0"}};
      std::unordered_map<std::string, std::string> tmpFil = {{}}; // empty map
      imager_json["SpatialFilter"] = json(spatialFil);
      imager_json["TemporalFilter"] = json(tmpFil);

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

json
ifm3d::Camera::ToJSON_(const bool open_session)
{
  auto timeNow =
    std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::ostringstream time_buf;
  time_buf << std::ctime(&timeNow);
  auto time_s = time_buf.str();
  ifm3d::trim(time_s);

  auto exec_toJSON = [this]() {
    return std::make_tuple(this->getApplicationInfosToJSON(),
                           json(this->NetInfo()),
                           this->TimeInfo());
  };

  json app_info, net_info, time_info;
  if (open_session)
    {
      std::tie(app_info, net_info, time_info) =
        this->pImpl->WrapInEditSession<std::tuple<json, json, json>>(
          exec_toJSON);
    }
  else
    {
      std::tie(app_info, net_info, time_info) = exec_toJSON();
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

json
ifm3d::Camera::ToJSON()
{
  return ToJSON_();
}

void
ifm3d::Camera::FromJSON_(
  const json& j_curr,
  const json& j_new,
  std::function<void(const std::string&, const std::string&)> SetFunc,
  std::function<void()> SaveFunc,
  const std::string& name,
  int idx)
{
  VLOG(IFM3D_TRACE) << "Setting " << name << " parameters";
  if (!j_new.is_object())
    {
      LOG(ERROR) << "The passed in " << name << " json should be an object!";
      VLOG(IFM3D_TRACE) << "Invalid JSON was: " << j_new.dump();

      throw ifm3d::error_t(IFM3D_JSON_ERROR);
    }

  if (idx > 0)
    {
      if (!this->AmI(device_family::O3X))
        {
          VLOG(IFM3D_TRACE) << "Editing app at idx=" << idx;
          this->pImpl->EditApplication(idx);
        }
    }

  bool do_save = false;
  for (auto it = j_new.begin(); it != j_new.end(); ++it)
    {
      std::string key = it.key();
      VLOG(IFM3D_TRACE) << "Processing key=" << key
                        << ", with val=" << j_new[key].dump(2);
      if (it.value().is_null())
        {
          LOG(WARNING) << "Skipping " << key
                       << ", null value -- should be string!";
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
                        << "Skipping read-only " << name << " param: " << key;
                      continue;
                    }
                }
              catch (const std::out_of_range& /*ex*/)
                {
                  // just swallow the error -- we are setting a
                  // r/w parameter
                }

              VLOG(IFM3D_TRACE)
                << "Setting " << name << " parameter: " << key << "=" << val;
              SetFunc(key, val);
              do_save = true;
            }
          catch (const ifm3d::error_t& ex)
            {
              if (ex.code() == IFM3D_READONLY_PARAM)
                {
                  LOG(WARNING)
                    << "Tried to set read-only " << name << " param: " << key;
                }
              else
                {
                  throw;
                }
            }
        }
      else
        {
          VLOG(IFM3D_TRACE) << "Skipping " << key << ", no change in value";
        }
    }

  if (do_save)
    {
      SaveFunc();
    }
  if (idx > 0)
    {
      if (!this->AmI(device_family::O3X))
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
  if (!j.is_object())
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
  this->pImpl->WrapInEditSession([this, &root, &j, &current]() {
    // Device
    json j_dev = root["Device"];
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
            LOG(ERROR) << "The `Apps` element should be an array!";
            VLOG(IFM3D_TRACE) << "Invalid JSON was: " << j_apps.dump();

            throw ifm3d::error_t(IFM3D_JSON_ERROR);
          }

        VLOG(IFM3D_TRACE) << "Looping over applications";
        for (auto& j_app : j_apps)
          {
            if (!j_app.is_object())
              {
                LOG(ERROR) << "All 'Apps' must be a JSON object!";
                VLOG(IFM3D_TRACE) << "Invalid JSON was: " << j_app.dump();
                throw ifm3d::error_t(IFM3D_JSON_ERROR);
              }

            // First we determine if we are editing an existing application or
            // if we are creating a new one. If no index is specified, we
            // create a new application.
            int idx = -1;
            if (j_app["Index"].is_null())
              {
                if (!this->AmI(device_family::O3X))
                  {
                    VLOG(IFM3D_TRACE) << "Creating new application";
                    idx = j_app["Type"].is_null() ?
                            this->pImpl->CreateApplication(
                              DEFAULT_APPLICATION_TYPE) :
                            this->pImpl->CreateApplication(
                              j_app["Type"].get<std::string>());

                    VLOG(IFM3D_TRACE) << "Created new app, updating our dump";
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
                VLOG(IFM3D_TRACE) << "Getting index of existing application";
                idx = std::stoi(j_app["Index"].get<std::string>());
              }

            VLOG(IFM3D_TRACE) << "Application of interest is at index=" << idx;

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

            if (!app_found)
              {
                LOG(ERROR) << "Could not find an application at index=" << idx;
                throw ifm3d::error_t(IFM3D_JSON_ERROR);
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

            json s_filt = j_im["SpatialFilter"];
            if (!s_filt.is_null())
              {
                j_im.erase("SpatialFilter");
              }

            json t_filt = j_im["TemporalFilter"];
            if (!t_filt.is_null())
              {
                j_im.erase("TemporalFilter");
              }

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
              [this]() { this->pImpl->SaveApp(); },
              "Imager",
              idx);

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
        json j_time = root["Time"];
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
    json j_net = root["Net"];
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
            catch (const ifm3d::error_t& ex)
              {
                if (ex.code() == IFM3D_XMLRPC_TIMEOUT)
                  {
                    LOG(WARNING) << "XML-RPC timeout saving net params, "
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
ifm3d::Camera::SetPassword(std::string password)
{
  this->pImpl->WrapInEditSession([this, password]() {
    password == "" ? this->pImpl->DisablePassword() :
                     this->pImpl->ActivatePassword(password);
    this->pImpl->SaveDevice();
  });
}
void
ifm3d::Camera::ForceTrigger()
{
  if (this->AmI(device_family::O3X))
    {
      return this->pImpl->ForceTrigger();
    }
}