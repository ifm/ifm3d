/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/device/device.h>
#include <string>
#include <sstream>
#include <vector>
#include <glog/logging.h>
#include <ifm3d/device/o3d.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/o3x.h>
#include <ifm3d/device/err.h>
#include <ifm3d/device/logging.h>
#include <device_impl.hpp>
#include <discovery.hpp>
#include <xmlrpc_wrapper.hpp>
#include <fmt/core.h>
#include <curl/curl.h>

//================================================
// Public constants
//================================================
const std::string ifm3d::DEFAULT_PASSWORD;
const std::uint16_t ifm3d::DEFAULT_XMLRPC_PORT = 80;
const int ifm3d::DEFAULT_PCIC_PORT = 50010;
const std::uint16_t ifm3d::PCIC_PORT = 0;
const std::string ifm3d::DEFAULT_IP = std::getenv("IFM3D_IP") == nullptr ?
                                        "192.168.0.69" :
                                        std::string(std::getenv("IFM3D_IP"));
const int ifm3d::MAX_HEARTBEAT = 300; // secs
const std::size_t ifm3d::SESSION_ID_SZ = 32;
const std::string ifm3d::DEFAULT_APPLICATION_TYPE = "Camera";

const long ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT = 10;     // seconds
const long ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT = 30; // seconds

auto ifm3d_session_id__ = []() -> std::string {
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
          if (!((sid.size() == ifm3d::SESSION_ID_SZ) &&
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
      LOG(WARNING) << "When trying to set default session id: " << ex.what();

      sid = "";
    }

  return sid;
};

const std::string ifm3d::DEFAULT_SESSION_ID = ifm3d_session_id__();

const int ifm3d::DEV_O3D_MIN = 1;
const int ifm3d::DEV_O3D_MAX = 255;
const int ifm3d::DEV_O3R_MIN = 768;
const int ifm3d::DEV_O3R_MAX = 1023;
const int ifm3d::DEV_O3X_MIN = 512;
const int ifm3d::DEV_O3X_MAX = 767;
const std::string ifm3d::ASSUME_DEVICE =
  std::getenv("IFM3D_DEVICE") == nullptr ?
    "" :
    std::string(std::getenv("IFM3D_DEVICE"));

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

const unsigned int ifm3d::O3X_DISTANCE_NOISE_IMAGE_SUPPORT_MAJOR = 1;
const unsigned int ifm3d::O3X_DISTANCE_NOISE_IMAGE_SUPPORT_MINOR = 1;
const unsigned int ifm3d::O3X_DISTANCE_NOISE_IMAGE_SUPPORT_PATCH = 190;

const ifm3d::SemVer O3R_MINIMUM_FIRWARE_SUPPORTED(0, 13, 13);

//================================================
// Function for Searching Devices on Network
//================================================

std::vector<ifm3d::IFMNetworkDevice>
ifm3d::Device::DeviceDiscovery()
{
  ifm3d::IFMDeviceDiscovery ifm_discovery;
  auto devices = ifm_discovery.NetworkSearch();
  return devices;
}

//================================================
// Factory function for making cameras
//================================================
ifm3d::Device::Ptr
ifm3d::Device::MakeShared(const std::string& ip,
                          const std::uint16_t xmlrpc_port,
                          const std::string& password,
                          bool throwIfUnavailable)
{
  auto base = std::make_shared<ifm3d::Device>(ip, xmlrpc_port);
  try
    {
      //
      // It is an optimization to return the specialized subclass
      //
      if (base->AmI(device_family::O3R))
        {
          if (base->CheckMinimumFirmwareVersion(
                O3R_MINIMUM_FIRWARE_SUPPORTED.major_num,
                O3R_MINIMUM_FIRWARE_SUPPORTED.minor_num,
                O3R_MINIMUM_FIRWARE_SUPPORTED.patch_num))
            {
              VLOG(IFM3D_TRACE) << "Instantiating O3R...";
              return std::make_shared<ifm3d::O3R>(ip, xmlrpc_port);
            }
          else
            {
              const std::string error_msg =
                fmt::format("Please update the firmware, minimum firmware "
                            "version required is {}",
                            O3R_MINIMUM_FIRWARE_SUPPORTED);

              VLOG(IFM3D_TRACE) << error_msg;
              throw Error(IFM3D_INVALID_FIRMWARE_VERSION, error_msg);
            }
        }
      if (base->AmI(device_family::O3X))
        {
          VLOG(IFM3D_TRACE) << "Instantiating O3X...";
          return std::make_shared<ifm3d::O3X>(ip, xmlrpc_port, password);
        }
      else if (base->AmI(device_family::O3D))
        {
          VLOG(IFM3D_TRACE) << "Instantiating O3D...";
          return std::make_shared<ifm3d::O3D>(ip, xmlrpc_port, password);
        }
      else
        {
          LOG(WARNING) << "Unexpected camera device type: "
                       << base->DeviceType();
        }
    }
  catch (const ifm3d::Error& ex)
    {
      if (ex.code() == IFM3D_XMLRPC_TIMEOUT)
        {
          LOG(WARNING) << "Could not probe device type: " << ex.what();
          if (throwIfUnavailable)
            {
              throw;
            }
        }
      else
        {
          LOG(ERROR) << "While trying to instantiate camera: " << ex.what();
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

ifm3d::Device::Device(const std::string& ip, const std::uint16_t xmlrpc_port)
  : pImpl(std::make_unique<Device::Impl>(
      std::make_shared<XMLRPCWrapper>(ip, xmlrpc_port))),
    device_type_("")
{}

ifm3d::Device::~Device() = default;

std::string
ifm3d::Device::IP()
{
  return this->pImpl->IP();
}

std::uint16_t
ifm3d::Device::XMLRPCPort()
{
  return this->pImpl->XMLRPCPort();
}

void
ifm3d::Device::Reboot(const ifm3d::Device::boot_mode& mode)
{
  this->pImpl->Reboot(static_cast<int>(mode));
}

std::vector<std::string>
ifm3d::Device::TraceLogs(int count)
{
  return this->pImpl->TraceLogs(count);
}

std::string
ifm3d::Device::DeviceParameter(const std::string& key)
{
  return this->pImpl->DeviceParameter(key);
}

std::string
ifm3d::Device::DeviceType(bool use_cached)
{
  if (!ifm3d::ASSUME_DEVICE.empty())
    {
      LOG(WARNING) << "Returning assumed device type: "
                   << ifm3d::ASSUME_DEVICE;
      return ifm3d::ASSUME_DEVICE;
    }

  if (!this->device_type_.empty())
    {
      if (use_cached)
        {
          return this->device_type_;
        }
    }

  this->device_type_ = this->pImpl->DeviceParameter("DeviceType");
  return this->device_type_;
}

int
ifm3d::Device::DeviceID()
{
  auto devType = this->DeviceType();
  auto pos = devType.find(':');
  if (pos != std::string::npos)
    {
      try
        {
          return std::atoi(devType.substr(pos + 1).c_str());
        }
      catch (std::out_of_range& ex)
        {
          LOG(WARNING) << ex.what();
        }
    }
  return -1;
}

bool
ifm3d::Device::checkDeviceID(int deviceID, int minID, int maxID)
{
  if ((deviceID >= minID) && (deviceID <= maxID))
    {
      return true;
    }
  return false;
}

ifm3d::Device::device_family
ifm3d::Device::WhoAmI()
{
  if (checkDeviceID(DeviceID(), ifm3d::DEV_O3D_MIN, ifm3d::DEV_O3D_MAX))
    {
      return device_family::O3D;
    }
  if (checkDeviceID(DeviceID(), ifm3d::DEV_O3X_MIN, ifm3d::DEV_O3X_MAX))
    {
      return device_family::O3X;
    }
  if (checkDeviceID(DeviceID(), ifm3d::DEV_O3R_MIN, ifm3d::DEV_O3R_MAX))
    {
      return device_family::O3R;
    }

  return device_family::UNKNOWN;
}

bool
ifm3d::Device::AmI(device_family family)
{
  return this->WhoAmI() == family;
}

bool
ifm3d::Device::CheckMinimumFirmwareVersion(unsigned int major,
                                           unsigned int minor,
                                           unsigned int patch)
{
  return this->pImpl->CheckMinimumFirmwareVersion(
    ifm3d::SemVer(major, minor, patch));
}

ifm3d::SemVer
ifm3d::Device::FirmwareVersion()
{
  return this->pImpl->FirmwareVersion();
}

void
ifm3d::Device::ForceTrigger()
{}

json
ifm3d::Device::ToJSON()
{
  throw ifm3d::Error(IFM3D_UNSUPPORTED_OP);
}

void
ifm3d::Device::FromJSON(const json& j)
{
  throw ifm3d::Error(IFM3D_UNSUPPORTED_OP);
}

std::string
ifm3d::Device::ToJSONStr()
{
  return this->ToJSON().dump(2);
}

void
ifm3d::Device::FromJSONStr(const std::string& jstr)
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
      throw ifm3d::Error(IFM3D_JSON_ERROR);
    }

  this->FromJSON(j);
}

std::shared_ptr<ifm3d::XMLRPCWrapper>
ifm3d::Device::XWrapper()
{
  return pImpl->XWrapper();
}

bool
isV1SWUpdate(const std::string& ip)
{
  /* SWU_V1 device expose a /id.lp endpoint in recovery, so we check for it's
   * existance to determine if this is a SWU_V1 device */
  auto curl = curl_easy_init();
  if (curl)
    {
      auto url = fmt::format("http://{}:8080/id.lp", ip);
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);

      curl_easy_setopt(curl,
                       CURLOPT_CONNECTTIMEOUT,
                       ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT);
      curl_easy_setopt(curl,
                       CURLOPT_TIMEOUT,
                       ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT);

      if (curl_easy_perform(curl) == CURLE_OK)
        {
          int response_code;
          curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

          if ((response_code >= 200) && (response_code < 300))
            {
              return true;
            }
        }
      curl_easy_cleanup(curl);

      curl = NULL;
    }

  return false;
}

ifm3d::Device::swu_version
ifm3d::Device::SwUpdateVersion()
{
  return isV1SWUpdate(this->IP()) ? ifm3d::Device::swu_version::SWU_V1 :
                                    ifm3d::Device::swu_version::SWU_V2;
}
