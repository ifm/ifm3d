/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <device_impl.hpp>
#include <discovery.hpp>
#include <exception>
#include <fmt/core.h> // NOLINT(*)
#include <httplib.h>
#include <ifm3d/common/err.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/ifm_network_device.h>
#include <ifm3d/device/o3d.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/o3x.h>
#include <ifm3d/device/semver.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <xmlrpc.hpp>

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

namespace
{
  const auto IFM3D_SESSION_ID = []() -> std::string {
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
            if ((sid.size() != ifm3d::SESSION_ID_SZ) ||
                (sid.find_first_not_of("0123456789abcdefABCDEF") !=
                 std::string::npos))
              {
                LOG_WARNING("Invalid session id: {}", sid);
                sid = "";
              }
            else
              {
                LOG_INFO("Default session id: {}", sid);
              }
          }
      }
    catch (const std::exception& ex)
      {
        LOG_WARNING("When trying to set default session id: {}", ex.what());

        sid = "";
      }

    return sid;
  };

  bool
  is_v1_sw_update(const std::string& ip)
  {
    /* SWU_V1 device expose a /id.lp endpoint in recovery, so we check for its
     * existence to determine if this is a SWU_V1 device */
    httplib::Client cli(ip, 8080);
    cli.set_connection_timeout(
      std::chrono::seconds(ifm3d::DEFAULT_CURL_CONNECT_TIMEOUT));
    cli.set_read_timeout(
      std::chrono::seconds(ifm3d::DEFAULT_CURL_TRANSACTION_TIMEOUT));

    auto res = cli.Head("/");

    if (res && res->status >= 200 && res->status < 300)
      {
        return true;
      }

    auto dev = ifm3d::Device(ip);
    return dev.WhoAmI() == ifm3d::Device::DeviceFamily::O3D ||
           dev.WhoAmI() == ifm3d::Device::DeviceFamily::O3X;
  }
}

const std::string ifm3d::DEFAULT_SESSION_ID = IFM3D_SESSION_ID();

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
// Function for Setting Temporary IP Address
//================================================

void
ifm3d::Device::SetTempIPAddress(const std::string& mac,
                                const std::string& temp_ip)
{
  ifm3d::IFMDeviceDiscovery ifm_discovery;
  ifm_discovery.SetTemporaryIP(mac, temp_ip);
}

//================================================
// Factory function for making cameras
//================================================
ifm3d::Device::Ptr
ifm3d::Device::MakeShared(const std::string& ip,
                          const std::uint16_t xmlrpc_port,
                          const std::string& password,
                          bool throw_if_unavailable)
{
  auto base = std::make_shared<ifm3d::Device>(ip, xmlrpc_port);
  try
    {
      //
      // It is an optimization to return the specialized subclass
      //
      if (base->AmI(DeviceFamily::O3R))
        {
          if (base->CheckMinimumFirmwareVersion(
                O3R_MINIMUM_FIRWARE_SUPPORTED.major_num,
                O3R_MINIMUM_FIRWARE_SUPPORTED.minor_num,
                O3R_MINIMUM_FIRWARE_SUPPORTED.patch_num))
            {
              LOG_VERBOSE("Instantiating O3R...");
              return std::make_shared<ifm3d::O3R>(ip, xmlrpc_port);
            }

          const std::string error_msg =
            fmt::format("Please update the firmware, minimum firmware "
                        "version required is {}",
                        O3R_MINIMUM_FIRWARE_SUPPORTED);

          LOG_VERBOSE(error_msg);
          throw Error(IFM3D_INVALID_FIRMWARE_VERSION, error_msg);
        }
      if (base->AmI(DeviceFamily::O3X))
        {
          LOG_VERBOSE("Instantiating O3X...");
          return std::make_shared<ifm3d::O3X>(ip, xmlrpc_port, password);
        }
      if (base->AmI(DeviceFamily::O3D))
        {
          LOG_VERBOSE("Instantiating O3D...");
          return std::make_shared<ifm3d::O3D>(ip, xmlrpc_port, password);
        }

      LOG_WARNING("Unexpected camera device type: {}", base->DeviceType());
    }
  catch (const ifm3d::Error& ex)
    {
      if (throw_if_unavailable)
        {
          LOG_WARNING("Could not probe device type: {}", ex.what());
          throw;
        }
    }

  //
  // worst case: we return the "non-optimized" base class
  //
  if (throw_if_unavailable)
    {
      LOG_WARNING("Returning instance of base camera class!");
    }
  return base;
}

//================================================
// Camera class - the public interface
//================================================

ifm3d::Device::Device(const std::string& ip, const std::uint16_t xmlrpc_port)
  : _impl(std::make_unique<Device::Impl>(
      std::make_shared<XMLRPC>(ip, xmlrpc_port)))
{}

ifm3d::Device::~Device() = default;

std::string
ifm3d::Device::IP()
{
  return this->_impl->IP();
}

std::uint16_t
ifm3d::Device::XMLRPCPort()
{
  return this->_impl->XMLRPCPort();
}

void
ifm3d::Device::Reboot(const ifm3d::Device::BootMode& mode)
{
  this->_impl->Reboot(static_cast<int>(mode));
}

std::vector<std::string>
ifm3d::Device::TraceLogs(int count)
{
  return this->_impl->TraceLogs(count);
}

std::string
ifm3d::Device::DeviceParameter(const std::string& key)
{
  return this->_impl->DeviceParameter(key);
}

std::string
ifm3d::Device::DeviceType(bool use_cached)
{
  if (!ifm3d::ASSUME_DEVICE.empty())
    {
      LOG_WARNING("Returning assumed device type: {}", ifm3d::ASSUME_DEVICE);
      return ifm3d::ASSUME_DEVICE;
    }

  if (!this->_device_type.empty())
    {
      if (use_cached)
        {
          return this->_device_type;
        }
    }

  this->_device_type = this->_impl->DeviceParameter("DeviceType");
  return this->_device_type;
}

int
ifm3d::Device::device_id()
{
  auto dev_type = this->DeviceType();
  auto pos = dev_type.find(':');
  if (pos != std::string::npos)
    {
      try
        {
          return std::atoi(dev_type.substr(pos + 1).c_str());
        }
      catch (std::out_of_range& ex)
        {
          LOG_WARNING(ex.what());
        }
    }
  return -1;
}

bool
ifm3d::Device::check_device_id(int device_id, int min_id, int max_id)
{
  return (device_id >= min_id) && (device_id <= max_id);
}

ifm3d::Device::DeviceFamily
ifm3d::Device::WhoAmI()
{
  if (check_device_id(device_id(), ifm3d::DEV_O3D_MIN, ifm3d::DEV_O3D_MAX))
    {
      return DeviceFamily::O3D;
    }
  if (check_device_id(device_id(), ifm3d::DEV_O3X_MIN, ifm3d::DEV_O3X_MAX))
    {
      return DeviceFamily::O3X;
    }
  if (check_device_id(device_id(), ifm3d::DEV_O3R_MIN, ifm3d::DEV_O3R_MAX))
    {
      return DeviceFamily::O3R;
    }

  return DeviceFamily::UNKNOWN;
}

bool
ifm3d::Device::AmI(DeviceFamily family)
{
  return this->WhoAmI() == family;
}

bool
ifm3d::Device::CheckMinimumFirmwareVersion(unsigned int major,
                                           unsigned int minor,
                                           unsigned int patch)
{
  return this->_impl->CheckMinimumFirmwareVersion(
    ifm3d::SemVer(major, minor, patch));
}

ifm3d::SemVer
ifm3d::Device::FirmwareVersion()
{
  return this->_impl->FirmwareVersion();
}

void
ifm3d::Device::ForceTrigger()
{}

ifm3d::json
ifm3d::Device::ToJSON()
{
  throw ifm3d::Error(IFM3D_UNSUPPORTED_OP);
}

void
ifm3d::Device::FromJSON(const json& /*j*/)
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
      LOG_ERROR("JSON: {}", ex.what());
      throw ifm3d::Error(IFM3D_JSON_ERROR);
    }
  this->FromJSON(j);
}

std::shared_ptr<ifm3d::XMLRPC>
ifm3d::Device::x_wrapper()
{
  return _impl->XWrapper();
}

ifm3d::Device::SWUVersion
ifm3d::Device::SwUpdateVersion()
{
  return is_v1_sw_update(this->IP()) ? ifm3d::Device::SWUVersion::SWU_V1 :
                                       ifm3d::Device::SWUVersion::SWU_V2;
}

ifm3d::json
ifm3d::Device::GetSWVersion()
{
  return this->_impl->GetSWVersion();
}