/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/camera/camera_base.h>
#include <string>
#include <sstream>
#include <vector>
#include <glog/logging.h>
#include <ifm3d/camera/camera_o3d.h>
#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/camera/camera_o3x.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <camera_base_impl.hpp>
#include <discovery.hpp>
#include <xmlrpc_wrapper.hpp>

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

auto __ifm3d_session_id__ = []() -> std::string {
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

const std::string ifm3d::DEFAULT_SESSION_ID = __ifm3d_session_id__();

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

//================================================
// Function for Searching Devices on Network
//================================================

std::vector<ifm3d::IFMNetworkDevice>
ifm3d::CameraBase::DeviceDiscovery()
{
  ifm3d::IFMDeviceDiscovery ifm_discovery;
  auto devices = ifm_discovery.NetworkSearch();
  return devices;
}

//================================================
// Factory function for making cameras
//================================================
ifm3d::CameraBase::Ptr
ifm3d::CameraBase::MakeShared(const std::string& ip,
                              const std::uint16_t xmlrpc_port,
                              const std::string& password)
{
  auto base = std::make_shared<ifm3d::CameraBase>(ip, xmlrpc_port);
  try
    {
      //
      // It is an optimization to return the specialized subclass
      //
      if (base->AmI(device_family::O3R))
        {
          VLOG(IFM3D_TRACE) << "Instantiating O3R...";
          return std::make_shared<ifm3d::O3RCamera>(ip, xmlrpc_port);
        }
      if (base->AmI(device_family::O3X))
        {
          VLOG(IFM3D_TRACE) << "Instantiating O3X...";
          return std::make_shared<ifm3d::O3XCamera>(ip, xmlrpc_port, password);
        }
      else if (base->AmI(device_family::O3D))
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
          LOG(WARNING) << "Could not probe device type: " << ex.what();
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

ifm3d::CameraBase::CameraBase(const std::string& ip,
                              const std::uint16_t xmlrpc_port)
  : pImpl(std::make_unique<CameraBase::Impl>(
      std::make_shared<XMLRPCWrapper>(ip, xmlrpc_port))),
    device_type_("")
{}

ifm3d::CameraBase::~CameraBase() = default;

std::string
ifm3d::CameraBase::IP()
{
  return this->pImpl->IP();
}

std::uint16_t
ifm3d::CameraBase::XMLRPCPort()
{
  return this->pImpl->XMLRPCPort();
}

void
ifm3d::CameraBase::Reboot(const ifm3d::CameraBase::boot_mode& mode)
{
  this->pImpl->Reboot(static_cast<int>(mode));
}

std::vector<std::string>
ifm3d::CameraBase::TraceLogs(int count)
{
  return this->pImpl->TraceLogs(count);
}

std::string
ifm3d::CameraBase::DeviceParameter(const std::string& key)
{
  return this->pImpl->DeviceParameter(key);
}

std::string
ifm3d::CameraBase::DeviceType(bool use_cached)
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
ifm3d::CameraBase::DeviceID()
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
ifm3d::CameraBase::checkDeviceID(int deviceID, int minID, int maxID)
{
  if ((deviceID >= minID) && (deviceID <= maxID))
    {
      return true;
    }
  return false;
}

ifm3d::CameraBase::device_family
ifm3d::CameraBase::WhoAmI()
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
ifm3d::CameraBase::AmI(device_family family)
{
  return this->WhoAmI() == family;
}

bool
ifm3d::CameraBase::CheckMinimumFirmwareVersion(unsigned int major,
                                               unsigned int minor,
                                               unsigned int patch)
{
  return this->pImpl->CheckMinimumFirmwareVersion(major, minor, patch);
}

void
ifm3d::CameraBase::ForceTrigger()
{}

json
ifm3d::CameraBase::ToJSON()
{
  return {};
}

void
ifm3d::CameraBase::FromJSON(const json& j)
{}

std::string
ifm3d::CameraBase::ToJSONStr()
{
  return this->ToJSON().dump(2);
}

void
ifm3d::CameraBase::FromJSONStr(const std::string& jstr)
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

std::shared_ptr<ifm3d::XMLRPCWrapper>
ifm3d::CameraBase::XWrapper()
{
  return pImpl->XWrapper();
}

ifm3d::CameraBase::swu_version
ifm3d::CameraBase::SwUpdateVersion()
{
  /*SWU_V1 is retured as device type is not avaliable
   in recovery mode and SWU_V2 doesnot support
   recovery mode hence device type is always SWU_V1*/
  return ifm3d::CameraBase::swu_version::SWU_V1;
}
