/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_CAMERA_BASE_IMPL_HPP
#define IFM3D_CAMERA_CAMERA_BASE_IMPL_HPP

#include <fmt/ostream.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/device/semver.h>
#include <vector>
#include <xmlrpc.hpp>

namespace ifm3d
{
  class XMLRPC;
  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_NO_EXPORT Device::Impl
  {
  public:
    explicit Impl(std::shared_ptr<XMLRPC> xwrapper);
    ~Impl();

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    // accessor/mutators
    std::shared_ptr<XMLRPC> XWrapper();
    std::string XPrefix();
    std::string IP();
    std::uint16_t XMLRPCPort();

    bool CheckMinimumFirmwareVersion(const SemVer& minimum_version);
    SemVer FirmwareVersion();
    //
    // public xmlrpc interface methods
    //

    // Main
    std::string DeviceParameter(const std::string& param);
    std::vector<std::string> TraceLogs(int count);
    void Reboot(int mode);
    json GetSWVersion();

  protected:
    std::shared_ptr<XMLRPC> _xwrapper;
  }; // end: class Camera::Impl
} // end: namespace ifm3d

//============================================================
// Impl - Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------

inline ifm3d::Device::Impl::Impl(std::shared_ptr<XMLRPC> xwrapper)
  : _xwrapper(std::move(xwrapper))
{
  LOG_VERBOSE("Initializing Camera: ip={}, xmlrpc_port={}",
              this->IP(),
              this->XMLRPCPort());
}

inline ifm3d::Device::Impl::~Impl() { LOG_VERBOSE("Dtor..."); }

//-------------------------------------
// Accessor/mutators
//-------------------------------------

inline std::shared_ptr<ifm3d::XMLRPC>
ifm3d::Device::Impl::XWrapper()
{
  return this->_xwrapper;
}

inline std::string
ifm3d::Device::Impl::IP()
{
  return this->_xwrapper->IP();
}

inline std::uint16_t
ifm3d::Device::Impl::XMLRPCPort()
{
  return this->_xwrapper->XMLRPCPort();
}

// =============================================
// Public XMLRPC interface - worker methods
// =============================================

// ---------------------------------------------
// Main (no edit session necessary)
// ---------------------------------------------

inline std::string
ifm3d::Device::Impl::DeviceParameter(const std::string& param)
{
  return this->_xwrapper->XCallMain("getParameter", param.c_str()).AsString();
}

inline bool
ifm3d::Device::Impl::CheckMinimumFirmwareVersion(
  const ifm3d::SemVer& minimum_version)
{
  return FirmwareVersion() >= minimum_version;
}

inline ifm3d::SemVer
ifm3d::Device::Impl::FirmwareVersion()
{
  auto data = this->_xwrapper->XCallMain("getSWVersion").AsMap();
  auto swversion = ifm3d::SemVer::Parse(data["IFM_Software"].AsString());
  return swversion.value_or(ifm3d::SemVer(0, 0, 0));
}

inline std::vector<std::string>
ifm3d::Device::Impl::TraceLogs(int count)
{
  auto result = this->_xwrapper->XCallMain("getTraceLogs", count).AsArray();

  std::vector<std::string> retval;
  retval.reserve(result.size());
  for (auto& entry : result)
    {
      retval.push_back(entry.AsString());
    }
  return retval;
}

inline void
ifm3d::Device::Impl::Reboot(int mode)
{
  this->_xwrapper->XCallMain("reboot", mode);
}

inline ifm3d::json
ifm3d::Device::Impl::GetSWVersion()
{
  return this->_xwrapper->XCallMain("getSWVersion").ToJson();
}

#endif // IFM3D_CAMERA_CAMERA_BASE_IMPL_HPP