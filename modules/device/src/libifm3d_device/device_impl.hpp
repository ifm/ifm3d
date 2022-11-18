/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_CAMERA_BASE_IMPL_HPP
#define IFM3D_CAMERA_CAMERA_BASE_IMPL_HPP

#include <vector>
#include <fmt/ostream.h>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/device/logging.h>
#include <ifm3d/device/semver.h>
#include <xmlrpc_wrapper.hpp>

namespace ifm3d
{
  class XMLRPCWrapper;
  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_DEVICE_LOCAL Device::Impl
  {
  public:
    explicit Impl(std::shared_ptr<XMLRPCWrapper> xwrapper);
    ~Impl();

    // accessor/mutators
    std::shared_ptr<XMLRPCWrapper> XWrapper();
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

  protected:
    std::shared_ptr<XMLRPCWrapper> xwrapper_;
  }; // end: class Camera::Impl
} // end: namespace ifm3d

//============================================================
// Impl - Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------

ifm3d::Device::Impl::Impl(std::shared_ptr<XMLRPCWrapper> xwrapper)
  : xwrapper_(std::move(xwrapper))
{
  VLOG(IFM3D_TRACE) << "Initializing Camera: ip=" << this->IP()
                    << ", xmlrpc_port=" << this->XMLRPCPort();
  VLOG(IFM3D_TRACE) << "XMLRPC URL Prefix=" << this->XPrefix();
}

ifm3d::Device::Impl::~Impl() { VLOG(IFM3D_TRACE) << "Dtor..."; }

//-------------------------------------
// Accessor/mutators
//-------------------------------------

std::shared_ptr<ifm3d::XMLRPCWrapper>
ifm3d::Device::Impl::XWrapper()
{
  return this->xwrapper_;
}

std::string
ifm3d::Device::Impl::XPrefix()
{
  return this->xwrapper_->XPrefix();
}

std::string
ifm3d::Device::Impl::IP()
{
  return this->xwrapper_->IP();
}

std::uint16_t
ifm3d::Device::Impl::XMLRPCPort()
{
  return this->xwrapper_->XMLRPCPort();
}

// =============================================
// Public XMLRPC interface - worker methods
// =============================================

// ---------------------------------------------
// Main (no edit session necessary)
// ---------------------------------------------

std::string
ifm3d::Device::Impl::DeviceParameter(const std::string& param)
{
  return xmlrpc_c::value_string(
           this->xwrapper_->XCallMain("getParameter", param.c_str()))
    .cvalue();
}

bool
ifm3d::Device::Impl::CheckMinimumFirmwareVersion(
  const ifm3d::SemVer& minimum_version)
{
  return FirmwareVersion() >= minimum_version;
}

ifm3d::SemVer
ifm3d::Device::Impl::FirmwareVersion()
{
  auto data = this->xwrapper_->value_struct_to_map(
    this->xwrapper_->XCallMain("getSWVersion"));
  auto swversion = ifm3d::SemVer::Parse(data["IFM_Software"]);
  return swversion.value_or(ifm3d::SemVer(0, 0, 0));
}

std::vector<std::string>
ifm3d::Device::Impl::TraceLogs(int count)
{
  xmlrpc_c::value_array result(
    this->xwrapper_->XCallMain("getTraceLogs", count));
  std::vector<xmlrpc_c::value> const res_vec(result.vectorValueValue());

  std::vector<std::string> retval;
  for (auto& entry : res_vec)
    {
      xmlrpc_c::value_string const entry_str(entry);
      retval.push_back(static_cast<std::string>(entry_str));
    }
  return retval;
}

void
ifm3d::Device::Impl::Reboot(int mode)
{
  this->xwrapper_->XCallMain("reboot", mode);
}

#endif // IFM3D_CAMERA_CAMERA_BASE_IMPL_HPP