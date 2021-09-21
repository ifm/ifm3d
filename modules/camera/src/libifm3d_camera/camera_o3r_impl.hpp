// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_CAMERA_O3R_IMPL_HPP
#define IFM3D_CAMERA_CAMERA_O3R_IMPL_HPP

#include <chrono>
#include <cstdint>
#include <ctime>
#include <functional>
#include <map>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>
#include <glog/logging.h>
#include <xmlrpc-c/client.hpp>
#include <ifm3d/camera/camera_o3r.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <xmlrpc_wrapper.hpp>

namespace ifm3d
{
  class XMLRPCWrapper;

  //============================================================
  // Impl interface
  //============================================================
  class O3RCamera::Impl
  {
  public:
    explicit Impl(std::shared_ptr<XMLRPCWrapper> xwrapper);
    ~Impl();

    std::string GetTemporaryConfiguration();
    void SetTemporaryConfiguration(const std::string& config);
    void SaveInitConfiguration();
    void FactoryReset(bool keepNetworkSettings);
    void Reboot();
    json Schema();

  protected:
    std::shared_ptr<XMLRPCWrapper> xwrapper_;
  }; // end: class O3RCamera::Impl

} // end: namespace ifm3d

ifm3d::O3RCamera::Impl::Impl(std::shared_ptr<XMLRPCWrapper> xwrapper)
  : xwrapper_(std::move(xwrapper))
{}

ifm3d::O3RCamera::Impl::~Impl() {}

std::string
ifm3d::O3RCamera::Impl::GetTemporaryConfiguration()
{
  return xmlrpc_c::value_string(
           this->xwrapper_->XCallMain("get", std::vector<std::string>()))
    .cvalue();
}

void
ifm3d::O3RCamera::Impl::SetTemporaryConfiguration(const std::string& config)
{
  this->xwrapper_->XCallMain("set", config);
}

void
ifm3d::O3RCamera::Impl::SaveInitConfiguration()
{
  this->xwrapper_->XCallMain("saveInit");
}

void
ifm3d::O3RCamera::Impl::FactoryReset(bool keepNetworkSettings)
{
  this->xwrapper_->XCallMain("factoryReset", keepNetworkSettings);
}

void
ifm3d::O3RCamera::Impl::Reboot()
{
  this->xwrapper_->XCallMain("reboot");
}

json
ifm3d::O3RCamera::Impl::Schema()
{
  std::string schema =
    xmlrpc_c::value_string(this->xwrapper_->XCallMain("getSchema")).cvalue();
  return json::parse(schema);
}

#endif // IFM3D_CAMERA_CAMERA_O3R_IMPL_HPP