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

    std::string Get(const std::vector<std::string>& path);
    void Set(const std::string& config);
    std::string GetInit();
    void SaveInit();
    std::string GetInitStatus();
    std::string GetSchema();
    void Lock(const std::string& password);
    void Unlock(const std::string& password);

    void FactoryReset(bool keepNetworkSettings);
    void Reboot();

  protected:
    std::shared_ptr<XMLRPCWrapper> xwrapper_;
  }; // end: class O3RCamera::Impl

} // end: namespace ifm3d

ifm3d::O3RCamera::Impl::Impl(std::shared_ptr<XMLRPCWrapper> xwrapper)
  : xwrapper_(std::move(xwrapper))
{}

ifm3d::O3RCamera::Impl::~Impl() {}

std::string
ifm3d::O3RCamera::Impl::Get(const std::vector<std::string>& path)
{
  return xmlrpc_c::value_string(this->xwrapper_->XCallMain("get", path))
    .cvalue();
}

void
ifm3d::O3RCamera::Impl::Set(const std::string& config)
{
  this->xwrapper_->XCallMain("set", config);
}

std::string
ifm3d::O3RCamera::Impl::GetInit()
{
  return xmlrpc_c::value_string(
           this->xwrapper_->XCallMain("getInit", std::vector<std::string>()))
    .cvalue();
}

void
ifm3d::O3RCamera::Impl::SaveInit()
{
  this->xwrapper_->XCallMain("saveInit");
}

std::string
ifm3d::O3RCamera::Impl::GetInitStatus()
{
  return xmlrpc_c::value_string(
           this->xwrapper_->XCallMain("getInitStatus",
                                      std::vector<std::string>()))
    .cvalue();
}

std::string
ifm3d::O3RCamera::Impl::GetSchema()
{
  return xmlrpc_c::value_string(this->xwrapper_->XCallMain("getSchema"))
    .cvalue();
}

void
ifm3d::O3RCamera::Impl::Lock(const std::string& password)
{
  this->xwrapper_->XCallMain("lock", password);
}

void
ifm3d::O3RCamera::Impl::Unlock(const std::string& password)
{
  this->xwrapper_->XCallMain("unlock", password);
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

#endif // IFM3D_CAMERA_CAMERA_O3R_IMPL_HPP