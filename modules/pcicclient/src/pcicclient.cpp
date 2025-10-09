// -*- c++ -*-
/*
 * Copyright (C) 2017 Kuhn & Völkel GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cstdint>
#include <functional>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/pcicclient/pcicclient.h>
#include <pcicclient_impl.hpp>
#include <string>
#include <utility>

ifm3d::PCICClient::PCICClient(ifm3d::LegacyDevice::Ptr cam,
                              const std::uint16_t pcic_port)
  : _impl(new ifm3d::PCICClient::Impl(std::move(cam), pcic_port))
{}

ifm3d::PCICClient::~PCICClient() = default;

void
ifm3d::PCICClient::Stop()
{
  this->_impl->Stop();
}

long
ifm3d::PCICClient::Call(
  const std::string& request,
  std::function<void(const std::string& response)> callback)
{
  return this->_impl->Call(request, std::move(callback));
}

std::string
ifm3d::PCICClient::Call(const std::string& request)
{
  return this->_impl->Call(request);
}

bool
ifm3d::PCICClient::Call(const std::string& request,
                        std::string& response,
                        long timeout_millis)
{
  return this->_impl->Call(request, response, timeout_millis);
}

long
ifm3d::PCICClient ::SetErrorCallback(
  std::function<void(const std::string& error)> callback)
{
  return this->_impl->SetErrorCallback(std::move(callback));
}

long
ifm3d::PCICClient ::SetNotificationCallback(
  std::function<void(const std::string& notification)> callback)
{
  return this->_impl->SetNotificationCallback(std::move(callback));
}

void
ifm3d::PCICClient::CancelCallback(long callback_id)
{
  this->_impl->CancelCallback(callback_id);
}
