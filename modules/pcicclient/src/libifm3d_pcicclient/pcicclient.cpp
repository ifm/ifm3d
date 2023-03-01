// -*- c++ -*-
/*
 * Copyright (C) 2017 Kuhn & Völkel GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#include <ifm3d/pcicclient/pcicclient.h>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <ifm3d/device/err.h>
#include <pcicclient_impl.hpp>

ifm3d::PCICClient::PCICClient(ifm3d::LegacyDevice::Ptr cam,
                              const std::uint16_t pcic_port)
  : pImpl(new ifm3d::PCICClient::Impl(cam, pcic_port))
{}

ifm3d::PCICClient::~PCICClient() = default;

void
ifm3d::PCICClient::Stop()
{
  return this->pImpl->Stop();
}

long
ifm3d::PCICClient::Call(
  const std::string& request,
  std::function<void(const std::string& response)> callback)
{
  return this->pImpl->Call(request, callback);
}

std::string
ifm3d::PCICClient::Call(const std::string& request)
{
  return this->pImpl->Call(request);
}

bool
ifm3d::PCICClient::Call(const std::string& request,
                        std::string& response,
                        long timeout_millis)
{
  return this->pImpl->Call(request, response, timeout_millis);
}

long
ifm3d::PCICClient ::SetErrorCallback(
  std::function<void(const std::string& error)> callback)
{
  return this->pImpl->SetErrorCallback(callback);
}

long
ifm3d::PCICClient ::SetNotificationCallback(
  std::function<void(const std::string& notification)> callback)
{
  return this->pImpl->SetNotificationCallback(callback);
}

void
ifm3d::PCICClient::CancelCallback(long callback_id)
{
  return this->pImpl->CancelCallback(callback_id);
}
