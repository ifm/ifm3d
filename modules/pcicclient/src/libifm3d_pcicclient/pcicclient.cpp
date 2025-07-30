// -*- c++ -*-
/*
 * Copyright (C) 2017 Kuhn & Völkel GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ifm3d/device/legacy_device.h"
#include <cstdint>
#include <ifm3d/pcicclient/pcicclient.h>
#include <functional>
#include <string>
#include <pcicclient_impl.hpp>
#include <utility>

ifm3d::PCICClient::PCICClient(ifm3d::LegacyDevice::Ptr cam,
                              const std::uint16_t pcic_port)
  : pImpl(new ifm3d::PCICClient::Impl(std::move(cam), pcic_port))
{}

ifm3d::PCICClient::~PCICClient() = default;

void
ifm3d::PCICClient::Stop()
{
  this->pImpl->Stop();
}

long
ifm3d::PCICClient::Call(
  const std::string& request,
  std::function<void(const std::string& response)> callback)
{
  return this->pImpl->Call(request, std::move(callback));
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
  return this->pImpl->SetErrorCallback(std::move(callback));
}

long
ifm3d::PCICClient ::SetNotificationCallback(
  std::function<void(const std::string& notification)> callback)
{
  return this->pImpl->SetNotificationCallback(std::move(callback));
}

void
ifm3d::PCICClient::CancelCallback(long callback_id)
{
  this->pImpl->CancelCallback(callback_id);
}
