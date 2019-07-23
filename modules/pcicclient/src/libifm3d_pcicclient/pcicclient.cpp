// -*- c++ -*-
/*
 * Copyright (C) 2017 Kuhn & Völkel GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ifm3d/pcicclient.h>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <glog/logging.h>
#include <ifm3d/camera/err.h>
#include <pcicclient_impl.hpp>


ifm3d::PCICClient::PCICClient(ifm3d::Camera::Ptr cam): pImpl(new ifm3d::PCICClient::Impl(cam))
{
}

ifm3d::PCICClient::~PCICClient() = default;

void
ifm3d::PCICClient::Stop()
{
	return this->pImpl->Stop();
}

long
ifm3d::PCICClient::Call(const std::string& request,
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
			 std::string& response, long timeout_millis)
{
	return this->pImpl->Call(request, response, timeout_millis);
}

long
ifm3d::PCICClient
::SetErrorCallback(std::function<void(const std::string& error)> callback)
{
	return this->pImpl->SetErrorCallback(callback);
}

long
ifm3d::PCICClient
::SetNotificationCallback(std::function<void(const std::string& notification)> callback)
{
	return this->pImpl->SetNotificationCallback(callback);
}

void
ifm3d::PCICClient::CancelCallback(long callback_id)
{
	return this->pImpl->CancelCallback(callback_id);
}

