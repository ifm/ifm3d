/*
 * Copyright (C) 2017 Love Park Robotics, LLC
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

#include <ifm3d/camera/camera.h>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <glog/logging.h>
#include <xmlrpc-c/client.hpp>
#include <ifm3d/camera/version.h>
#include <ifm3d/camera/logging.h>

//------------------------------------------------
// Public constants
//------------------------------------------------
const std::string ifm3d::DEFAULT_PASSWORD = "";
const std::uint16_t ifm3d::DEFAULT_XMLRPC_PORT = 80;
const std::string ifm3d::DEFAULT_IP =
  std::getenv("IFM3D_IP") == nullptr ?
  "192.168.0.69" : std::string(std::getenv("IFM3D_IP"));

//------------------------------------------------
// Private constants
//------------------------------------------------
namespace ifm3d
{
  const int NET_WAIT = 3000; // millis

  const std::string XMLRPC_MAIN = "/api/rpc/v1/com.ifm.efector/";
  const std::string XMLRPC_SESSION = "session_XXX/";
  const std::string XMLRPC_EDIT = "edit/";
  const std::string XMLRPC_DEVICE = "device/";
  const std::string XMLRPC_NET = "network/";
  const std::string XMLRPC_APP = "application/";
  const std::string XMLRPC_IMAGER = "imager_001/";
  const std::string XMLRPC_SPATIALFILTER = "spatialfilter";
  const std::string XMLRPC_TEMPORALFILTER = "temporalfilter";
} // end: namespace ifm3d

//------------------------------------------------
// Camera::Impl class - the private implementation
//------------------------------------------------

class ifm3d::Camera::Impl
{
private:
  std::string ip_;
  std::uint16_t xmlrpc_port_;
  std::string password_;
  std::string xmlrpc_url_prefix_;
  xmlrpc_c::clientPtr xclient_;

public:
  Impl(const std::string& ip,
       const std::uint16_t xmlrpc_port,
       const std::string& password)
    : ip_(ip),
      xmlrpc_port_(xmlrpc_port),
      password_(password),
      xmlrpc_url_prefix_("http://" + ip + ":" + std::to_string(xmlrpc_port)),
      xclient_(new xmlrpc_c::client_xml(
                 xmlrpc_c::clientXmlTransportPtr(
                   new xmlrpc_c::clientXmlTransport_curl(
                     xmlrpc_c::clientXmlTransport_curl::constrOpt().
                     timeout(ifm3d::NET_WAIT)))))
  {
    VLOG(IFM3D_TRACE) << "Initializing Camera: ip="
                      << this->IP()
                      << ", xmlrpc_port=" << this->XMLRPCPort()
                      << ", password=" << this->Password();
    VLOG(IFM3D_TRACE) << "XMLRPC URL Prefix=" << this->xmlrpc_url_prefix_;
  }

  ~Impl()
  {
    VLOG(IFM3D_TRACE) << "Dtor...";
  }

  std::string IP()
  {
    return this->ip_;
  }

  std::uint16_t XMLRPCPort()
  {
    return this->xmlrpc_port_;
  }

  std::string Password()
  {
    return this->password_;
  }

}; // end: class ifm3d::Camera::Impl

//------------------------------------------------
// Camera class - the public interface
//------------------------------------------------

ifm3d::Camera::Camera(const std::string& ip,
                      const std::uint16_t xmlrpc_port,
                      const std::string& password)
  : pImpl(new ifm3d::Camera::Impl(ip, xmlrpc_port, password))
{ }

ifm3d::Camera::~Camera() = default;

std::string
ifm3d::Camera::IP()
{
  return this->pImpl->IP();
}

std::uint16_t
ifm3d::Camera::XMLRPCPort()
{
  return this->pImpl->XMLRPCPort();
}

std::string
ifm3d::Camera::Password()
{
  return this->pImpl->Password();
}

json
ifm3d::Camera::ToJSON()
{
  json j =
    {
      {
       "ifm3d",
       {
         {std::string(IFM3D_LIBRARY_NAME) + "_version", IFM3D_VERSION}
       }
      }
    };

  return j;
}
