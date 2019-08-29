// -*- c++ -*-
/*
 * Copyright (C) 2019 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ifm3d/tools/udp_app.h>
#include <cstdint>
#include <iostream>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

namespace po = boost::program_options;

ifm3d::UdpApp::UdpApp(int argc, const char **argv,
                      const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("disable", po::bool_switch()->default_value(false),
     "Disable UDP functionality")
    ("target-ip", po::value<std::string>()->default_value(std::string("")),
     "IP address of the receive endpoint in a unicast set-up")
    ("port,p",
     po::value<std::uint16_t>()->default_value(ifm3d::DEFAULT_UDP_PORT),
     "The port number to which data needs to be sent")
    ("mask",
     po::value<std::uint16_t>()->default_value(ifm3d::DEFAULT_SCHEMA_MASK),
     "The schema mask describing which channels to transmit")
    ("max-payload-size",
     po::value<std::uint16_t>()->default_value(ifm3d::DEFAULT_UDP_PACKET_SZ),
     "The maximum payload size for each UDP packet");

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

int ifm3d::UdpApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto const disable = vm_.count("disable") ?
    vm_["disable"].as<bool>() : false;

  if (disable)
    {
      this->cam_->DisableUdp();
    }
  else
    {
      auto target_ip = this->vm_["target-ip"].as<std::string>();
      auto port = this->vm_["port"].as<std::uint16_t>();
      auto mask = this->vm_["mask"].as<std::uint16_t>();
      auto max_payload_size = this->vm_["max-payload-size"].as<std::uint16_t>();

      this->cam_->EnableUdp(target_ip, port, mask, max_payload_size);
    }

  return 0;
}
