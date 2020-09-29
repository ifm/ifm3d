/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/fg/fg_app.h>
#include <string>

ifm3d::FgApp::FgApp(int argc, const char** argv, const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
	// clang-format off
  this->all_opts_.add_options("global")(
    "nat-port",
    "port number for pcic communication on NAT router",
    cxxopts::value<unsigned short>()->default_value(std::to_string(ifm3d::DEFAULT_NAT_PCIC_PORT)));
  // clang-format on
  this->_Parse(argc, argv);
 
  this->nat_port_ = (*this->vm_)["nat-port"].as<unsigned short>();
  std::cout << this->nat_port_;
  this->fg_ = std::make_shared<ifm3d::FrameGrabber>(
    this->cam_,
    ifm3d::DEFAULT_SCHEMA_MASK,this->nat_port_);
}
