/*
 * Copyright 2020-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/fg/fg_app.h>
#include <string>

const int ifm3d::FG_TIMEOUT = 10000;

ifm3d::FgApp::FgApp(int argc, const char** argv, const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options("global")(
    "pcic-port",
    "port number for pcic communication",
    cxxopts::value<unsigned short>()->default_value(std::to_string(ifm3d::DEFAULT_NAT_PCIC_PORT)));
  // clang-format on
  this->_Parse(argc, argv);

  this->pcic_port_ = (*this->vm_)["pcic-port"].as<unsigned short>();

  if (!(this->vm_->count("help")))
    {
      this->fg_ =
        std::make_shared<ifm3d::FrameGrabber>(this->cam_,
                                              ifm3d::DEFAULT_SCHEMA_MASK,
                                              this->pcic_port_);
    }
}
