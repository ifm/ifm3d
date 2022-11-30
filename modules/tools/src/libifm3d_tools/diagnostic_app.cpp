/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/diagnostic_app.h>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device/o3r.h>

ifm3d::DiagnosticApp::DiagnosticApp(int argc,
                                    const char** argv,
                                    const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("f,filter", "A filter expression in JSON format", cxxopts::value<std::string>());

  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::DiagnosticApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  if (this->vm_->count("filter") == 0)
    {
      std::cout << std::static_pointer_cast<ifm3d::O3R>(this->cam_)
                     ->GetDiagnostic()
                     .dump(2)
                << std::endl;
    }
  else
    {
      std::cout << std::static_pointer_cast<ifm3d::O3R>(this->cam_)
                     ->GetDiagnosticFiltered(
                       json::parse((*this->vm_)["filter"].as<std::string>()))
                     .dump(2)
                << std::endl;
    }

  return 0;
}

bool
ifm3d::DiagnosticApp::CheckCompatibility()
{
  return this->cam_->AmI(Device::device_family::O3R);
}