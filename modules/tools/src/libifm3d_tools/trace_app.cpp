/*
 * Copyright (C) 2018 ifm syntron gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <iostream>
#include <ifm3d/tools/trace_app.h>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera.h>

ifm3d::TraceApp::TraceApp(int argc, const char** argv, const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format off
  this->local_opts_.add_options()
    ("limit,l", po::value<int>(),
     "Limit the amount of trace log messages printed. (default: all)");
  // clang-format on

  po::store(po::command_line_parser(argc, argv)
              .options(this->local_opts_)
              .allow_unregistered()
              .run(),
            this->vm_);
  po::notify(this->vm_);
}

int
ifm3d::TraceApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto limit = 0;
  if (this->vm_.count("limit"))
    {
      limit = std::max(1, this->vm_["limit"].as<int>());
    }

  std::vector<std::string> logs = this->cam_->TraceLogs(limit);

  for (auto& log : logs)
    {
      std::cout << log << std::endl << std::flush;
    }

  return 0;
}
