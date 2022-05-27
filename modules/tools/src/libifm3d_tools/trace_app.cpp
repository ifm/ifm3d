/*
 * Copyright (C) 2018 ifm syntron gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <iostream>
#include <ifm3d/tools/trace_app.h>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device/device.h>
#include <string>

namespace
{
  constexpr auto DEFAULT_TRACE_LIMIT{100};
}

ifm3d::TraceApp::TraceApp(int argc, const char** argv, const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  auto const message =
    "Limit the amount of trace log messages printed. (default: " +
    std::to_string(DEFAULT_TRACE_LIMIT) + ")";
  // clang-format off
  this->all_opts_.add_options(name)
    ("l,limit",
     message,
     cxxopts::value<int>());

  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::TraceApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  auto limit = DEFAULT_TRACE_LIMIT;
  if (this->vm_->count("limit"))
    {
      /*
       * there is a bug in the XML-RPC library which causes a
       * timeout for larger numbers of trace-logs which prevents
       * us from using -1 and 0 to retrieve all trace logs
       */
      limit = std::max(1, (*this->vm_)["limit"].as<int>());
    }

  std::vector<std::string> logs = this->cam_->TraceLogs(limit);

  for (auto& log : logs)
    {
      std::cout << log << std::endl << std::flush;
    }

  return 0;
}
