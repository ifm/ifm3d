/*
 * Copyright (C) 2018 ifm syntron gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <iostream>
#include <ifm3d/tools/common/trace_app.h>
#include <ifm3d/device/device.h>

namespace
{
  constexpr auto DEFAULT_TRACE_LIMIT{100};
}

ifm3d::TraceApp::~TraceApp() {}

void
ifm3d::TraceApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();

  auto limit = DEFAULT_TRACE_LIMIT;
  if (this->limit)
    {
      /*
       * there is a bug in the XML-RPC library which causes a
       * timeout for larger numbers of trace-logs which prevents
       * us from using -1 and 0 to retrieve all trace logs
       */
      limit = std::max(1, this->limit);
    }

  std::vector<std::string> logs = device->TraceLogs(limit);

  for (auto& log : logs)
    {
      std::cout << log << std::endl << std::flush;
    }
}

CLI::App*
ifm3d::TraceApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand(
        "trace",
        "Get trace messages from the internal camera trace buffer.")
      ->require_subcommand(0, 0);

  command->add_option(
    "-l,--limit",
    this->limit,
    fmt::format(
      "Limit the amount of trace log messages printed. (default: {} )",
      DEFAULT_TRACE_LIMIT));

  return command;
}
