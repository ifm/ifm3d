/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/ovp8xx/get_service_report_app.h>
#include <iostream>
#include <string>
#include <ifm3d/device.h>

ifm3d::GetServiceReportApp::~GetServiceReportApp() {}

void
ifm3d::GetServiceReportApp::Execute(CLI::App* app)
{
  auto device = Parent<MainCommand>()->GetDevice();
  std::static_pointer_cast<ifm3d::O3R>(device)->DownloadServiceReport();
}

CLI::App*
ifm3d::GetServiceReportApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent
      ->add_subcommand("getServiceReport",
                       "Get the service report of O3R VPU (only available for "
                       "firmware version 1.4.x and above)")
      ->require_subcommand(0, 0);

  return command;
}
