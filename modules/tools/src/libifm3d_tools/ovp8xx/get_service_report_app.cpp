/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include "ifm3d/device/o3r.h"
#include <ifm3d/tools/ovp8xx/get_service_report_app.h>
#include <memory>
#include <string>

ifm3d::GetServiceReportApp::~GetServiceReportApp() = default;

void
ifm3d::GetServiceReportApp::Execute(CLI::App* /*app*/)
{
  auto device = Parent<MainCommand>()->GetDevice();
  std::static_pointer_cast<ifm3d::O3R>(device)->DownloadServiceReport(
    this->outfile);
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

  command
    ->add_option("--outfile",
                 this->outfile,
                 "Save the serviceReport into outfile with .zip extension")
    ->default_str("{service_report.zip}");

  return command;
}
