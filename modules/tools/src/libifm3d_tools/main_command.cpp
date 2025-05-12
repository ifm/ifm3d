/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <CLI/Validators.hpp>
#include <functional>
#include "ifm3d/common/logging/log_level.h"
#include "ifm3d/common/logging/log_formatter_text.h"
#include "ifm3d/tools/legacy/o3d3xx_app.h"
#include "ifm3d/tools/ovp8xx/ovp8xx_app.h"
#include "ifm3d/tools/common/discover_app.h"
#include "ifm3d/device/version.h"
#include "fmt/core.h"
#include <ifm3d/tools/main_command.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/common/features.h>
#include <ifm3d/common/logging/logger.h>
#include <ifm3d/common/logging/log_writer_file.h>
#include <memory>

ifm3d::MainCommand::MainCommand()
  : ip{ifm3d::DEFAULT_IP},
    xmlrpc_port{ifm3d::DEFAULT_XMLRPC_PORT},
    password{ifm3d::DEFAULT_PASSWORD}
{}

CLI::App*
ifm3d::MainCommand::CreateCommand(CLI::App* parent)
{
  parent->name(IFM3D_LIBRARY_NAME)
    ->description("ifm3d Command Line Interface (CLI)");

  parent->require_subcommand(1);
  parent->fallthrough();

  parent->set_version_flag("-v,--version",
                           GetAppVersion(),
                           "Print version information");

  parent->add_option("--ip", this->ip, "IP address of the sensor")
    ->default_val(ifm3d::DEFAULT_IP)
    ->check(CLI::ValidIPV4);

  parent
    ->add_option("--xmlrpc-port",
                 this->xmlrpc_port,
                 "XMLRPC port of the sensor")
    ->default_val(ifm3d::DEFAULT_XMLRPC_PORT);

  parent
    ->add_option("--password",
                 this->password,
                 "Password for establishing an edit-session with the sensor")
    ->default_val(ifm3d::DEFAULT_PASSWORD);

  std::function<void(const std::string&)> const log_level_cb =
    [&](const std::string& value) {
      ifm3d::Logger::Get().SetLogLevel(
        ifm3d::LogLevelFromString(value.c_str()));
    };

  parent
    ->add_option_function("--log-level",
                          log_level_cb,
                          "The minimum severity of messages that will be "
                          "logged, possible values are in descending order")
    ->check(CLI::IsMember(
      {"CRITICAL", "ERROR", "WARN", "INFO", "DEBUG", "VERBOSE", "NONE"},
      CLI::ignore_case))
    ->default_val("WARN");

  std::function<void(const std::string&)> const log_file_cb =
    [&](const std::string& value) {
      ifm3d::Logger::Get().SetWriter(
        std::make_shared<ifm3d::LogWriterFile<ifm3d::LogFormatterText>>(
          value));
    };

  parent
    ->add_option_function("--log-file",
                          log_file_cb,
                          "Log to a file instead of stderr")
    ->option_text("TEXT");

  RegisterSubcommand<ifm3d::O3X1XX_O3X2XX>(parent)->SetDetails(
    "o3x1xx",
    "Interact with the O3X1xx devices");

  // Note: O3X2xx device command will be available in future, once the device
  // O3X2xx is ready to use.
  /*RegisterSubcommand<ifm3d::O3X1XX_O3X2XX>(parent)
    ->SetDetails("o3x2xx", "Interact with the O3X2xx devices");*/

  RegisterSubcommand<ifm3d::O3D3XX>(parent);

  RegisterSubcommand<ifm3d::OVP8xx>(parent);

  std::string const global_command_deprecation_message =
    "The global commands have been deprecated, please use the device specific "
    "commands instead.";

  RegisterSubcommand<ifm3d::AppTypesApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::ConfigSetApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::CpApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::DiagnosticApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::DiscoverApp>(parent);
  RegisterSubcommand<ifm3d::DumpApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::ExportApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
#if defined(BUILD_MODULE_FRAMEGRABBER)
  RegisterSubcommand<ifm3d::HzApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
#endif
  RegisterSubcommand<ifm3d::ImagerApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::ImportApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
#if defined(BUILD_MODULE_FRAMEGRABBER)
  RegisterSubcommand<ifm3d::JitterApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
#endif
  RegisterSubcommand<ifm3d::JSONSchemaApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::LsApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::PasswordApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::RebootApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::ResetApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::RmApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
#if defined(BUILD_MODULE_SWUPDATER)
  RegisterSubcommand<ifm3d::SWUpdateDeprecatedApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
#endif
  RegisterSubcommand<ifm3d::TimeApp>(parent)->SetDeprecated(
    global_command_deprecation_message);
  RegisterSubcommand<ifm3d::TraceApp>(parent)->SetDeprecated(
    global_command_deprecation_message);

  parent->failure_message(CLI::FailureMessage::help);

  return parent;
}

std::string
ifm3d::MainCommand::GetAppVersion()
{
  int major = 0;
  int minor = 0;
  int patch = 0;
  ifm3d::version(&major, &minor, &patch);
  return std::string(fmt::format("{}: version={}.{}.{}{}{}",
                                 IFM3D_LIBRARY_NAME,
                                 major,
                                 minor,
                                 patch,
                                 IFM3D_VERSION_TWEAK,
                                 IFM3D_VERSION_META));
}

ifm3d::Device::Ptr
ifm3d::MainCommand::GetDevice(bool throw_if_unavailable) const
{
  return ifm3d::Device::MakeShared(this->ip,
                                   this->xmlrpc_port,
                                   this->password,
                                   throw_if_unavailable);
}
