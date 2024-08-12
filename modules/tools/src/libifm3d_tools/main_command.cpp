/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/main_command.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/common/logging/logger.h>
#include <ifm3d/common/logging/log_writer_file.h>

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

  std::function<void(const std::string&)> log_level_cb =
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

  std::function<void(const std::string&)> log_file_cb =
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

  std::string globalCommandDeprecationMessage =
    "The global commands have been deprecated, please use the device specific "
    "commands instead.";

  RegisterSubcommand<ifm3d::AppTypesApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::ConfigSetApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::CpApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::DiagnosticApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::DiscoverApp>(parent);
  RegisterSubcommand<ifm3d::DumpApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::ExportApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::HzApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::ImagerApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::ImportApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::JitterApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::JSONSchemaApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::LsApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::PasswordApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::RebootApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::ResetApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::RmApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::SWUpdateDeprecatedApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::TimeApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);
  RegisterSubcommand<ifm3d::TraceApp>(parent)->SetDeprecated(
    globalCommandDeprecationMessage);

  parent->failure_message(CLI::FailureMessage::help);

  return parent;
}

std::string
ifm3d::MainCommand::GetAppVersion()
{
  int major, minor, patch;
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
ifm3d::MainCommand::GetDevice(bool throwIfUnavailable)
{
  return ifm3d::Device::MakeShared(this->ip,
                                   this->xmlrpc_port,
                                   this->password,
                                   throwIfUnavailable);
}