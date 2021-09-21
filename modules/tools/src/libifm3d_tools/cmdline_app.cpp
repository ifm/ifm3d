/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/tools/mutable_args.h>
#include <cstdint>
#include <iostream>
#include <string>
#include <memory>
#include <ifm3d/camera.h>

ifm3d::CmdLineApp::CmdLineApp(int argc,
                              const char** argv,
                              const std::string& name)
  : all_opts_("ifm3d")
{
  // clang-format off
  this->all_opts_.add_options("global")
    ("h,help", "Produce this help message and exit")
    ("ip","IP address of the sensor",
     cxxopts::value<std::string>()->default_value(ifm3d::DEFAULT_IP))
    ("xmlrpc-port","XMLRPC port of the sensor",
     cxxopts::value<std::uint16_t>()->default_value(std::to_string(ifm3d::DEFAULT_XMLRPC_PORT)))
    ("password","Password for establishing an edit-session with the sensor",
     cxxopts::value<std::string>()->default_value(ifm3d::DEFAULT_PASSWORD));

  this->all_opts_.add_options("Hidden")("command",
    "ifm3d Sub-command to execute", cxxopts::value<std::string>()->default_value(name));

  // clang-format on
  this->_Parse(argc, argv);

  this->ip_ = (*this->vm_)["ip"].as<std::string>();
  this->xmlrpc_port_ = (*this->vm_)["xmlrpc-port"].as<uint16_t>();
  this->password_ = (*this->vm_)["password"].as<std::string>();

  // slight optimization -- if it is a `help' or `version' request
  // no need to ping the h/w ... which is slow when no device is present.
  if ((!(this->vm_->count("help"))) && (name != "version"))
    {
      this->cam_ = ifm3d::CameraBase::MakeShared(this->ip_,
                                                 this->xmlrpc_port_,
                                                 this->password_);
    }
}

void
ifm3d::CmdLineApp::_Parse(int argc, const char** argv)
{
  auto args = std::make_unique<ifm3d::MutableArgs>(argc, argv);
  this->all_opts_.allow_unrecognised_options();
  vm_ = std::make_unique<cxxopts::ParseResult>(
    this->all_opts_.parse(args->argc, args->argv));
}

void
ifm3d::CmdLineApp::_LocalHelp()
{
  std::string cmd = (*this->vm_)["command"].as<std::string>();
  this->all_opts_.custom_help("[<global options>] " + cmd + " [<" + cmd +
                              " options>]");
  std::cout << this->all_opts_.help({"global", cmd}) << std::endl;
}

int
ifm3d::CmdLineApp::Run()
{
  int major, minor, patch;
  std::string help_msg =
    R"(
These are common commands used in various situations:

    app-types     List the application types supported by the sensor.

    config        Configure sensor settings from a JSON description of
                  the desired sensor state. See also `dump'.

    cp            Create a new application on the sensor,
                  bootstrapped from a copy of an existing one.

    discover      Discover ifm devices on the network.

    dump          Serialize the sensor state to JSON.

    export        Export an application or whole sensor configuration
                  into a format compatible with ifm Vision Assistant.
      )";

#if defined(BUILD_MODULE_FRAMEGRABBER)
  help_msg +=
    R"(
    hz            Compute the actual frequency at which the FrameGrabber
                  is running.
      )";
#endif

  help_msg +=
    R"(
    imager-types  List the imager types supported by the sensor.

    import        Import an application or whole sensor configuration
                  that is compatible with ifm Vision Assistant's export
                  format.
      )";

#if defined(BUILD_MODULE_FRAMEGRABBER)
  help_msg +=
    R"(
    jitter        Collects statistics on framegrabber (and optionally, image
                  construction) jitter.
      )";
#endif

  help_msg +=
    R"(
    jsonschema    Gets current JSON schema configuration.
    
    ls            Lists the applications currently installed on
                  the sensor.

    passwd        Sets the password on the sensor.

    reboot        Reboot the sensor, potentially into recovery
                  mode. Recovery mode is useful for putting the
                  sensor into a state where it can be flashed
                  with new firmware.

    reset         Reset the sensor to factory defaults.

    rm            Deletes an application from the sensor.
      )";

#if defined(BUILD_MODULE_FRAMEGRABBER)
  help_msg +=
    R"(
    schema        Construct and analyze image acquisition schema masks.
      )";
#endif

  help_msg +=
    R"(
    swupdate      Perform a firmware update on the camera. Please ensure
                  that the camera is booted to recovery beforehand.

    time          Get/set the current time on the camera.

    trace         Get trace messages from the internal camera trace buffer.

For bug reports, please see:
https://github.com/ifm/ifm3d/issues
      )";

  ifm3d::version(&major, &minor, &patch);
  std::cout << IFM3D_LIBRARY_NAME << ": version=" << major << "." << minor
            << "." << patch << IFM3D_VERSION_META << std::endl;

  if (this->vm_->count("help"))
    {
      this->all_opts_.custom_help("[<global options>] <command> [<args>]");
      std::cout << std::endl;
      std::cout << this->all_opts_.help({"global"});
      std::cout << help_msg << std::endl;
    }

  return 0;
}

bool
ifm3d::CmdLineApp::CheckCompatibility()
{
  return true;
}