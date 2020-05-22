/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ifm3d/tools/cmdline_app.h>
#include <cstdint>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <ifm3d/camera.h>

namespace po = boost::program_options;

ifm3d::CmdLineApp::CmdLineApp(int argc, const char **argv,
                              const std::string& name)
  : global_opts_("global options"),
    local_opts_(name + " options")
{
  this->global_opts_.add_options()
    ("help,h", "Produce this help message and exit")
    ("ip", po::value<std::string>()->default_value(ifm3d::DEFAULT_IP),
     "IP address of the sensor")
    ("xmlrpc-port",
     po::value<std::uint16_t>()->default_value(ifm3d::DEFAULT_XMLRPC_PORT),
     "XMLRPC port of the sensor")
    ("password",
     po::value<std::string>()->default_value(ifm3d::DEFAULT_PASSWORD),
     "Password for establishing an edit-session with the sensor");

  po::options_description hidden_opts;
  hidden_opts.add_options()
    ("command", po::value<std::string>()->default_value(name),
     "ifm3d Sub-command to execute");

  po::options_description all_opts;
  all_opts.add(this->global_opts_).add(hidden_opts);

  po::positional_options_description p;
  p.add("command", 1);

  po::store(po::command_line_parser(argc, argv).
            options(all_opts).positional(p).
            allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);

  this->ip_ = this->vm_["ip"].as<std::string>();
  this->xmlrpc_port_ = this->vm_["xmlrpc-port"].as<std::uint16_t>();
  this->password_ = this->vm_["password"].as<std::string>();

  // slight optimization -- if it is a `help' or `version' request
  // no need to ping the h/w ... which is slow when no device is present.
  if ((!(this->vm_.count("help"))) && (name != "version"))
    {
      this->cam_ = ifm3d::Camera::MakeShared(this->ip_,
                                             this->xmlrpc_port_,
                                             this->password_);
    }
}

void
ifm3d::CmdLineApp::_LocalHelp()
{
  std::string cmd = this->vm_["command"].as<std::string>();
  std::cout << "usage: " << IFM3D_LIBRARY_NAME
            << " [<global options>] "
            << cmd
            << " [<" << cmd << " options>]"
            << std::endl << std::endl;
  std::cout << this->global_opts_ << std::endl;
  std::cout << this->local_opts_ << std::endl;
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
  std::cout << IFM3D_LIBRARY_NAME
            << ": version=" << major << "."
            << minor << "." << patch << std::endl;

  if (this->vm_.count("help"))
    {
      std::cout << "usage: " << IFM3D_LIBRARY_NAME
                << " [<global options>] <command> [<args>]"
                << std::endl << std::endl;
      std::cout << this->global_opts_ << std::endl;
      std::cout << help_msg << std::endl;
    }

  return 0;
}
