/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <glog/logging.h>
#include <ifm3d/tools.h>
#include <ifm3d/camera.h>

std::unordered_map<
  std::string,
  std::function<ifm3d::CmdLineApp::Ptr(int, const char**, const std::string&)>>
  app_factory = {
    {"app-types",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::AppTypesApp>(argc, argv, cmd);
     }},

    {"config",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::ConfigApp>(argc, argv, cmd);
     }},

    {"cp",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::CpApp>(argc, argv, cmd);
     }},

    {"discover",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::DiscoverApp>(argc, argv, cmd);
     }},

    {"dump",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::DumpApp>(argc, argv, cmd);
     }},

    {"export",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::ExportApp>(argc, argv, cmd);
     }},

    {"imager-types",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::ImagerTypesApp>(argc, argv, cmd);
     }},

    {"import",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::ImportApp>(argc, argv, cmd);
     }},

    {"jsonschema",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::JsonSchemaApp>(argc, argv, cmd);
     }},

    {"ls",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::LsApp>(argc, argv, cmd);
     }},

    {"passwd",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::PasswdApp>(argc, argv, cmd);
     }},

    {"reboot",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::RebootApp>(argc, argv, cmd);
     }},

    {"reset",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::ResetApp>(argc, argv, cmd);
     }},

    {"rm",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::RmApp>(argc, argv, cmd);
     }},

    {"time",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::TimeApp>(argc, argv, cmd);
     }},

    {"trace",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::TraceApp>(argc, argv, cmd);
     }},

#if defined(BUILD_MODULE_FRAMEGRABBER)
    {"schema",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::SchemaApp>(argc, argv, cmd);
     }},

    {"hz",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::HzApp>(argc, argv, cmd);
     }},

    {"jitter",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::JitterApp>(argc, argv, cmd);
     }},
#endif

#if defined(BUILD_MODULE_SWUPDATER)
    {"swupdate",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::SWUpdateApp>(argc, argv, cmd);
     }},
#endif

    {"version",
     [](int argc, const char** argv, const std::string& cmd)
       -> ifm3d::CmdLineApp::Ptr {
       return std::make_shared<ifm3d::CmdLineApp>(argc, argv, cmd);
     }}};

ifm3d::CmdLineApp::Ptr
ifm3d::make_app(int argc, const char** argv)
{
  cxxopts::Options options("ifm3d");
  // clang-format off
  options.add_options("version")
    ("command", "ifm3d Sub-command to execute",
     cxxopts::value<std::string>()->default_value("version"));
  // clang-format on
  auto vm = [&options](int argc, const char** argv) -> cxxopts::ParseResult {
    options.allow_unrecognised_options();
    options.parse_positional("command");

    auto args = std::make_unique<ifm3d::MutableArgs>(argc, argv);
    return options.parse(args->argc, args->argv);
  }(argc, argv);

  std::string cmd = vm["command"].as<std::string>();

  try
    {
      return app_factory.at(cmd)(argc, argv, cmd);
    }
  catch (const std::out_of_range& /*ex*/)
    {
      std::vector<std::string> keys;
      for (auto& c : app_factory)
        {
          keys.push_back(c.first);
        }
      std::sort(keys.begin(), keys.end());

      std::cerr << "Unrecognized command: " << cmd << std::endl;
      std::cerr << "Valid commands are: " << std::endl;
      for (auto& k : keys)
        {
          std::cerr << "- " << k << std::endl;
        }
      std::cerr << std::endl;

      throw ifm3d::error_t(IFM3D_SUBCOMMAND_ERROR);
    }
}
