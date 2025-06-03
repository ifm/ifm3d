// -*- c++ -*-
/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_SEALEDBOX_APP_H
#define IFM3D_TOOLS_SEALEDBOX_APP_H

#ifdef BUILD_MODULE_CRYPTO

#  include <string>
#  include <ifm3d/tools/command.hpp>
#  include <ifm3d/tools/main_command.hpp>

namespace ifm3d::tools::ovp8xx::sealedbox
{
  class SealedBox : public Command
  {
  public:
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::vector<std::string> paths = {};
  };

  class Set : public Command
  {
  public:
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string password = {};
    std::string config_file{"-"};
    bool save;
  };

  class RemovePassword : public Command
  {
  public:
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string password = {};
  };

  class GetPublicKey : public Command
  {
  public:
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::optional<std::string> output_format = std::nullopt;
  };

  class IsPasswordProtected : public Command
  {
  public:
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;
  };

  class SetPassword : public Command
  {
  public:
    virtual void Execute(CLI::App* app) override;
    virtual CLI::App* CreateCommand(CLI::App* parent) override;

    std::string new_password = {};
    std::optional<std::string> old_password = std::nullopt;
  };
}

#endif // BUILD_MODULE_CRYPTO

#endif // IFM3D_TOOLS_SEALEDBOX_APP_H
