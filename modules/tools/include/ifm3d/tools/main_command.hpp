/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_COMMANDS_MAIN_HPP
#define IFM3D_TOOLS_COMMANDS_MAIN_HPP
#pragma once

#include <ifm3d/device.h>
#include <ifm3d/tools/command.hpp>

namespace ifm3d
{
  class MainCommand : public Command
  {
  public:
    MainCommand();
    ~MainCommand() override = default;
    MainCommand(const MainCommand&) = default;
    MainCommand& operator=(const MainCommand&) = default;
    MainCommand(MainCommand&&) = delete;
    MainCommand& operator=(MainCommand&&) = delete;

    CLI::App* CreateCommand(CLI::App* parent) override;
    std::string GetAppVersion();
    [[nodiscard]] ifm3d::Device::Ptr GetDevice(
      bool throw_if_unavailable = true) const;

    template <typename T>
    typename T::Ptr
    GetDevice(bool throwIfUnavailable = true)
    {
      auto device =
        std::dynamic_pointer_cast<T>(this->GetDevice(throwIfUnavailable));

      if (!device)
        {
          throw ifm3d::Error(IFM3D_TOOL_COMMAND_UNSUPPORTED_DEVICE);
        }

      return device;
    }

    std::string ip;
    std::uint16_t xmlrpc_port;
    std::string password;
  };
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_COMMANDS_MAIN_HPP