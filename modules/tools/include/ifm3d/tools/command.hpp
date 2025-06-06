/*
 * Copyright 2024-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_COMMAND_HPP
#define IFM3D_TOOLS_COMMAND_HPP

#include <CLI/CLI.hpp>
#include <optional>
#include <ifm3d/device/err.h>

namespace ifm3d
{
  class Command
  {

  public:
    virtual ~Command() {}

    virtual CLI::App* CreateCommand(CLI::App* parent) = 0;
    virtual void Execute(CLI::App* command){};

    template <typename T>
    T*
    Parent()
    {
      static_assert(std::is_base_of<Command, T>::value,
                    "Parent must a subclass of Command");

      T* parent = dynamic_cast<T*>(_parent);
      return parent ? parent : _parent ? _parent->Parent<T>() : nullptr;
    }

    template <typename T, typename... Args>
    T*
    RegisterSubcommand(CLI::App* parent, Args... args)
    {
      static_assert(std::is_constructible<T, Args...>::value,
                    "No matching constructor found for subcommand");

      static_assert(std::is_base_of<Command, T>::value,
                    "Subcommand must a subclass of Command");

      auto command = std::make_shared<T>(args...);
      command->_parent = this;

      _subcommands.push_back(command);
      command->_context = command->CreateCommand(parent);

      command->_context->final_callback([command, this]() {
        if (!command->CheckCompatibility())
          {
            throw ifm3d::Error(IFM3D_TOOL_COMMAND_UNSUPPORTED_DEVICE);
          }
        if (command->_deprecated.has_value())
          {
            std::cerr << std::endl
                      << "ifm3d error: Deprecated subcommand: "
                      << command->_context->get_name() << ". "
                      << command->_deprecated.value() << std::endl;
          }
        command->Execute(command->_context);
      });
      return dynamic_cast<T*>(command.get());
    }

    Command*
    SetDeprecated(const std::string& reason)
    {
      this->_deprecated = std::optional<std::string>{reason};
      this->_context->group("");
      return this;
    }

    Command*
    SetDetails(const std::string& name, const std::string& description)
    {
      this->_context->name(name)->description(description);
      return this;
    }

    virtual bool
    CheckCompatibility()
    {
      auto parent = Parent<Command>();
      return parent ? parent->CheckCompatibility() : true;
    }

    CLI::App*
    GetSubcommandApp()
    {
      return this->_context;
    }

  private:
    Command* _parent = nullptr;
    std::vector<std::shared_ptr<Command>> _subcommands;
    std::optional<std::string> _deprecated;
    CLI::App* _context;
  };
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_COMMAND_HPP