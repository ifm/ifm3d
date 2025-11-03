/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <CLI/Validators.hpp>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <ifm3d/common/err.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/common/util.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/util.h>
#include <ifm3d/tools/ovp8xx/sealedbox.h>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>

#if defined(BUILD_MODULE_CRYPTO)

namespace
{
  const size_t MAX_RETRIES = 3;
}

namespace ifm3d::tools::ovp8xx::sealedbox
{
  void
  SealedBox::Execute(CLI::App* /*app*/)
  {
    Parent<MainCommand>()->GetDevice<ifm3d::O3R>()->SaveInit(this->paths);
  }

  CLI::App*
  SealedBox::CreateCommand(CLI::App* parent)
  {
    CLI::App* command =
      parent
        ->add_subcommand("sealedbox",
                         "sealedbox is used to for accessing the password "
                         "protected features of the device.")
        ->require_subcommand(1);

    RegisterSubcommand<GetPublicKey>(command);
    RegisterSubcommand<IsPasswordProtected>(command);
    RegisterSubcommand<RemovePassword>(command);
    RegisterSubcommand<Set>(command);
    RegisterSubcommand<SetPassword>(command);

    return command;
  }

  void
  GetPublicKey::Execute(CLI::App* /*app*/)
  {
    auto device = Parent<MainCommand>()->GetDevice<ifm3d::O3R>();
    auto public_key = device->SealedBox()->GetPublicKey();

    auto output_format = this->output_format.value_or(
      IFM3D_IS_A_TTY(stdout) ? "base64" : "binary");

    if (output_format == "binary")
      {
        std::cout.write(reinterpret_cast<const char*>(public_key.data()),
                        static_cast<std::streamsize>(public_key.size()));
      }
    else if (output_format == "base64")
      {
        std::cout << ifm3d::base64_encode(public_key) << "\n";
      }
  }

  CLI::App*
  GetPublicKey::CreateCommand(CLI::App* parent)
  {
    CLI::App* command = parent->add_subcommand(
      "getPublicKey",
      "Get the public key of the device used for sending encrypted messages.");

    command
      ->add_option("--output-format",
                   this->output_format,
                   "Output format of the public key.")
      ->transform(CLI::IsMember({"binary", "base64"}, CLI::ignore_case))
      ->type_name("")
      ->option_text("[binary|base64]")
      ->required(false);

    return command;
  }

  void
  IsPasswordProtected::Execute(CLI::App* /*app*/)
  {
    auto device = Parent<MainCommand>()->GetDevice<ifm3d::O3R>();
    auto is_password_protected = device->SealedBox()->IsPasswordProtected();

    std::cout << (is_password_protected ? "True" : "False") << "\n";
  }

  CLI::App*
  IsPasswordProtected::CreateCommand(CLI::App* parent)
  {
    CLI::App* command = parent->add_subcommand(
      "isPasswordProtected",
      "Returns whether the device is password protected or not");

    return command;
  }

  void
  RemovePassword::Execute(CLI::App* /*app*/)
  {
    auto device = Parent<MainCommand>()->GetDevice<ifm3d::O3R>();

    if (!device->SealedBox()->IsPasswordProtected())
      {
        std::cout << "Device is not password protected.\n";
        return;
      }

    auto password = Parent<MainCommand>()->password;

    if (password.empty())
      {
        std::cout << "Password: " << std::flush;
        password = ifm3d::read_password();
      }

    device->SealedBox()->RemovePassword(password);
  }

  CLI::App*
  RemovePassword::CreateCommand(CLI::App* parent)
  {
    CLI::App* command = parent->add_subcommand(
      "removePassword",
      "Removes the password protection from the device.");

    return command;
  }

  void
  Set::Execute(CLI::App* /*app*/)
  {
    auto device = Parent<MainCommand>()->GetDevice<ifm3d::O3R>();

    auto password = Parent<MainCommand>()->password;

    if (password.empty())
      {
        std::cout << "Password: " << std::flush;
        password = ifm3d::read_password();
      }

    std::string jstr;
    const std::string infile = this->config_file;

    if (infile == "-")
      {
        std::ostringstream buff;

        if (ifm3d::is_stdin_available())
          {
            std::string line;
            while (std::getline(std::cin, line))
              {
                buff << line << "\n";
              }
          }
        else
          {
            throw ifm3d::Error(IFM3D_NO_INPUT_PROVIDED);
          }

        jstr.assign(buff.str());
      }
    else
      {
        std::ifstream ifs(infile, std::ios::in);
        if (!ifs)
          {
            std::cerr << "Could not parse file: " << infile << "\n";
            throw ifm3d::Error(IFM3D_IO_ERROR);
          }

        jstr.assign((std::istreambuf_iterator<char>(ifs)),
                    (std::istreambuf_iterator<char>()));
      }

    device->SealedBox()->Set(password, ifm3d::json::parse(jstr));

    if (this->save)
      {
        std::static_pointer_cast<ifm3d::O3R>(device)->SaveInit();
      }
  }

  CLI::App*
  Set::CreateCommand(CLI::App* parent)
  {
    CLI::App* command =
      parent
        ->add_subcommand("set",
                         "Sets the configuration of the device. This allows "
                         "setting password protected parameters.")
        ->require_subcommand(0, 0);
    command
      ->add_option("--file",
                   this->config_file,
                   "Input JSON configuration file (defaults to stdin)")
      ->option_text("TEXT")
      ->default_str("-");

    command->add_flag("--save",
                      this->save,
                      "Save the new configuration as initial JSON");

    return command;
  }

  void
  SetPassword::Execute(CLI::App* /*app*/)
  {
    auto device = Parent<MainCommand>()->GetDevice<O3R>();

    auto new_password = this->new_password;
    if (new_password.empty())
      {
        for (size_t i = 0; i < MAX_RETRIES; ++i)
          {
            std::cout << "New Password: " << std::flush;
            new_password = ifm3d::read_password();

            std::cout << "Repeat Password: " << std::flush;
            auto password_repeat = ifm3d::read_password();

            if (new_password == password_repeat)
              {
                break;
              }

            new_password.clear();
            std::cout << "Passwords do not match. Try again...\n";
          }
      }

    if (new_password.empty())
      {
        std::cout << "Unable to get new password, aborting.\n";
        return;
      }

    if (!device->SealedBox()->IsPasswordProtected())
      {
        device->SealedBox()->SetPassword(new_password);
        return;
      }

    auto password = Parent<MainCommand>()->password;
    if (!password.empty())
      {
        device->SealedBox()->SetPassword(new_password, password);
        return;
      }

    for (size_t i = 0; i < MAX_RETRIES; ++i)
      {
        std::cout << "Old Password: " << std::flush;
        password = ifm3d::read_password();

        try
          {
            device->SealedBox()->SetPassword(new_password, password);
            break;
          }
        catch (const ifm3d::Error& e)
          {
            if (strstr(e.message(), "Password is missing or incorrect") !=
                  nullptr &&
                i < MAX_RETRIES - 1)
              {
                std::cout << "Incorrect password. Try again...\n";
                continue;
              }
            throw;
          }
      }
  }

  CLI::App*
  SetPassword::CreateCommand(CLI::App* parent)
  {
    CLI::App* command = parent->add_subcommand(
      "setPassword",
      "Save the current configuration as initial configuration");

    command
      ->add_option("--new-password",
                   this->new_password,
                   "The new password for the device")
      ->required(false);

    return command;
  }
}

#endif