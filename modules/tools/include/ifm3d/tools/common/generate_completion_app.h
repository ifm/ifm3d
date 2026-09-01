/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_GENERATE_COMPLETION_APP_H
#define IFM3D_TOOLS_GENERATE_COMPLETION_APP_H

#include <ifm3d/tools/command.hpp>
#include <string>

namespace ifm3d
{
  class GenerateCompletionApp : public Command
  {
  public:
    GenerateCompletionApp() = default;
    GenerateCompletionApp(const GenerateCompletionApp&) = default;
    GenerateCompletionApp(GenerateCompletionApp&&) = delete;
    GenerateCompletionApp& operator=(const GenerateCompletionApp&) = default;
    GenerateCompletionApp& operator=(GenerateCompletionApp&&) = delete;
    ~GenerateCompletionApp() override = default;

    CLI::App* CreateCommand(CLI::App* parent) override;
    void Execute(CLI::App* app) override;

  private:
    std::string shell;
  };
} // end: namespace ifm3d

#endif // IFM3D_TOOLS_GENERATE_COMPLETION_APP_H
