/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <CLI/App.hpp>
#include <CLI/Option.hpp>
#include <CLI/Validators.hpp>
#include <algorithm>
#include <cctype>
#include <cstddef>
#include <ifm3d/tools/common/generate_completion_app.h>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
  using CompletionMap =
    std::unordered_map<std::string, std::vector<std::string>>;
  using DescriptionMap = std::unordered_map<std::string, std::string>;
  constexpr const char* ROOT_CONTEXT = "root";

  void
  push_unique(std::vector<std::string>& values, const std::string& value)
  {
    if (std::find(values.begin(), values.end(), value) == values.end())
      {
        values.push_back(value);
      }
  }

  std::string
  join(const std::vector<std::string>& values, const std::string& separator)
  {
    std::ostringstream joined;
    for (std::size_t i = 0; i < values.size(); ++i)
      {
        if (i > 0)
          {
            joined << separator;
          }
        joined << values[i];
      }
    return joined.str();
  }

  std::string
  normalize_description(const std::string& description)
  {
    std::string normalized;
    normalized.reserve(description.size());
    bool previous_was_space = false;

    for (const char c : description)
      {
        if (c == '\n' || c == '\r' || c == '\t' || c == ' ')
          {
            if (!previous_was_space)
              {
                normalized.push_back(' ');
                previous_was_space = true;
              }
            continue;
          }

        previous_was_space = false;
        normalized.push_back(c);
      }

    const auto first = normalized.find_first_not_of(' ');
    if (first == std::string::npos)
      {
        return "";
      }

    const auto last = normalized.find_last_not_of(' ');
    return normalized.substr(first, last - first + 1);
  }

  std::string
  zsh_describe_escape(const std::string& value)
  {
    std::string escaped;
    escaped.reserve(value.size());

    for (const char c : value)
      {
        if (c == '\\' || c == ':')
          {
            escaped.push_back('\\');
          }
        escaped.push_back(c);
      }

    return escaped;
  }

  std::string
  escape_single_quoted_shell(const std::string& value,
                             const std::string& shell)
  {
    std::string escaped;
    escaped.reserve(value.size());
    const bool is_pwsh = shell == "pwsh";

    for (const char c : value)
      {
        if (c == '\'')
          {
            escaped += is_pwsh ? "''" : "'\"'\"'";
          }
        else
          {
            escaped.push_back(c);
          }
      }

    return escaped;
  }

  std::string
  context_key(const std::string& parent_key,
              const std::string& subcommand_name)
  {
    return parent_key == ROOT_CONTEXT ? subcommand_name :
                                        parent_key + "/" + subcommand_name;
  }

  std::string
  description_key(const std::string& context, const std::string& candidate)
  {
    return context + "::" + candidate;
  }

  void
  collect_option_names(CLI::Option* option,
                       const std::string& context,
                       std::vector<std::string>& context_options,
                       DescriptionMap* option_descriptions)
  {
    std::string description;
    if (option_descriptions != nullptr)
      {
        description = zsh_describe_escape(
          normalize_description(option->get_description()));
      }

    for (const std::string& short_name : option->get_snames())
      {
        if (!short_name.empty())
          {
            const std::string candidate = "-" + short_name;
            push_unique(context_options, candidate);
            if (option_descriptions != nullptr)
              {
                (*option_descriptions)[description_key(context, candidate)] =
                  description;
              }
          }
      }

    for (const std::string& long_name : option->get_lnames())
      {
        if (!long_name.empty())
          {
            const std::string candidate = "--" + long_name;
            push_unique(context_options, candidate);
            if (option_descriptions != nullptr)
              {
                (*option_descriptions)[description_key(context, candidate)] =
                  description;
              }
          }
      }
  }

  void
  emit_completion_map_bash(std::ostringstream& script,
                           const std::string& map_name,
                           const CompletionMap& values)
  {
    script << "  declare -A " << map_name << "\n";
    for (const auto& [key, entries] : values)
      {
        script << "  " << map_name << "['"
               << escape_single_quoted_shell(key, "bash") << "']='"
               << escape_single_quoted_shell(join(entries, " "), "bash")
               << "'\n";
      }
    script << "\n";
  }

  void
  emit_completion_map_zsh(std::ostringstream& script,
                          const std::string& map_name,
                          const CompletionMap& values)
  {
    script << "  typeset -A " << map_name << "\n";
    script << "  " << map_name << "=(\n";
    for (const auto& [key, entries] : values)
      {
        script << "    '" << escape_single_quoted_shell(key, "zsh") << "' '"
               << escape_single_quoted_shell(join(entries, " "), "zsh")
               << "'\n";
      }
    script << "  )\n\n";
  }

  void
  emit_description_map_zsh(std::ostringstream& script,
                           const std::string& map_name,
                           const DescriptionMap& values)
  {
    script << "  typeset -A " << map_name << "\n";
    script << "  " << map_name << "=(\n";
    for (const auto& [key, value] : values)
      {
        script << "    '" << escape_single_quoted_shell(key, "zsh") << "' '"
               << escape_single_quoted_shell(value, "zsh") << "'\n";
      }
    script << "  )\n\n";
  }

  void
  emit_completion_map_pwsh(std::ostringstream& script,
                           const std::string& map_name,
                           const CompletionMap& values)
  {
    script << "$global:" << map_name << " = @{\n";
    for (const auto& [key, entries] : values)
      {
        script << "  '" << escape_single_quoted_shell(key, "pwsh") << "' = @(";
        for (std::size_t i = 0; i < entries.size(); ++i)
          {
            if (i > 0)
              {
                script << ", ";
              }
            script << "'" << escape_single_quoted_shell(entries[i], "pwsh")
                   << "'";
          }
        script << ")\n";
      }
    script << "}\n\n";
  }

  void
  emit_completion_map(std::ostringstream& script,
                      const std::string& map_name,
                      const CompletionMap& values,
                      const std::string& shell)
  {
    if (shell == "bash")
      {
        emit_completion_map_bash(script, map_name, values);
      }
    else if (shell == "zsh")
      {
        emit_completion_map_zsh(script, map_name, values);
      }
    else
      {
        emit_completion_map_pwsh(script, map_name, values);
      }
  }

  void
  collect_completion_data(CLI::App* app,
                          const std::string& context,
                          CompletionMap& subcommands,
                          CompletionMap& options,
                          DescriptionMap* subcommand_descriptions,
                          DescriptionMap* option_descriptions)
  {
    for (CLI::Option* option : app->get_options())
      {
        collect_option_names(option,
                             context,
                             options[context],
                             option_descriptions);
      }

    for (CLI::App* subcommand :
         app->get_subcommands([](CLI::App*) { return true; }))
      {
        const std::string subcommand_name = subcommand->get_name();
        if (subcommand_name.empty())
          {
            continue;
          }

        push_unique(subcommands[context], subcommand_name);
        if (subcommand_descriptions != nullptr)
          {
            (*subcommand_descriptions)[description_key(context,
                                                       subcommand_name)] =
              zsh_describe_escape(
                normalize_description(subcommand->get_description()));
          }
        collect_completion_data(subcommand,
                                context_key(context, subcommand_name),
                                subcommands,
                                options,
                                subcommand_descriptions,
                                option_descriptions);
      }
  }

  std::string
  generate_completion_script(CLI::App* app, const std::string& shell)
  {
    CompletionMap subcommands;
    CompletionMap options;
    const bool is_zsh = shell == "zsh";
    const bool is_pwsh = shell == "pwsh";
    DescriptionMap subcommand_descriptions;
    DescriptionMap option_descriptions;
    DescriptionMap* subcommand_descriptions_ptr =
      is_zsh ? &subcommand_descriptions : nullptr;
    DescriptionMap* option_descriptions_ptr =
      is_zsh ? &option_descriptions : nullptr;
    const char* current_word =
      is_zsh ? "${words[CURRENT]}" : "${COMP_WORDS[COMP_CWORD]}";
    const char* token_word = is_zsh ? "${words[i]}" : "${COMP_WORDS[i]}";
    const char* loop_start = is_zsh ? "2" : "1";
    const char* loop_end = is_zsh ? "CURRENT" : "COMP_CWORD";

    collect_completion_data(app,
                            ROOT_CONTEXT,
                            subcommands,
                            options,
                            subcommand_descriptions_ptr,
                            option_descriptions_ptr);

    std::ostringstream script;
    if (is_pwsh)
      {
        script << "# PowerShell completion for ifm3d\n";
        script << "# generated by: ifm3d generate-completion pwsh\n\n";
        script << "if (Get-Command Set-PSReadLineKeyHandler -ErrorAction "
                  "SilentlyContinue) {\n";
        script << "  Set-PSReadLineKeyHandler -Key Tab -ScriptBlock {\n";
        script << "    param($key, $arg)\n";
        script << "    $normalizePattern = "
                  "'^\\s*(ifm3d(?:\\.exe)?\\s+)-\\s+(--?\\S+)'\n";
        script << "    $line = ''\n";
        script << "    $cursor = 0\n";
        script << "    "
                  "[Microsoft.PowerShell.PSConsoleReadLine]::"
                  "GetBufferState([ref]$line, [ref]$cursor)\n";
        script << "    $normalizedBefore = [regex]::Replace($line, "
                  "$normalizePattern, '$1$2')\n";
        script << "    if ($normalizedBefore -ne $line) {\n";
        script << "      [Microsoft.PowerShell.PSConsoleReadLine]::Replace(0, "
                  "$line.Length, $normalizedBefore)\n";
        script << "      if ($cursor -ge $line.Length) {\n";
        script << "        "
                  "[Microsoft.PowerShell.PSConsoleReadLine]::"
                  "SetCursorPosition($normalizedBefore.Length)\n";
        script << "      }\n";
        script << "      "
                  "[Microsoft.PowerShell.PSConsoleReadLine]::GetBufferState(["
                  "ref]$line, [ref]$cursor)\n";
        script << "    }\n";
        script << "    $prefix = if ($cursor -gt 0) { "
                  "$line.Substring(0, $cursor) } else { '' }\n";
        script
          << "    if ($prefix -match '^\\s*ifm3d(?:\\.exe)?\\s+--?$') {\n";
        script << "      [Microsoft.PowerShell.PSConsoleReadLine]::Insert(' "
                  "')\n";
        script << "    }\n";
        script
          << "    [Microsoft.PowerShell.PSConsoleReadLine]::MenuComplete()\n";
        script << "\n";
        script << "    $lineAfter = ''\n";
        script << "    $cursorAfter = 0\n";
        script << "    "
                  "[Microsoft.PowerShell.PSConsoleReadLine]::GetBufferState(["
                  "ref]$lineAfter, [ref]$cursorAfter)\n";
        script << "    $normalized = [regex]::Replace($lineAfter, "
                  "$normalizePattern, '$1$2')\n";
        script << "    if ($normalized -ne $lineAfter) {\n";
        script << "      [Microsoft.PowerShell.PSConsoleReadLine]::Replace(0, "
                  "$lineAfter.Length, $normalized)\n";
        script << "      if ($cursorAfter -ge $lineAfter.Length) {\n";
        script << "        "
                  "[Microsoft.PowerShell.PSConsoleReadLine]::"
                  "SetCursorPosition($normalized.Length)\n";
        script << "      }\n";
        script << "    }\n";
        script << "  }\n";
        script << "}\n\n";
        emit_completion_map(script, "ifm3dSubcommands", subcommands, shell);
        emit_completion_map(script, "ifm3dOptions", options, shell);
        script << "Register-ArgumentCompleter -Native -CommandName @('ifm3d', "
                  "'ifm3d.exe') -ScriptBlock {\n";
        script << "  param($wordToComplete, $commandAst, $cursorPosition)\n\n";
        script << "  $ctx = 'root'\n";
        script << "  $elements = $commandAst.CommandElements\n";
        script << "  $previousToken = $null\n";
        script << "  # Context tracking skips option tokens, but not option "
                  "values.\n";
        script << "  # If an option argument matches a subcommand name, "
                  "context may shift incorrectly.\n";
        script << "  for ($i = 1; $i -lt $elements.Count; $i++) {\n";
        script << "    $token = $elements[$i].Extent.Text\n";
        script << "    if ($token -like '-*') { continue }\n";
        script << "    if ($token -eq $wordToComplete) { break }\n";
        script << "    if ($global:ifm3dSubcommands.ContainsKey($ctx) -and "
                  "($global:ifm3dSubcommands[$ctx] -contains $token)) {\n";
        script << "      $ctx = if ($ctx -eq 'root') { $token } else { "
                  "\"$ctx/$token\" }\n";
        script << "    }\n";
        script << "  }\n\n";
        script << "  if ($elements.Count -gt 1) {\n";
        script << "    $previousToken = $elements[$elements.Count - "
                  "1].Extent.Text\n";
        script << "  }\n";
        script << "  $isOptionContext = ($wordToComplete -like '-*' -or "
                  "($wordToComplete -eq '' -and ("
                  "$commandAst.Extent.Text -match '\\s--?$' -or "
                  "$previousToken -eq '-' -or "
                  "$previousToken -eq '--')))\n";
        script << "  $candidates = @()\n";
        script << "  if ($isOptionContext) {\n";
        script << "    if ($global:ifm3dOptions.ContainsKey($ctx)) { "
                  "$candidates += $global:ifm3dOptions[$ctx] }\n";
        script << "    if ($ctx -ne 'root' -and "
                  "$global:ifm3dOptions.ContainsKey('root')) { "
                  "$candidates += $global:ifm3dOptions['root'] }\n";
        script << "  } else {\n";
        script << "    if ($global:ifm3dSubcommands.ContainsKey($ctx)) { "
                  "$candidates += $global:ifm3dSubcommands[$ctx] }\n";
        script << "  }\n\n";
        script << "  $candidates | Where-Object { $_ -like "
                  "\"$wordToComplete*\" } | Sort-Object -Unique | "
                  "ForEach-Object {\n";
        script << "    $type = if ($_ -like '-*') { 'ParameterName' } "
                  "else { 'ParameterValue' }\n";
        script << "    [System.Management.Automation.CompletionResult]::"
                  "new($_, $_, $type, $_)\n";
        script << "  }\n";
        script << "}\n";
        return script.str();
      }

    if (is_zsh)
      {
        script << "#compdef ifm3d\n";
      }

    script << "# " << shell << " completion for ifm3d\n";
    script << "# generated by: ifm3d generate-completion " << shell << "\n\n";
    script << "_ifm3d_completion()\n";
    script << "{\n";
    script << "  local cur=\"" << current_word << "\"\n";
    script << "  local ctx=\"root\"\n";
    script << "  local token children candidates\n";
    if (is_zsh)
      {
        script << "  integer i\n";
      }
    script << "\n";
    emit_completion_map(script, "_ifm3d_subcommands", subcommands, shell);
    emit_completion_map(script, "_ifm3d_options", options, shell);
    if (is_zsh)
      {
        emit_description_map_zsh(script,
                                 "_ifm3d_subcommand_descriptions",
                                 subcommand_descriptions);
        emit_description_map_zsh(script,
                                 "_ifm3d_option_descriptions",
                                 option_descriptions);
      }
    script << "  # Context tracking skips option tokens, but not option "
              "values.\n";
    script << "  # If an option argument matches a subcommand name, "
              "context may shift incorrectly.\n";
    script << "  for ((i = " << loop_start << "; i < " << loop_end
           << "; ++i)); do\n";
    script << "    token=\"" << token_word << "\"\n";
    script << "    if [[ \"$token\" == -* ]]; then\n";
    script << "      continue\n";
    script << "    fi\n";
    script << "\n";
    script << "    children=\"${_ifm3d_subcommands[$ctx]}\"\n";
    script << "    if [[ \" $children \" == *\" $token \"* ]]; then\n";
    script << "      if [[ \"$ctx\" == \"root\" ]]; then\n";
    script << "        ctx=\"$token\"\n";
    script << "      else\n";
    script << "        ctx=\"$ctx/$token\"\n";
    script << "      fi\n";
    script << "    fi\n";
    script << "  done\n";
    script << "\n";
    script << "  if [[ \"$cur\" == -* ]]; then\n";
    script << "    if [[ \"$ctx\" == \"root\" ]]; then\n";
    script << "      candidates=\"${_ifm3d_options[root]}\"\n";
    script << "    else\n";
    script << "      candidates=\"${_ifm3d_options[root]} "
              "${_ifm3d_options[$ctx]}\"\n";
    script << "    fi\n";
    script << "  else\n";
    script << "    candidates=\"";
    script << "${_ifm3d_subcommands[$ctx]}";
    script << "\"\n";
    script << "  fi\n";
    script << "\n";
    if (is_zsh)
      {
        script << "  local desc desc_key\n";
        script << "  local -a raw_candidates described_candidates\n";
        script << "  local -A seen\n";
        script << "\n";
        script << "  if [[ \"$cur\" == -* ]]; then\n";
        script << "    raw_candidates=(${(s: :)candidates})\n";
        script << "    for token in ${raw_candidates[@]}; do\n";
        script << "      [[ -z \"$token\" || -n \"${seen[$token]}\" ]] && "
                  "continue\n";
        script << "      seen[$token]=1\n";
        script << "      desc_key=\"$ctx::$token\"\n";
        script << "      desc=\"${_ifm3d_option_descriptions[$desc_key]}\"\n";
        script << "      if [[ -z \"$desc\" ]]; then\n";
        script
          << "        desc=\"${_ifm3d_option_descriptions[root::$token]}\"\n";
        script << "      fi\n";
        script << "      described_candidates+=(\"$token:${desc:-option}\")\n";
        script << "    done\n";
        script << "    _describe -t ifm3d-options 'ifm3d options' "
                  "described_candidates\n";
        script << "  else\n";
        script << "    raw_candidates=(${(s: :)candidates})\n";
        script << "    for token in ${raw_candidates[@]}; do\n";
        script << "      [[ -z \"$token\" || -n \"${seen[$token]}\" ]] && "
                  "continue\n";
        script << "      seen[$token]=1\n";
        script << "      "
                  "desc=\"${_ifm3d_subcommand_descriptions[$ctx::$token]}\"\n";
        script
          << "      described_candidates+=(\"$token:${desc:-subcommand}\")"
             "\n";
        script << "    done\n";
        script << "    _describe -t ifm3d-subcommands 'ifm3d subcommands' "
                  "described_candidates\n";
        script << "  fi\n";
      }
    else
      {
        script
          << "  COMPREPLY=( $(compgen -W \"$candidates\" -- \"$cur\") )\n";
      }
    script << "}\n\n";

    if (is_zsh)
      {
        script << "compdef _ifm3d_completion ifm3d\n";
      }
    else
      {
        script << "complete -o bashdefault -o default ";
        script << "-F _ifm3d_completion ifm3d\n";
      }

    return script.str();
  }
} // end: namespace

CLI::App*
ifm3d::GenerateCompletionApp::CreateCommand(CLI::App* parent)
{
  CLI::App* command =
    parent->add_subcommand("generate-completion",
                           "Generate shell completion script to stdout");

  command->add_option("shell", this->shell, "Target shell: bash, zsh, pwsh")
    ->required()
    ->check(CLI::IsMember({"bash", "zsh", "pwsh"}, CLI::ignore_case));

  return command;
}

void
ifm3d::GenerateCompletionApp::Execute(CLI::App* app)
{
  std::transform(
    this->shell.begin(),
    this->shell.end(),
    this->shell.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (this->shell == "bash")
    {
      std::cout << generate_completion_script(app->get_parent(), "bash");
    }
  else if (this->shell == "zsh")
    {
      std::cout << generate_completion_script(app->get_parent(), "zsh");
    }
  else if (this->shell == "pwsh")
    {
      std::cout << generate_completion_script(app->get_parent(), "pwsh");
    }
}
