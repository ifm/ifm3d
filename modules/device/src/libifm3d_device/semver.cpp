/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/device/semver.h>

#include <regex>

// https://semver.org/#is-there-a-suggested-regular-expression-regex-to-check-a-semver-string
const std::regex SEMVER_REGEX(
  R"_(^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(?:\.(0|[1-9]\d*))?(?:-((?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*)(?:\.(?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*))*))?(?:\+([0-9a-zA-Z-]+(?:\.[0-9a-zA-Z-]+)*))?$)_",
  std::regex_constants::optimize);
constexpr std::size_t SEMVER_REGEX_GROUP_MAJOR = 1;
constexpr std::size_t SEMVER_REGEX_GROUP_MINOR = 2;
constexpr std::size_t SEMVER_REGEX_GROUP_PATCH = 3;
constexpr std::size_t SEMVER_REGEX_GROUP_BUILD = 4;
constexpr std::size_t SEMVER_REGEX_GROUP_PRERELEASE = 5;
constexpr std::size_t SEMVER_REGEX_GROUP_BUILD_META = 6;

std::optional<ifm3d::SemVer>
ifm3d::SemVer::Parse(const std::string& version_string)
{
  std::smatch match;
  if (!std::regex_match(version_string, match, SEMVER_REGEX))
    {
      return std::nullopt;
    }

  auto major = std::stoull(match[SEMVER_REGEX_GROUP_MAJOR].str());
  auto minor = std::stoull(match[SEMVER_REGEX_GROUP_MINOR].str());
  auto patch = std::stoull(match[SEMVER_REGEX_GROUP_PATCH].str());

  std::optional<std::string> prerelease;
  if (match[SEMVER_REGEX_GROUP_BUILD].matched)
    {
      prerelease = match[SEMVER_REGEX_GROUP_BUILD].str();
    }
  if (match[SEMVER_REGEX_GROUP_PRERELEASE].matched)
    {
      prerelease = match[SEMVER_REGEX_GROUP_PRERELEASE].str();
    }

  std::optional<std::string> build_meta;
  if (match[SEMVER_REGEX_GROUP_BUILD_META].matched)
    {
      build_meta = match[SEMVER_REGEX_GROUP_BUILD_META].str();
    }

  return SemVer(major, minor, patch, prerelease, build_meta);
}