// -*- c++ -*-
/*
 * Copyright 2022 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_SEMVER_H
#define IFM3D_DEVICE_SEMVER_H

#include <iostream>
#include <optional>
#include <string>
#include <vector>
#include <ifm3d/device/device_export.h>

namespace ifm3d
{
  /** @ingroup Device */
  struct IFM3D_DEVICE_EXPORT SemVer
  {
    SemVer(size_t major,
           size_t minor,
           size_t patch,
           const std::optional<std::string> prerelease = std::nullopt,
           const std::optional<std::string> build_meta = std::nullopt)
      : major_num(major),
        minor_num(minor),
        patch_num(patch),
        prerelease(prerelease),
        build_meta(build_meta)

    {}

    const size_t major_num;
    const size_t minor_num;
    const size_t patch_num;
    const std::optional<std::string> prerelease;
    const std::optional<std::string> build_meta;

    constexpr bool
    operator<(const SemVer& rhs) const
    {
      // Note: sorting by prerelease is not implemented as it's not needed for
      // our usecase
      return major_num < rhs.major_num ||
             (major_num == rhs.major_num &&
              (minor_num < rhs.minor_num ||
               (minor_num == rhs.minor_num && (patch_num < rhs.patch_num))));
    }

    constexpr bool
    operator==(const SemVer& rhs) const
    {
      return ((major_num == rhs.major_num) && (minor_num == rhs.minor_num) &&
              (patch_num == rhs.patch_num) && (prerelease == rhs.prerelease) &&
              (build_meta == rhs.build_meta));
    }

    constexpr bool
    operator!=(const SemVer& rhs) const
    {
      return !(*this == rhs);
    }

    constexpr bool
    operator>=(const SemVer& rhs) const
    {
      return !(*this < rhs);
    }

    constexpr bool
    operator>(const SemVer& rhs) const
    {
      return rhs < *this;
    }

    constexpr bool
    operator<=(const SemVer& rhs) const
    {
      return !(rhs < *this);
    }

    /* To support fmt ostream */
    friend std::ostream&
    operator<<(std::ostream& os, const SemVer& version)
    {
      os << version.major_num << '.' << version.minor_num << '.'
         << version.patch_num;

      if (version.prerelease.has_value())
        {
          os << '-' << version.prerelease.value();
        }

      if (version.build_meta.has_value())
        {
          os << '+' << version.build_meta.value();
        }

      return os;
    }

    static std::optional<SemVer> Parse(const std::string& version_string);
  };
} // end: namespace ifm3d

#endif // IFM3D_DEVICE_SEMVER_H
