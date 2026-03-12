// -*- c++ -*-
/*
 * Copyright 2022 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_SEMVER_H
#define IFM3D_DEVICE_SEMVER_H

#include <fmt/format.h>
#include <ifm3d/device/module_device.h>
#include <iostream>
#include <optional>
#include <string>

namespace ifm3d
{
  /** @ingroup Device */
  struct IFM3D_EXPORT SemVer
  {
    SemVer(size_t major,
           size_t minor,
           size_t patch,
           const std::optional<std::string>& prerelease = std::nullopt,
           const std::optional<std::string>& build_meta = std::nullopt)
      : major_num(major),
        minor_num(minor),
        patch_num(patch),
        prerelease(prerelease),
        build_meta(build_meta)

    {}

    size_t major_num;
    size_t minor_num;
    size_t patch_num;
    std::optional<std::string> prerelease;
    std::optional<std::string> build_meta;

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

    [[nodiscard]] std::string
    ToString() const
    {
      std::string version = std::to_string(major_num) + "." +
                            std::to_string(minor_num) + "." +
                            std::to_string(patch_num);

      if (prerelease.has_value())
        {
          version += "-" + prerelease.value();
        }

      if (build_meta.has_value())
        {
          version += "+" + build_meta.value();
        }

      return version;
    }

    /* To support fmt ostream */
    friend std::ostream&
    operator<<(std::ostream& os, const SemVer& version)
    {
      return os << version.ToString();
    }

    static std::optional<SemVer> Parse(const std::string& version_string);
  };
} // end: namespace ifm3d

template <>
struct fmt::formatter<ifm3d::SemVer> : fmt::formatter<std::string_view>
{
  auto
  format(const ifm3d::SemVer& version, format_context& ctx) const
  {
    const auto value = version.ToString();
    return fmt::formatter<std::string_view>::format(value, ctx);
  }
};

#endif // IFM3D_DEVICE_SEMVER_H
