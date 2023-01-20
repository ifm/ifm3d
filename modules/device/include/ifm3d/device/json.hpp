// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_JSON_HPP
#define IFM3D_DEVICE_JSON_HPP

#include <ifm3d/device/json_impl.hpp>

#ifdef DOXYGEN

namespace ifm3d
{
  /**
   * @section documentation Documentation
   *
   * ifm3d provides a copy of nlohmann::json, please see it's documentation for
   * detailed usage: https://json.nlohmann.me/
   *
   * @section converter Using ifm3d::json together with nlohmann::json
   *
   * Working with both nlohmann::json and ifm3d::json together doesn't require
   * any specific setup, however the types are not automatically convertible,
   * so to get a nlohmann::json instance from a ifm3d::json instance one has to
   * convert the json to a string and then parse it again.
   *
   * @code{.cpp}
   * ifm3d::json j1 = ...;
   * nlohmann::json j2 = nlohmann::json::parse(j1.dump(0));
   * @endcode
   *
   * To ease this conversion ifm3d provides a custom converter which can be
   * used by defining `IFM3D_JSON_NLOHMANN_COMPAT` before including
   * ifm3d::json.
   *
   * @code{.cpp}
   * #include <nlohmann/json.hpp> // nlohmann json needs to be included before
   *                              // any ifm3d include.
   *
   * #define IFM3D_JSON_NLOHMANN_COMPAT // enable the nlohmann json converter
   * #include <ifm3d/device/json.hpp>
   * @endcode
   *
   * After this the values can just be assigned and will be converted
   * automatically
   *
   * @code{.cpp}
   * ifm3d::json j1 = ...;
   * nlohman::json j2 = j1;
   * @endcode
   */
  class json
  {
  };
}

#else

#  ifndef IFM3D_JSON_USE_GLOBAL_UDLS
#    define IFM3D_JSON_USE_GLOBAL_UDLS 0
#  endif

#  ifdef IFM3D_JSON_NLOHMANN_COMPAT
#    ifndef NLOHMANN_JSON_VERSION_MAJOR
#      error Enabled ifm3d::json nlohmann::json compatiblity but nlohmann::json ist not available, make sure to include "nlohmann/json.hpp" before including "ifm3d/device/json.hpp"!
#    endif

namespace nlohmann
{
  template <>
  struct adl_serializer<ifm3d::json>
  {
    static ifm3d::json
    from_json(const nlohmann::json& nl)
    {
      return ifm3d::json::parse(nl.dump(0));
    }

    static void
    to_json(nlohmann::json& nl, ifm3d::json ifm)
    {
      nl = nlohmann::json::parse(ifm.dump(0));
    }
  };
}

#  endif // IFM3D_JSON_NLOHMANN_COMPAT

#endif

#endif // IFM3D_DEVICE_JSON_HPP