// -*- c++ -*-
/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_JSON_HPP
#define IFM3D_DEVICE_JSON_HPP

#ifdef DOXYGEN

namespace ifm3d
{
  /**
   * @ingroup Common
   *
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
   * #include <ifm3d/common/json.hpp>
   * @endcode
   *
   * After this the values can just be assigned and will be converted
   * automatically
   *
   * @code{.cpp}
   * ifm3d::json j1 = ...;
   * nlohman::json j2 = j1;
   * @endcode
   *
   * @subsection udl User Defined Literals
   *
   * Using the user-defined string literals `operator""_json` and
   * `operator""_json_pointer`
   *
   * @code{.cpp}
   * using namespace ifm3d::literals;
   * ifm3d::json j = "[1,2,3]"_json;
   * @endcode
   *
   * ambiguity errors will occur since by default nlohmann::json currently
   * places it's `operator""_json` and `operator""_json_pointer` into the
   * global namespace. To solve this nlohman::json can be configured to place
   * them into the nlohmann namespace by defining
   * @code{.cpp}
   * #define JSON_USE_GLOBAL_UDLS 0
   * #include <nlohmann/json.hpp>
   * @endcode
   * before including it (see the [nlohmann::json
   * doc](https://json.nlohmann.me/api/macros/json_use_global_udls/) for more
   * details).
   *
   * After this the correct namespaces can always be brought into scope when
   * needed, but some care needs to be given to never bring both into the same
   * scope, e.g. the following **WILL NOT WORK**:
   *
   * @code{.cpp}
   * // Bring both UDLs into scope
   * using namespace nlohmann::json_literals;
   * using namespace ifm3d::json_literals;
   *
   * int main()
   * {
   *     // auto j = "42"_json; // This line would fail to compile,
   *                            // because _json is ambigous
   *
   *     std::cout << j << std::endl;
   * }
   * @endcode
   *
   * To counteract this, the namspaces can be brought into smaller scopes, the
   * following will all be valid
   *
   * @code{.cpp}
   * void json_nl()
   * {
   *    using namespace nlohmann::literals;
   *    auto nl = "{}"_json;
   * }
   *
   * void json_ifm()
   * {
   *   using namespace ifm3d::literals;
   *   auto ifm = "{}"_json;
   * }
   *
   * void json_both()
   * {
   *   nlohmann::json nl;
   *   ifm3d::json ifm;
   *
   *   {
   *     using namespace nlohmann::literals;
   *     nl = "{}"_json;
   *   }
   *
   *   {
   *     using namespace ifm3d::literals;
   *     ifm = "{}"_json;
   *   }
   * }
   *
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

#  include <ifm3d/common/json_impl.hpp>

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