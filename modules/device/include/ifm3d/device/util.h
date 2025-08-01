// -*- c++ -*-
/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_UTIL_H
#define IFM3D_DEVICE_UTIL_H

#include <string>
#include <vector>
#include <ifm3d/device/module_device.h>
#include <cstdint>

namespace httplib
{
  class Result;
}

namespace ifm3d
{
  /**
   * Trim whitespace from left side of string
   */
  IFM3D_EXPORT std::string& ltrim(std::string& str,
                                  const std::string& chars = "\t\n\v\f\r ");

  /**
   * Trim whitespace from right side of string
   */
  IFM3D_EXPORT std::string& rtrim(std::string& str,
                                  const std::string& chars = "\t\n\v\f\r ");

  /**
   * Trim whitespace from left and right side of string
   */
  IFM3D_EXPORT std::string& trim(std::string& str,
                                 const std::string& chars = "\t\n\v\f\r ");

  /**
   * Split a string into its component parts based on a delimeter
   */
  IFM3D_EXPORT std::vector<std::string> split(const std::string& in,
                                              char delim);

  IFM3D_EXPORT std::string base64_encode(
    const std::vector<std::uint8_t>& data);

  IFM3D_EXPORT std::vector<std::uint8_t> base64_decode(
    const std::string& base64);

  IFM3D_EXPORT void check_http_result(httplib::Result const& res);

  IFM3D_EXPORT bool IsStdinAvailable(int timeout_seconds = 2);

} // end: namespace ifm3d

#endif // IFM3D_CAMERA_UTIL_H
