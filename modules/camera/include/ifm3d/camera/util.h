// -*- c++ -*-
/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_CAMERA_UTIL_H
#define IFM3D_CAMERA_UTIL_H

#include <string>
#include <vector>

namespace ifm3d
{
  /**
   * Trim whitespace from left side of string
   */
  std::string& ltrim(std::string& str,
                     const std::string& chars = "\t\n\v\f\r ");

  /**
   * Trim whitespace from right side of string
   */
  std::string& rtrim(std::string& str,
                     const std::string& chars = "\t\n\v\f\r ");

  /**
   * Trim whitespace from left and right side of string
   */
  std::string& trim(std::string& str,
                    const std::string& chars = "\t\n\v\f\r ");

  /**
   * Split a string into its component parts based on a delimeter
   */
  std::vector<std::string> split(const std::string& in, char delim);

} // end: namespace ifm3d

#endif // IFM3D_CAMERA_UTIL_H
