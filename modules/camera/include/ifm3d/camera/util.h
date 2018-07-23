// -*- c++ -*-
/*
 * Copyright (C) 2018 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IFM3D_CAMERA_UTIL_H__
#define __IFM3D_CAMERA_UTIL_H__

#include <string>
#include <vector>

namespace ifm3d
{
  /**
   * Trim whitespace from left side of string
   */
  std::string&
  ltrim(std::string& str, const std::string& chars = "\t\n\v\f\r ");

  /**
   * Trim whitespace from right side of string
   */
  std::string&
  rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ");

  /**
   * Trim whitespace from left and right side of string
   */
  std::string&
  trim(std::string& str, const std::string& chars = "\t\n\v\f\r ");

  /**
   * Split a string into its component parts based on a delimeter
   */
  std::vector<std::string> split(const std::string& in, char delim);

} // end: namespace ifm3d

#endif // __IFM3D_CAMERA_UTIL_H__
