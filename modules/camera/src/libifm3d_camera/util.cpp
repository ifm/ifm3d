/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/camera/util.h>
#include <sstream>
#include <string>
#include <vector>

std::string&
ifm3d::ltrim(std::string& str, const std::string& chars)
{
  str.erase(0, str.find_first_not_of(chars));
  return str;
}

std::string&
ifm3d::rtrim(std::string& str, const std::string& chars)
{
  str.erase(str.find_last_not_of(chars) + 1);
  return str;
}

std::string&
ifm3d::trim(std::string& str, const std::string& chars)
{
  return ifm3d::ltrim(ifm3d::rtrim(str, chars), chars);
}

std::vector<std::string>
ifm3d::split(const std::string& in, char delim)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(in);
  while (std::getline(tokenStream, token, delim))
    {
      tokens.push_back(token);
    }
  return tokens;
}
