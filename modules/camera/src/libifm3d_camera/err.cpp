/*
 * Copyright (C) 2017 Love Park Robotics, LLC
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

#include <ifm3d/camera/err.h>
#include <cstring>

const int IFM3D_NO_ERRORS = 0;

const char *ifm3d::strerror(int errnum)
{
  switch (errnum)
    {
    case IFM3D_NO_ERRORS:
      return "OK";
    default:
      return ::strerror(errnum);
    }
}

ifm3d::error_t::error_t(int errnum)
  : std::exception(), errnum_(errnum) { }

int ifm3d::error_t::code() const noexcept
{
  return this->errnum_;
}

const char *ifm3d::error_t::what() const noexcept
{
  return ifm3d::strerror(this->code());
}
