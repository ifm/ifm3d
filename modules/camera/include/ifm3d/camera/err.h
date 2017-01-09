// -*- c++ -*-
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

#ifndef __IFM3D_CAMERA_ERR_H__
#define __IFM3D_CAMERA_ERR_H__

#include <exception>

// library errors
extern const int IFM3D_NO_ERRORS;
extern const int IFM3D_XMLRPC_FAILURE;
extern const int IFM3D_XMLRPC_TIMEOUT;

// sensor errors
extern const int IFM3D_INVALID_PARAM;
extern const int IFM3D_INVALID_VALUE_TYPE;
extern const int IFM3D_VALUE_OUT_OF_RANGE;
extern const int IFM3D_SESSION_ALREADY_ACTIVE;
extern const int IFM3D_INVALID_PASSWORD;
extern const int IFM3D_INVALID_SESSIONID;
extern const int IFM3D_COULD_NOT_REBOOT;
extern const int IFM3D_EEPROM_FAIL;

namespace ifm3d
{
  /**
   * @brief Human consumable string for an IFM3D error
   *
   * @param[in] errnum The error number to translate to a string
   * @return A stringified version of the error
   */
  const char *strerror(int errnum);

  /**
   * Exception wrapper for library and system errors encountered by the
   * library.
   */
  class error_t : public std::exception
  {
  public:
    /**
     * The ctor simply sets the error value into a local instance variable that
     * may be retrieved with a call to `code()`.
     */
    error_t(int errnum);

    /**
     * Exception message
     */
    virtual const char *what() const noexcept;

    /**
     * Accessor to the underlying error code
     */
    int code() const noexcept;

  private:
    /**
     * The error code from the sensor/system/library that this exception
     * wraps.
     */
    int errnum_;

  }; // end: class error_t

} // end: namespace ifm3d

#endif // __IFM3D_CAMERA_ERR_H__
