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

// library errors
const int IFM3D_NO_ERRORS = 0;
const int IFM3D_XMLRPC_FAILURE = -100000;
const int IFM3D_XMLRPC_TIMEOUT = -100001;
const int IFM3D_JSON_ERROR = -100002;
const int IFM3D_NO_ACTIVE_APPLICATION = -100003;
const int IFM3D_SUBCOMMAND_ERROR = -100004;
const int IFM3D_IO_ERROR = -100005;

// sensor errors
const int IFM3D_INVALID_PARAM = 101000;
const int IFM3D_INVALID_VALUE_TYPE = 101001;
const int IFM3D_VALUE_OUT_OF_RANGE = 101002;
const int IFM3D_READONLY_PARAM = 101003;
const int IFM3D_SESSION_ALREADY_ACTIVE = 101004;
const int IFM3D_INVALID_PASSWORD = 101005;
const int IFM3D_INVALID_SESSIONID = 101006;
const int IFM3D_COULD_NOT_REBOOT = 101007;
const int IFM3D_INVALID_FORMAT = 101010;
const int IFM3D_INVALID_DEVICE_TYPE = 101011;
const int IFM3D_INVALID_IMPORT_FLAGS = 101012;
const int IFM3D_INVALID_APP_INDEX = 101013;
const int IFM3D_APP_IN_EDIT_MODE = 101014;
const int IFM3D_MAX_APP_LIMIT = 101015;
const int IFM3D_UNSUPPORTED_APP_TYPE = 101028;
const int IFM3D_EEPROM_FAIL = 101046;
const int IFM3D_IMPORT_EXPORT_IN_PROGRESS = 101052;
const int IFM3D_INVALID_FIRMWARE_VERSION = 101058;

const char *ifm3d::strerror(int errnum)
{
  switch (errnum)
    {
    case IFM3D_NO_ERRORS:
      return "OK";
    case IFM3D_XMLRPC_FAILURE:
      return "Lib: Unknown XMLRPC failure";
    case IFM3D_XMLRPC_TIMEOUT:
      return "Lib: XMLRPC Timeout - can you `ping' the sensor?";
    case IFM3D_JSON_ERROR:
      return "Lib: Error processing JSON";
    case IFM3D_NO_ACTIVE_APPLICATION:
      return "Lib: No application is marked active";
    case  IFM3D_SUBCOMMAND_ERROR:
      return "Lib: Missing or invalid sub-command";
    case IFM3D_IO_ERROR:
      return "Lib: I/O error";
    case IFM3D_INVALID_PARAM:
      return "Sensor: The parameter name is invalid";
    case IFM3D_INVALID_VALUE_TYPE:
      return "Sensor: Parameter value data type is invalid";
    case IFM3D_VALUE_OUT_OF_RANGE:
      return "Sensor: Value out of range";
    case IFM3D_READONLY_PARAM:
      return "Sensor: Cannot mutate a read-only parameter";
    case IFM3D_SESSION_ALREADY_ACTIVE:
      return "Sensor: Device already has an edit-session active";
    case IFM3D_INVALID_PASSWORD:
      return "Sensor: Invalid password";
    case IFM3D_INVALID_SESSIONID:
      return "Sensor: Invalid session id";
    case IFM3D_COULD_NOT_REBOOT:
      return "Sensor: Could not execute reboot command";
    case IFM3D_INVALID_FORMAT:
      return "Sensor: Data format is invalid";
    case IFM3D_INVALID_DEVICE_TYPE:
      return "Sensor: Invalid device type";
    case IFM3D_INVALID_IMPORT_FLAGS:
      return "Sensor: Invalid import flags";
    case IFM3D_INVALID_APP_INDEX:
      return "Sensor: There is no application at the supplied index";
    case IFM3D_APP_IN_EDIT_MODE:
      return "Sensor: Operation not allowed while an app is in edit mode";
    case IFM3D_MAX_APP_LIMIT:
      return "Sensor: Maximum number of applications has been reached";
    case IFM3D_EEPROM_FAIL:
      return "Sensor: Failed to read EEPROM";
    case  IFM3D_UNSUPPORTED_APP_TYPE:
      return "Sensor: Unsupported application type";
    case IFM3D_IMPORT_EXPORT_IN_PROGRESS:
      return "Sensor: Device busy, import/export in progress";
    case IFM3D_INVALID_FIRMWARE_VERSION:
      return "Sensor: Invalid firmware version";
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
