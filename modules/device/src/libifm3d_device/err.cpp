/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/device/err.h>
#include <cstring>
#include <fmt/core.h>

// library errors
const int IFM3D_NO_ERRORS = 0;
const int IFM3D_XMLRPC_FAILURE = -100000;
const int IFM3D_XMLRPC_TIMEOUT = -100001;
const int IFM3D_JSON_ERROR = -100002;
const int IFM3D_NO_ACTIVE_APPLICATION = -100003;
const int IFM3D_SUBCOMMAND_ERROR = -100004;
const int IFM3D_IO_ERROR = -100005;
const int IFM3D_THREAD_INTERRUPTED = -100006;
const int IFM3D_PCIC_BAD_REPLY = -100007;
const int IFM3D_UNSUPPORTED_OP = -100008;
const int IFM3D_IMG_CHUNK_NOT_FOUND = -100009;
const int IFM3D_PIXEL_FORMAT_ERROR = -100010;
const int IFM3D_UNSUPPORTED_DEVICE = -100011;
const int IFM3D_UPDATE_ERROR = -100012;
const int IFM3D_RECOVERY_CONNECTION_ERROR = -100013;
const int IFM3D_PCICCLIENT_UNSUPPORTED_DEVICE = -100014;
const int IFM3D_HEADER_VERSION_MISMATCH = -100015;
const int IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE = -100016;
const int IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE = -100017;
const int IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE = -100018;
const int IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE = -100019;
const int IFM3D_CURL_ERROR = -100020;
const int IFM3D_CURL_TIMEOUT = -100021;
const int IFM3D_CURL_ABORTED = -100022;
const int IFM3D_SWUPDATE_BAD_STATE = -100023;
const int IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED = -100024;
const int IFM3D_PROXY_AUTH_REQUIRED = -100025;
const int IFM3D_PIXEL_FORMAT_NOT_SUPPORTED = 100026;
const int IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_DEVICE = -100027;
const int IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_FIRMWARE = -100028;
const int IFM3D_INVALID_PORT = -100029;
const int IFM3D_TOOL_COMMAND_UNSUPPORTED_DEVICE = -100030;
const int IFM3D_UNSUPPORTED_SCHEMA_ON_DEVICE = -100031;
const int IFM3D_BUFFER_ID_NOT_AVAILABLE = -100032;
const int IFM3D_NETWORK_ERROR = -100033;
const int IFM3D_SYSTEM_ERROR = -100034;
const int IFM3D_CORRUPTED_STRUCT = -100035;
const int IFM3D_DEVICE_PORT_INCOMPATIBLE_WITH_ORGANIZER = -100036;
const int IFM3D_DEVICE_PORT_NOT_SUPPORTED = -100037;
// sensor errors
const int IFM3D_XMLRPC_OBJ_NOT_FOUND = 100000;
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
const int IFM3D_NO_APP_IN_EDIT_MODE = 101016;
const int IFM3D_CANNOT_SW_TRIGGER = 101024;
const int IFM3D_INVALID_IMAGER_TYPE = 101027;
const int IFM3D_UNSUPPORTED_APP_TYPE = 101028;
const int IFM3D_PIN_ALREADY_IN_USE = 101032;
const int IFM3D_NO_SUCH_MODEL_OR_ROI = 101033;
const int IFM3D_TEMPORAL_FILTER_TRIGGER_CONFLICT = 101036;
const int IFM3D_EEPROM_FAIL = 101046;
const int IFM3D_INVALID_NET_CONFIG = 101051;
const int IFM3D_IMPORT_EXPORT_IN_PROGRESS = 101052;
const int IFM3D_LED_DUTY_CYCLE_VIOLATION = 101055;
const int IFM3D_AUTO_EXPOSURE_NOT_SUPPORTED = 101056;
const int IFM3D_INVALID_FIRMWARE_VERSION = 101058;

const char*
ifm3d::strerror(int errnum)
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
    case IFM3D_SUBCOMMAND_ERROR:
      return "Lib: Missing or invalid sub-command";
    case IFM3D_IO_ERROR:
      return "Lib: I/O error";
    case IFM3D_THREAD_INTERRUPTED:
      return "Lib: Thread interrupted";
    case IFM3D_PCIC_BAD_REPLY:
      return "Lib: Bad or unexpected data from PCIC";
    case IFM3D_UNSUPPORTED_OP:
      return "Lib: An attempted operation is not supported by the device";
    case IFM3D_IMG_CHUNK_NOT_FOUND:
      return "Lib: Image chunk not found";
    case IFM3D_PIXEL_FORMAT_ERROR:
      return "Lib: Pixel format error - didn't expect a particular pixel type";
    case IFM3D_UNSUPPORTED_DEVICE:
      return "Lib: The detected device is not supported by the library";
    case IFM3D_UPDATE_ERROR:
      return "Lib: An error occured while performing the update";
    case IFM3D_RECOVERY_CONNECTION_ERROR:
      return "Lib: Couldn't connect to the device (make sure the device is in "
             "Recovery Mode)";
    case IFM3D_PCICCLIENT_UNSUPPORTED_DEVICE:
      return "Lib: PCICClient is not supported for this device";
    case IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE:
      return "Lib: Intrinsic parameter is not supported by Device";
    case IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE:
      return "Lib:  Intrinsic parameter is not supported by Firmware";
    case IFM3D_CURL_ERROR:
      return "Lib: Encountered an unexpected error in the CURL library";
    case IFM3D_CURL_TIMEOUT:
      return "Lib: An HTTP operation with CURL timed out. Can you 'ping' the "
             "camera?";
    case IFM3D_CURL_ABORTED:
      return "Lib: An HTTP operation with CURL was aborted.";
    case IFM3D_SWUPDATE_BAD_STATE:
      return "Lib: SWUpdater process on camera is in invalid state. Reboot "
             "the camera and try again.";
    case IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED:
      return "Confidence image format not supported by ifm3d";
    case IFM3D_PROXY_AUTH_REQUIRED:
      return "Lib: Server returned HTTP 407 Proxy authentication required. "
             "Disable proxy and try again.";
    case IFM3D_PIXEL_FORMAT_NOT_SUPPORTED:
      return "Lib: unknown size of the pixel format";
    case IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_DEVICE:
      return "Lib: Current device does not support distance noise image";
    case IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_FIRMWARE:
      return "Lib: Firmware does not support distance noise image";
    case IFM3D_INVALID_PORT:
      return "The given port is invalid or not connected. Please make sure "
             "the port is connected and try again.";
    case IFM3D_TOOL_COMMAND_UNSUPPORTED_DEVICE:
      return "Lib: This command is not supported by the connected device";
    case IFM3D_UNSUPPORTED_SCHEMA_ON_DEVICE:
      return "Lib: One or multiple schema values are not supported by device";
    case IFM3D_BUFFER_ID_NOT_AVAILABLE:
      return "Lib: A buffer with the requested buffer_id is not available.";
    case IFM3D_NETWORK_ERROR:
      return "Lib: Network error";
    case IFM3D_SYSTEM_ERROR:
      return "Lib: System error";
    case IFM3D_CORRUPTED_STRUCT:
      return "Lib: The given Buffer does not contain the expected struct or "
             "the data is corrupted.";
    case IFM3D_DEVICE_PORT_INCOMPATIBLE_WITH_ORGANIZER:
      return "Lib: The device's PCIC port does not match any compatible "
             "organizers";
    case IFM3D_DEVICE_PORT_NOT_SUPPORTED:
      return "Lib: Port is not supported by the device";
    case IFM3D_XMLRPC_OBJ_NOT_FOUND:
      return "Sensor: XMLRPC obj not found - trying to access dead session?";
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
    case IFM3D_NO_APP_IN_EDIT_MODE:
      return "Sensor: There is no application in edit-mode";
    case IFM3D_EEPROM_FAIL:
      return "Sensor: Failed to read EEPROM";
    case IFM3D_CANNOT_SW_TRIGGER:
      return "Sensor: Operation mode does not allow S/W trigger";
    case IFM3D_INVALID_IMAGER_TYPE:
      return "Sensor: Unsupported imager type";
    case IFM3D_UNSUPPORTED_APP_TYPE:
      return "Sensor: Unsupported application type";
    case IFM3D_PIN_ALREADY_IN_USE:
      return "Sensor: App requires a pin that is already in use";
    case IFM3D_NO_SUCH_MODEL_OR_ROI:
      return "Sensor: Logic layer contains model-roi which does not exist";
    case IFM3D_TEMPORAL_FILTER_TRIGGER_CONFLICT:
      return "Sensor: Temporal filter conflicts with trigger mode";
    case IFM3D_IMPORT_EXPORT_IN_PROGRESS:
      return "Sensor: Device busy, import/export in progress";
    case IFM3D_INVALID_NET_CONFIG:
      return "Sensor: Invalid network config";
    case IFM3D_LED_DUTY_CYCLE_VIOLATION:
      return "Sensor: LED duty cycle violation";
    case IFM3D_AUTO_EXPOSURE_NOT_SUPPORTED:
      return "Sensor: Auto-exposure not supported";
    case IFM3D_INVALID_FIRMWARE_VERSION:
      return "Sensor: Invalid firmware version";
    case IFM3D_HEADER_VERSION_MISMATCH:
      return "Sensor: Header version mismatch while parsing data";
    default:
      return ::strerror(errnum);
    }
}

ifm3d::Error::Error(int errnum, const std::string& msg)
  : errnum_(errnum),
    errmsg_(msg),
    what_(msg.empty() ? ifm3d::strerror(errnum) :
                        fmt::format("{0}: {1}", ifm3d::strerror(errnum), msg))
{}

int
ifm3d::Error::code() const noexcept
{
  return this->errnum_;
}

const char*
ifm3d::Error::what() const noexcept
{
  return this->what_.c_str();
}

const char*
ifm3d::Error::message() const noexcept
{
  return this->errmsg_.c_str();
}
