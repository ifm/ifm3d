// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_DEVICE_ERR_H
#define IFM3D_DEVICE_ERR_H

#include <exception>
#include <string>
#include <ifm3d/device/device_export.h>

/** @ingroup Device
 * @{
 */
/** @defgroup ErrorCodes
 * @{
 */
// library errors
extern IFM3D_DEVICE_EXPORT const int IFM3D_NO_ERRORS;
extern IFM3D_DEVICE_EXPORT const int IFM3D_XMLRPC_FAILURE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_XMLRPC_TIMEOUT;
extern IFM3D_DEVICE_EXPORT const int IFM3D_JSON_ERROR;
extern IFM3D_DEVICE_EXPORT const int IFM3D_NO_ACTIVE_APPLICATION;
extern IFM3D_DEVICE_EXPORT const int IFM3D_SUBCOMMAND_ERROR;
extern IFM3D_DEVICE_EXPORT const int IFM3D_IO_ERROR;
extern IFM3D_DEVICE_EXPORT const int IFM3D_THREAD_INTERRUPTED;
extern IFM3D_DEVICE_EXPORT const int IFM3D_PCIC_BAD_REPLY;
extern IFM3D_DEVICE_EXPORT const int IFM3D_UNSUPPORTED_OP;
extern IFM3D_DEVICE_EXPORT const int IFM3D_IMG_CHUNK_NOT_FOUND;
extern IFM3D_DEVICE_EXPORT const int IFM3D_PIXEL_FORMAT_ERROR;
extern IFM3D_DEVICE_EXPORT const int IFM3D_UNSUPPORTED_DEVICE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_RECOVERY_CONNECTION_ERROR;
extern IFM3D_DEVICE_EXPORT const int IFM3D_UPDATE_ERROR;
extern IFM3D_DEVICE_EXPORT const int IFM3D_PCICCLIENT_UNSUPPORTED_DEVICE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_HEADER_VERSION_MISMATCH;
extern IFM3D_DEVICE_EXPORT const int
  IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE;
extern IFM3D_DEVICE_EXPORT const int
  IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE;
extern IFM3D_DEVICE_EXPORT const int
  IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE;
extern IFM3D_DEVICE_EXPORT const int
  IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_CURL_ERROR;
extern IFM3D_DEVICE_EXPORT const int IFM3D_CURL_TIMEOUT;
extern IFM3D_DEVICE_EXPORT const int IFM3D_CURL_ABORTED;
extern IFM3D_DEVICE_EXPORT const int IFM3D_SWUPDATE_BAD_STATE;
extern IFM3D_DEVICE_EXPORT const int
  IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED;
extern IFM3D_DEVICE_EXPORT const int
  IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_DEVICE;
extern IFM3D_DEVICE_EXPORT const int
  IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_FIRMWARE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_PORT;
extern IFM3D_DEVICE_EXPORT const int IFM3D_TOOL_COMMAND_UNSUPPORTED_DEVICE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_UNSUPPORTED_SCHEMA_ON_DEVICE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_BUFFER_ID_NOT_AVAILABLE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_NETWORK_ERROR;
extern IFM3D_DEVICE_EXPORT const int IFM3D_SYSTEM_ERROR;
extern IFM3D_DEVICE_EXPORT const int IFM3D_CORRUPTED_STRUCT;
extern IFM3D_DEVICE_EXPORT const int
  IFM3D_DEVICE_PORT_INCOMPATIBLE_WITH_ORGANIZER;
extern IFM3D_DEVICE_EXPORT const int IFM3D_DEVICE_PORT_NOT_SUPPORTED;
// sensor errors
extern IFM3D_DEVICE_EXPORT const int IFM3D_XMLRPC_OBJ_NOT_FOUND;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_PARAM;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_VALUE_TYPE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_VALUE_OUT_OF_RANGE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_READONLY_PARAM;
extern IFM3D_DEVICE_EXPORT const int IFM3D_SESSION_ALREADY_ACTIVE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_PASSWORD;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_SESSIONID;
extern IFM3D_DEVICE_EXPORT const int IFM3D_COULD_NOT_REBOOT;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_FORMAT;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_DEVICE_TYPE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_IMPORT_FLAGS;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_APP_INDEX;
extern IFM3D_DEVICE_EXPORT const int IFM3D_APP_IN_EDIT_MODE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_MAX_APP_LIMIT;
extern IFM3D_DEVICE_EXPORT const int IFM3D_NO_APP_IN_EDIT_MODE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_CANNOT_SW_TRIGGER;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_IMAGER_TYPE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_UNSUPPORTED_APP_TYPE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_PIN_ALREADY_IN_USE;
extern IFM3D_DEVICE_EXPORT const int IFM3D_NO_SUCH_MODEL_OR_ROI;
extern IFM3D_DEVICE_EXPORT const int IFM3D_TEMPORAL_FILTER_TRIGGER_CONFLICT;
extern IFM3D_DEVICE_EXPORT const int IFM3D_EEPROM_FAIL;
extern IFM3D_DEVICE_EXPORT const int IFM3D_IMPORT_EXPORT_IN_PROGRESS;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_NET_CONFIG;
extern IFM3D_DEVICE_EXPORT const int IFM3D_LED_DUTY_CYCLE_VIOLATION;
extern IFM3D_DEVICE_EXPORT const int IFM3D_AUTO_EXPOSURE_NOT_SUPPORTED;
extern IFM3D_DEVICE_EXPORT const int IFM3D_INVALID_FIRMWARE_VERSION;
extern IFM3D_DEVICE_EXPORT const int IFM3D_PROXY_AUTH_REQUIRED;
extern IFM3D_DEVICE_EXPORT const int IFM3D_PIXEL_FORMAT_NOT_SUPPORTED;
/** @}*/
/** @}*/

namespace ifm3d
{
  /**
   * @brief Human consumable string for an IFM3D error
   *
   * @param[in] errnum The error number to translate to a string
   * @return A stringified version of the error
   */
  IFM3D_DEVICE_EXPORT const char* strerror(int errnum);

  /** @ingroup Device
   *
   * Exception wrapper for library and system errors encountered by the
   * library.
   */
  class IFM3D_DEVICE_EXPORT Error : public std::exception
  {
  public:
    /**
     * The ctor simply sets the error value and optional message into a local
     * instance variables that may be retrieved with a call to @ref code() and
     * @ref message().
     */
    Error(int errnum, const std::string& msg = "");

    /**
     * Exception message
     */
    virtual const char* what() const noexcept;

    /**
     * Accessor to the underlying error code
     */
    int code() const noexcept;

    /**
     * Accessor to the underlying error msg
     */
    const char* message() const noexcept;

  private:
    /**
     * The error code from the sensor/system/library that this exception
     * wraps.
     */
    int errnum_;

    /**
     * Optional error message to pass additional details about the error
     */
    std::string errmsg_;

    /**
     * String representation of the error including the error code and
     * optionally the message
     */
    std::string what_;

  }; // end: class Error

} // end: namespace ifm3d

#endif // IFM3D_DEVICE_ERR_H
