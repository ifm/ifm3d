/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_ERROR
#define IFM3D_PYBIND_BINDING_ERROR

#include "../util.hpp"

#include <pybind11/pybind11.h>
#include <string>

using namespace pybind11::literals;

void
bind_error(pybind11::module_& m)
{
  py::options options;
  options.disable_function_signatures();

  // pybind doesn't support custom exception classes with additional parameters
  // so we create a new exception class dynamically using the python type()
  // function.
  py::object pyerror = py::module::import("builtins").attr("RuntimeError");
  py::object pyproperty = py::module::import("builtins").attr("property");
  py::object pytype =
    py::reinterpret_borrow<py::object>((PyObject*)&PyType_Type);
  py::dict attributes;

  auto code_getter = py::cpp_function(
    [](py::object self) -> py::int_ { return self.attr("_code"); });

  auto message_getter = py::cpp_function(
    [](py::object self) -> py::str { return self.attr("_message"); });

  auto what_getter = py::cpp_function(
    [](py::object self) -> py::str { return self.attr("_what"); });

  py::dict error_attributes(
    "__init__"_a = py::cpp_function(
      [](py::object self,
         int errnum,
         const std::string& msg,
         const std::string& what) {
        self.attr("_code") = errnum;
        self.attr("_message") = msg;
        self.attr("_what") = what;
      },
      py::arg("errnum"),
      py::arg("msg"),
      py::arg("what"),
      py::is_method(py::none()),
      py::doc(R"(
        __init__(self, code: int, msg: str, what: str) -> None


        Create a Error with the given code, message and what.
      )")),
    "_get_code"_a = code_getter,
    "_get_message"_a = message_getter,
    "_get_what"_a = what_getter,
    "code"_a =
      pyproperty(code_getter, py::none(), py::none(), R"(Error Code)"),
    "message"_a = pyproperty(message_getter,
                             py::none(),
                             py::none(),
                             R"(Exception message)"),
    "what"_a = pyproperty(
      what_getter,
      py::none(),
      py::none(),
      R"(String representation of the error including the error code and optionally the message)"),
    "__str__"_a =
      py::cpp_function([](py::object self) { return self.attr("what"); },
                       py::is_method(py::none())),
    "__module__"_a = m.attr("__name__").cast<std::string>().c_str());

  static auto error_class =
    pytype("Error", py::make_tuple(pyerror), error_attributes);
  m.attr("Error") = error_class;

  error_class.attr("__doc__") = R"(
    Exception wrapper for library and system errors encountered by the library.

    **Error Codes*

    .. csv-table::

  )";

  py::register_local_exception_translator([](std::exception_ptr p) {
    try
      {
        if (p)
          {
            std::rethrow_exception(p);
          }
      }
    catch (const ifm3d::Error& e)
      {
        auto error = error_class(e.code(), e.message(), e.what());
        PyErr_SetObject(error_class.ptr(), error.ptr());
      }
    catch (const py::builtin_exception& e)
      {
        throw;
      }
    catch (const std::exception& e)
      {
        PyErr_SetString(PyExc_RuntimeError, e.what());
      }
  });

  // clang-format off
  ifm3d::add_attr(error_class, "IFM3D_NO_ERRORS", IFM3D_NO_ERRORS);
  ifm3d::add_attr(error_class, "IFM3D_XMLRPC_FAILURE", IFM3D_XMLRPC_FAILURE);
  ifm3d::add_attr(error_class, "IFM3D_XMLRPC_TIMEOUT", IFM3D_XMLRPC_TIMEOUT);
  ifm3d::add_attr(error_class, "IFM3D_JSON_ERROR", IFM3D_JSON_ERROR);
  ifm3d::add_attr(error_class, "IFM3D_NO_ACTIVE_APPLICATION", IFM3D_NO_ACTIVE_APPLICATION);
  ifm3d::add_attr(error_class, "IFM3D_SUBCOMMAND_ERROR", IFM3D_SUBCOMMAND_ERROR);
  ifm3d::add_attr(error_class, "IFM3D_IO_ERROR", IFM3D_IO_ERROR);
  ifm3d::add_attr(error_class, "IFM3D_THREAD_INTERRUPTED", IFM3D_THREAD_INTERRUPTED);
  ifm3d::add_attr(error_class, "IFM3D_PCIC_BAD_REPLY", IFM3D_PCIC_BAD_REPLY);
  ifm3d::add_attr(error_class, "IFM3D_UNSUPPORTED_OP", IFM3D_UNSUPPORTED_OP);
  ifm3d::add_attr(error_class, "IFM3D_IMG_CHUNK_NOT_FOUND", IFM3D_IMG_CHUNK_NOT_FOUND);
  ifm3d::add_attr(error_class, "IFM3D_PIXEL_FORMAT_ERROR", IFM3D_PIXEL_FORMAT_ERROR);
  ifm3d::add_attr(error_class, "IFM3D_UNSUPPORTED_DEVICE", IFM3D_UNSUPPORTED_DEVICE);
  ifm3d::add_attr(error_class, "IFM3D_RECOVERY_CONNECTION_ERROR", IFM3D_RECOVERY_CONNECTION_ERROR);
  ifm3d::add_attr(error_class, "IFM3D_UPDATE_ERROR", IFM3D_UPDATE_ERROR);
  ifm3d::add_attr(error_class, "IFM3D_PCICCLIENT_UNSUPPORTED_DEVICE", IFM3D_PCICCLIENT_UNSUPPORTED_DEVICE);
  ifm3d::add_attr(error_class, "IFM3D_HEADER_VERSION_MISMATCH", IFM3D_HEADER_VERSION_MISMATCH);
  ifm3d::add_attr(error_class, "IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE", IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE);
  ifm3d::add_attr(error_class, "IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE", IFM3D_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE);
  ifm3d::add_attr(error_class, "IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE", IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_DEVICE);
  ifm3d::add_attr(error_class, "IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE", IFM3D_INVERSE_INTRINSIC_CALIBRATION_UNSUPPORTED_FIRMWARE);
  ifm3d::add_attr(error_class, "IFM3D_CURL_ERROR", IFM3D_CURL_ERROR);
  ifm3d::add_attr(error_class, "IFM3D_CURL_TIMEOUT", IFM3D_CURL_TIMEOUT);
  ifm3d::add_attr(error_class, "IFM3D_CURL_ABORTED", IFM3D_CURL_ABORTED);
  ifm3d::add_attr(error_class, "IFM3D_SWUPDATE_BAD_STATE", IFM3D_SWUPDATE_BAD_STATE);
  ifm3d::add_attr(error_class, "IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED", IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED);
  ifm3d::add_attr(error_class, "IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_DEVICE", IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_DEVICE);
  ifm3d::add_attr(error_class, "IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_FIRMWARE", IFM3D_DISTANCE_NOISE_IMAGE_UNSUPPORTED_FIRMWARE);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_PORT", IFM3D_INVALID_PORT);
  ifm3d::add_attr(error_class, "IFM3D_TOOL_COMMAND_UNSUPPORTED_DEVICE", IFM3D_TOOL_COMMAND_UNSUPPORTED_DEVICE);
  ifm3d::add_attr(error_class, "IFM3D_UNSUPPORTED_SCHEMA_ON_DEVICE", IFM3D_UNSUPPORTED_SCHEMA_ON_DEVICE);
  ifm3d::add_attr(error_class, "IFM3D_BUFFER_ID_NOT_AVAILABLE", IFM3D_BUFFER_ID_NOT_AVAILABLE);
  ifm3d::add_attr(error_class, "IFM3D_NETWORK_ERROR", IFM3D_NETWORK_ERROR);
  ifm3d::add_attr(error_class, "IFM3D_SYSTEM_ERROR", IFM3D_SYSTEM_ERROR);
  ifm3d::add_attr(error_class, "IFM3D_CORRUPTED_STRUCT", IFM3D_CORRUPTED_STRUCT);
  ifm3d::add_attr(error_class, "IFM3D_DEVICE_PORT_INCOMPATIBLE_WITH_ORGANIZER", IFM3D_DEVICE_PORT_INCOMPATIBLE_WITH_ORGANIZER);
  ifm3d::add_attr(error_class, "IFM3D_DEVICE_PORT_NOT_SUPPORTED", IFM3D_DEVICE_PORT_NOT_SUPPORTED);
  ifm3d::add_attr(error_class, "IFM3D_XMLRPC_OBJ_NOT_FOUND", IFM3D_XMLRPC_OBJ_NOT_FOUND);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_PARAM", IFM3D_INVALID_PARAM);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_VALUE_TYPE", IFM3D_INVALID_VALUE_TYPE);
  ifm3d::add_attr(error_class, "IFM3D_VALUE_OUT_OF_RANGE", IFM3D_VALUE_OUT_OF_RANGE);
  ifm3d::add_attr(error_class, "IFM3D_READONLY_PARAM", IFM3D_READONLY_PARAM);
  ifm3d::add_attr(error_class, "IFM3D_SESSION_ALREADY_ACTIVE", IFM3D_SESSION_ALREADY_ACTIVE);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_PASSWORD", IFM3D_INVALID_PASSWORD);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_SESSIONID", IFM3D_INVALID_SESSIONID);
  ifm3d::add_attr(error_class, "IFM3D_COULD_NOT_REBOOT", IFM3D_COULD_NOT_REBOOT);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_FORMAT", IFM3D_INVALID_FORMAT);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_DEVICE_TYPE", IFM3D_INVALID_DEVICE_TYPE);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_IMPORT_FLAGS", IFM3D_INVALID_IMPORT_FLAGS);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_APP_INDEX", IFM3D_INVALID_APP_INDEX);
  ifm3d::add_attr(error_class, "IFM3D_APP_IN_EDIT_MODE", IFM3D_APP_IN_EDIT_MODE);
  ifm3d::add_attr(error_class, "IFM3D_MAX_APP_LIMIT", IFM3D_MAX_APP_LIMIT);
  ifm3d::add_attr(error_class, "IFM3D_NO_APP_IN_EDIT_MODE", IFM3D_NO_APP_IN_EDIT_MODE);
  ifm3d::add_attr(error_class, "IFM3D_CANNOT_SW_TRIGGER", IFM3D_CANNOT_SW_TRIGGER);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_IMAGER_TYPE", IFM3D_INVALID_IMAGER_TYPE);
  ifm3d::add_attr(error_class, "IFM3D_UNSUPPORTED_APP_TYPE", IFM3D_UNSUPPORTED_APP_TYPE);
  ifm3d::add_attr(error_class, "IFM3D_PIN_ALREADY_IN_USE", IFM3D_PIN_ALREADY_IN_USE);
  ifm3d::add_attr(error_class, "IFM3D_NO_SUCH_MODEL_OR_ROI", IFM3D_NO_SUCH_MODEL_OR_ROI);
  ifm3d::add_attr(error_class, "IFM3D_TEMPORAL_FILTER_TRIGGER_CONFLICT", IFM3D_TEMPORAL_FILTER_TRIGGER_CONFLICT);
  ifm3d::add_attr(error_class, "IFM3D_EEPROM_FAIL", IFM3D_EEPROM_FAIL);
  ifm3d::add_attr(error_class, "IFM3D_IMPORT_EXPORT_IN_PROGRESS", IFM3D_IMPORT_EXPORT_IN_PROGRESS);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_NET_CONFIG", IFM3D_INVALID_NET_CONFIG);
  ifm3d::add_attr(error_class, "IFM3D_LED_DUTY_CYCLE_VIOLATION", IFM3D_LED_DUTY_CYCLE_VIOLATION);
  ifm3d::add_attr(error_class, "IFM3D_AUTO_EXPOSURE_NOT_SUPPORTED", IFM3D_AUTO_EXPOSURE_NOT_SUPPORTED);
  ifm3d::add_attr(error_class, "IFM3D_INVALID_FIRMWARE_VERSION", IFM3D_INVALID_FIRMWARE_VERSION);
  ifm3d::add_attr(error_class, "IFM3D_PROXY_AUTH_REQUIRED", IFM3D_PROXY_AUTH_REQUIRED);
  ifm3d::add_attr(error_class, "IFM3D_PIXEL_FORMAT_NOT_SUPPORTED", IFM3D_PIXEL_FORMAT_NOT_SUPPORTED);
  // clang-format on
}

#endif // IFM3D_PYBIND_BINDING_ERROR