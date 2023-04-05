/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_ERROR
#define IFM3D_PYBIND_BINDING_ERROR

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
}

#endif // IFM3D_PYBIND_BINDING_ERROR