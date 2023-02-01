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
  // pybind doesn't support custom exception classes with additional parameters
  // so we create a new exception class dynamically using the python type()
  // function.
  py::object pyerror = py::module::import("builtins").attr("RuntimeError");
  py::object pytype =
    py::reinterpret_borrow<py::object>((PyObject*)&PyType_Type);
  py::dict attributes;

  py::dict error_attributes(
    "__init__"_a = py::cpp_function(
      [](py::object self,
         int errnum,
         const std::string& msg,
         const std::string& what) {
        self.attr("code") = errnum;
        self.attr("message") = msg;
        self.attr("what") = what;
      },
      py::arg("errnum"),
      py::arg("msg"),
      py::arg("what"),
      py::is_method(py::none())),
    "__str__"_a =
      py::cpp_function([](py::object self) { return self.attr("what"); },
                       py::is_method(py::none())));

  static auto error_class =
    pytype("Error", py::make_tuple(pyerror), error_attributes);
  m.attr("Error") = error_class;

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