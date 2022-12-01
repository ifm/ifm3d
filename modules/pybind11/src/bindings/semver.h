/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_SEMVER
#define IFM3D_PYBIND_BINDING_SEMVER

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <sstream>

using namespace pybind11::literals;

void
bind_semver(pybind11::module_& m)
{

  py::class_<ifm3d::SemVer> semver(m,
                                   "SemVer",
                                   R"(
     struct for holding the version information
    )");

  semver
    .def(py::init<size_t,
                  size_t,
                  size_t,
                  const std::optional<std::string>,
                  const std::optional<std::string>>(),
         "major"_a,
         "minor"_a,
         "build"_a,
         "prerelease"_a = std::nullopt,
         "build_meta"_a = std::nullopt)
    .def(py::self < py::self)
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self >= py::self)
    .def(py::self > py::self)
    .def(py::self <= py::self)
    .def("__repr__",
         [](ifm3d::SemVer& self) -> std::string {
           std::ostringstream stream;
           stream << self;
           return stream.str();
         })
    .def_static("Parse", [](const std::string& version_string) {
      return ifm3d::SemVer::Parse(version_string).value();
    });
}

#endif // IFM3D_PYBIND_BINDING_SEMVER