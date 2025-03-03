/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_PYBIND_BINDING_GLOBAL_DESERIALIZER
#define IFM3D_PYBIND_BINDING_GLOBAL_DESERIALIZER

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ifm3d/deserialize/deserialize.h>

void
bind_deserializer(py::module_& m)
{
  // clang-format off

  m.def("deserialize",
    [](py::object in_obj) -> py::object {
      py::array in_array = py::cast<py::array>(in_obj);

      auto bufferId = in_obj.attr("buffer_id");
      ifm3d::buffer_id buffer_id = bufferId.cast<ifm3d::buffer_id>();

      ifm3d::VariantType result = ifm3d::Deserialize(reinterpret_cast<const uint8_t*>(in_array.data(0)), in_array.nbytes(), buffer_id);

      return std::visit([](auto&& val) -> py::object {
        using T = std::decay_t<decltype(val)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
          return py::none();
        } else {
          return py::cast(val);
        }
      }, result);
    },
    R"(
        Deserialize the given buffer and return the appropriate data structure.
        Returns None if the buffer could not be interpreted.
    )");
}
// clang-format on

#endif // IFM3D_PYBIND_BINDING_GLOBAL_DESERIALIZER
