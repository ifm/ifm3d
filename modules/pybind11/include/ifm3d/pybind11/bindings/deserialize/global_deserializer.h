/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IFM3D_PYBIND_BINDING_GLOBAL_DESERIALIZER
#define IFM3D_PYBIND_BINDING_GLOBAL_DESERIALIZER

#include <ifm3d/deserialize/deserialize.h>
#include <ifm3d/fg/frame.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

inline void
bind_deserializer(py::module_& m)
{
  m.def(
    "deserialize",
    [](const py::object& in_obj) -> py::object {
      auto in_array = py::cast<py::array>(in_obj);

      auto buffer_id = in_obj.attr("buffer_id").cast<ifm3d::buffer_id>();

      ifm3d::VariantType result =
        ifm3d::deserialize(reinterpret_cast<const uint8_t*>(in_array.data(0)),
                           in_array.nbytes(),
                           buffer_id);

      return std::visit(
        [](auto&& val) -> py::object {
          using T = std::decay_t<decltype(val)>;
          if constexpr (std::is_same_v<T, std::monostate>)
            {
              return py::none();
            }
          else
            {
              return py::cast(val);
            }
        },
        result);
    },
    R"(
        Deserialize the given buffer and return the appropriate data structure.
        Returns None if the buffer could not be interpreted.
    )");
}

#endif // IFM3D_PYBIND_BINDING_GLOBAL_DESERIALIZER
