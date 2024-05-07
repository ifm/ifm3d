/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PY_UTIL_HPP
#define IFM3D_PY_UTIL_HPP

#include <stdexcept>
#include <ifm3d/fg/buffer.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

using namespace pybind11::literals;

namespace py = pybind11;

void
bind_numpy(pybind11::module_& m)
{
  py::options options;
  options.disable_function_signatures();
  py::object view_class =
    py::module::import("numpy").attr("ndarray").attr("view");
  py::object parent_class = py::module::import("numpy").attr("ndarray");
  py::object parent_metaclass =
    py::reinterpret_borrow<py::object>((PyObject*)&PyType_Type)(parent_class);
  py::dict attributes;

  py::object wrapper_class = parent_metaclass("ifm3d_ndarray",
                                              py::make_tuple(parent_class),
                                              attributes);

  wrapper_class.attr("__new__") = py::cpp_function(
    [parent_class, view_class](py::object self,
                               const py::array& data,
                               const std::optional<py::dict>& metadata,
                               py::args args,
                               py::kwargs kwargs

    ) {
      auto obj = view_class(data, self);
      obj.attr("metadata") = metadata;

      return obj;
    },
    //  py::arg("data"),
    //  py::arg("metadada"),
    py::is_method(wrapper_class),
    py::doc(R"(
        __new__(self, data: ndarray, metada: dict) -> ndarray
        Create a buffer as numpy.ndarray with metadata.
      )"));

  attributes["__array_finalize__"] = py::cpp_function(
    [](py::object self, py::object obj) {
      if (obj == Py_None)
        {
          return;
        }
      self.attr("metadata") = obj.attr("metadata");
    },
    py::is_method(wrapper_class));
  m.attr("buffer") = wrapper_class;
};

namespace ifm3d
{
  template <typename T>
  py::array_t<T>
  image_to_array_2d(const ifm3d::Buffer& img)
  {
    // Alloc a new ifm3d::Buffer_<T> and tie its lifecycle to the Python object
    // via a capsule. The resulting numpy.ndarray will not own the memory, but
    // the memory will remain valid for the lifecycle of the object.
    auto mat = new ifm3d::Buffer_<T>(img);
    auto capsule = py::capsule(mat, [](void* m) {
      delete reinterpret_cast<ifm3d::Buffer_<T>*>(m);
    });

    return py::array_t<T>({mat->height(), mat->width()},
                          {sizeof(T) * mat->width(), sizeof(T)},
                          reinterpret_cast<T*>(mat->ptr(0)),
                          capsule);
  }

  template <typename T>
  py::array_t<T>
  image_to_array_nd(const ifm3d::Buffer& cld)
  {
    // Alloc a new ifm3d::Buffer_<T> and tie its lifecycle to the Python object
    // via a capsule. The resulting numpy.ndarray will not own the memory, but
    // the memory will remain valid for the lifecycle of the object.
    auto mat = new ifm3d::Buffer_<ifm3d::Point3D<T>>(cld);
    auto capsule = py::capsule(mat, [](void* m) {
      delete reinterpret_cast<ifm3d::Buffer_<ifm3d::Point3D<T>>*>(m);
    });

    return py::array_t<T>({mat->height(), mat->width(), mat->nchannels()},
                          {sizeof(T) * mat->nchannels() * mat->width(),
                           sizeof(T) * mat->nchannels(),
                           sizeof(T)},
                          reinterpret_cast<T*>(mat->ptr(0)),
                          capsule);
  }

  template <typename T>
  py::array_t<T>
  image_to_array_(const ifm3d::Buffer& img)
  {
    if (img.nchannels() > 1)
      {
        return image_to_array_nd<T>(img);
      }
    else
      {
        return image_to_array_2d<T>(img);
      }
  }

  py::array
  image_to_array(const ifm3d::Buffer& img)
  {
    switch (img.dataFormat())
      {
      case ifm3d::pixel_format::FORMAT_8U:
        return image_to_array_<std::uint8_t>(img);
        break;
      case ifm3d::pixel_format::FORMAT_8S:
        return image_to_array_<std::int8_t>(img);
        break;
      case ifm3d::pixel_format::FORMAT_16U:
        return image_to_array_<std::uint16_t>(img);
        break;
      case ifm3d::pixel_format::FORMAT_16S:
        return image_to_array_<std::int16_t>(img);
        break;
      case ifm3d::pixel_format::FORMAT_32S:
        return image_to_array_<std::int32_t>(img);
        break;
      case ifm3d::pixel_format::FORMAT_32F:
        return image_to_array_<float>(img);
        break;
      case ifm3d::pixel_format::FORMAT_64F:
        return image_to_array_<double>(img);
        break;
      default:
        throw std::runtime_error("Unsupported ifm3d::image type");
      }
  }

  template <typename T>
  void
  add_attr(pybind11::object& o,
           const std::string& name,
           const T& value,
           const std::string& doc = "")
  {
    o.attr(name.c_str()) = value;
    o.doc() =
      o.doc().cast<std::string>() + "     \"" + name + "\", \"" + doc + "\"\n";
  }
}

#endif // IFM3D_PY_UTIL_HPP
