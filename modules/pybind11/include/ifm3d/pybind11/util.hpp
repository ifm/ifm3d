/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PY_UTIL_HPP
#define IFM3D_PY_UTIL_HPP

#include <ifm3d/common/features.h>
#include <ifm3d/device/device.h>
#if defined(BUILD_MODULE_FRAMEGRABBER)
#  include <ifm3d/fg/buffer.h>
#endif
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <stdexcept>

using namespace pybind11::literals;

namespace py = pybind11;

inline void
bind_numpy(pybind11::module_& m)
{
  py::options options;
  options.disable_function_signatures();
  py::object view_class =
    py::module::import("numpy").attr("ndarray").attr("view");
  py::object parent_class = py::module::import("numpy").attr("ndarray");
  py::object parent_metaclass = py::reinterpret_borrow<py::object>(
    reinterpret_cast<PyObject*>(&PyType_Type))(parent_class);
  py::dict attributes;

  py::object wrapper_class = parent_metaclass("ifm3d_ndarray",
                                              py::make_tuple(parent_class),
                                              attributes);

  wrapper_class.attr("__new__") = py::cpp_function(
    [parent_class, view_class](py::object self,
                               const py::array& data,
                               const std::optional<py::dict>& metadata,
                               const std::optional<py::object>& buffer_id,
                               const py::args& /*args*/,
                               const py::kwargs& /*kwargs*/

    ) {
      auto obj = view_class(data, self);
      obj.attr("metadata") = metadata.value_or(py::dict());
      if (buffer_id.has_value())
        {
          obj.attr("buffer_id") = *buffer_id;
        }
      return obj;
    },
    py::is_method(wrapper_class),
    py::doc(R"(
        __new__(self, data: ndarray, metadata: dict, buffer_id: Any) -> ndarray
        Create a buffer as numpy.ndarray with metadata and buffer_id.
      )"));

  attributes["__array_finalize__"] = py::cpp_function(
    [](const py::object& self, const py::object& obj) {
      if (obj == Py_None)
        {
          return;
        }
      if (py::hasattr(obj, "metadata"))
        {
          self.attr("metadata") = obj.attr("metadata");
        }
      if (py::hasattr(obj, "buffer_id"))
        {
          self.attr("buffer_id") = obj.attr("buffer_id");
        }
    },
    py::is_method(wrapper_class));
  m.attr("buffer") = wrapper_class;
};

namespace ifm3d
{
#if defined(BUILD_MODULE_FRAMEGRABBER)
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

    return py::array_t<T>({mat->Height(), mat->Width()},
                          {sizeof(T) * mat->Width(), sizeof(T)},
                          reinterpret_cast<T*>(mat->Ptr(0)),
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

    return py::array_t<T>({mat->Height(), mat->Width(), mat->Nchannels()},
                          {sizeof(T) * mat->Nchannels() * mat->Width(),
                           sizeof(T) * mat->Nchannels(),
                           sizeof(T)},
                          reinterpret_cast<T*>(mat->Ptr(0)),
                          capsule);
  }

  template <typename T>
  py::array_t<T>
  image_to_array(const ifm3d::Buffer& img)
  {
    if (img.NumChannels() > 1)
      {
        return image_to_array_nd<T>(img);
      }

    return image_to_array_2d<T>(img);
  }

  inline py::array
  image_to_array(const ifm3d::Buffer& img)
  {
    switch (img.DataFormat())
      {
      case ifm3d::PixelFormat::FORMAT_8U:
        return image_to_array<std::uint8_t>(img);
        break;
      case ifm3d::PixelFormat::FORMAT_8S:
        return image_to_array<std::int8_t>(img);
        break;
      case ifm3d::PixelFormat::FORMAT_16U:
      case ifm3d::PixelFormat::FORMAT_16U2:
        return image_to_array<std::uint16_t>(img);
        break;
      case ifm3d::PixelFormat::FORMAT_16S:
        return image_to_array<std::int16_t>(img);
        break;
      case ifm3d::PixelFormat::FORMAT_32U:
        return image_to_array<std::uint32_t>(img);
        break;
      case ifm3d::PixelFormat::FORMAT_32S:
        return image_to_array<std::int32_t>(img);
        break;
      case ifm3d::PixelFormat::FORMAT_32F:
      case ifm3d::PixelFormat::FORMAT_32F3:
        return image_to_array<float>(img);
        break;
      case ifm3d::PixelFormat::FORMAT_64F:
        return image_to_array<double>(img);
        break;
      default:
        throw std::runtime_error("Unsupported ifm3d::image type");
      }
  }
#endif

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
