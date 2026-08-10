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
#include <memory>
#include <optional>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/typing.h>
#include <stdexcept>
#include <utility>

using namespace pybind11::literals;

namespace py = pybind11;

namespace ifm3d
{
  /// The ``ifm3dpy.Buffer`` type created by bind_numpy(). The reference is
  /// cached on first use and intentionally kept for the lifetime of the
  /// process, the type object is owned by the ifm3dpy module anyway.
  inline py::handle
  py_buffer_type()
  {
    static py::handle type;
    if (!type)
      {
        try
          {
            type = py::object(py::module_::import("ifm3dpy").attr("Buffer"))
                     .release();
          }
        catch (const py::error_already_set&)
          {
            return {};
          }
      }
    return type;
  }

  /// Check function for PyBuffer, true for instances of ``ifm3dpy.Buffer``.
  inline int
  py_buffer_check(PyObject* obj)
  {
    const py::handle type = py_buffer_type();
    if (obj == nullptr || !type)
      {
        return 0;
      }

    const int result = PyObject_IsInstance(obj, type.ptr());
    if (result < 0)
      {
        PyErr_Clear();
        return 0;
      }
    return result;
  }

  /**
   * Wrapper around py::object used purely for the generated type annotations.
   * Functions returning or accepting the numpy.ndarray subclass created by
   * bind_numpy() should use this type so that the signature reads
   * ``ifm3dpy.Buffer`` instead of the useless ``object``.
   */
  class PyBuffer : public py::object
  {
  public:
    PYBIND11_OBJECT_DEFAULT(PyBuffer, py::object, ifm3d::py_buffer_check)
  };

  /// Always true, the wrappers using this exist purely to carry an annotation.
  inline int
  annotation_only_check(PyObject* obj)
  {
    return obj != nullptr;
  }

  /**
   * Non leaking stand in for the ``py::typing`` wrappers which are declared
   * with ``PyObject_Type`` as their check function. ``PyObject_Type`` is not a
   * predicate, it returns a *new* reference to the type of the object which
   * pybind11 then discards, so every conversion leaks one reference to the
   * type of the converted value. Wrap the annotation in this template to keep
   * the rendered signature while using a check function which does not leak.
   *
   * Affects ``Optional``, ``Union``, ``Final``, ``ClassVar``, ``Literal`` and
   * ``TypeVar``. Still present in pybind11 3.0.1 and on upstream master.
   */
  template <typename T>
  class Annotated : public py::object
  {
  public:
    PYBIND11_OBJECT_DEFAULT(Annotated,
                            py::object,
                            ifm3d::annotation_only_check)
  };
}

namespace pybind11
{
  namespace detail
  {
    template <>
    struct handle_type_name<ifm3d::PyBuffer>
    {
      static constexpr auto name = const_name("ifm3dpy.Buffer");
    };

    template <typename T>
    struct handle_type_name<ifm3d::Annotated<T>>
    {
      static constexpr auto name = handle_type_name<T>::name;
    };
  }
}

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

  py::object wrapper_class =
    parent_metaclass("Buffer", py::make_tuple(parent_class), attributes);

  // Without these the type is reported as ``ifm3d_ndarray`` defined in
  // ``importlib._bootstrap``, which prevents Sphinx from documenting and
  // cross referencing it.
  wrapper_class.attr("__module__") = "ifm3dpy";
  wrapper_class.attr("__qualname__") = "Buffer";
  wrapper_class.attr("__doc__") = R"(
      A :class:`numpy.ndarray` subclass carrying the image data of a single
      buffer together with the metadata reported by the device.

      Attributes
      ----------
      metadata : dict
          The metadata associated with this buffer, as reported by the device.

      buffer_id : ifm3dpy.framegrabber.buffer_id
          The id of the buffer this data was received for.
    )";

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
        Create a Buffer as numpy.ndarray with metadata and buffer_id.
      )"));

  // Must be set on the class: ``attributes`` was copied into the type when it
  // was created above, so mutating it here would have no effect and every view
  // or slice of a Buffer would silently lose ``metadata`` and ``buffer_id``.
  wrapper_class.attr("__array_finalize__") = py::cpp_function(
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
  m.attr("Buffer") = wrapper_class;
};

namespace ifm3d
{
  /// Run a cleanup function on an object when the last reference to it is
  /// dropped making sure the GIL is dropped before the cleanup function is
  /// called. This allows the cleanup function to reacquire the GIL without
  /// deadlocking if the last reference is dropped.
  ///
  /// @warning `~T` also runs without the GIL, so every Python object
  /// reachable from `T` must have a GIL-acquiring destructor. Callbacks bound
  /// as `std::function` satisfy this, because pybind11's caster wraps them in
  /// `func_handle`, whose destructor acquires the GIL. A raw `py::object`
  /// captured into a plain lambda does not, and decrementing its refcount
  /// here is undefined behaviour: it corrupts the refcount silently, or
  /// aborts the interpreter outright if the count reaches zero and
  /// `tp_dealloc` runs.
  ///
  /// @param object  Takes ownership
  /// @param cleanup Cleanup function to run on the object when the last
  /// reference is dropped
  template <typename T, typename CleanupFn>
  std::shared_ptr<T>
  with_cleanup(T* object, CleanupFn cleanup)
  {
    return {object, [cleanup = std::move(cleanup)](T* self) {
              std::optional<py::gil_scoped_release> release;
              if (PyGILState_Check())
                {
                  release.emplace();
                }

              try
                {
                  cleanup(self);
                }
              catch (...)
                {
                  // IGNORE: best-effort, the destructor retries the shutdown
                }
              delete self;
            }};
  }

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
