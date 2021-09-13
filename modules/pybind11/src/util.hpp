/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PY_UTIL_HPP
#define IFM3D_PY_UTIL_HPP

#include <stdexcept>
#include <ifm3d/stlimage/image.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace ifm3d
{
  template <typename T>
  py::array_t<T>
  image_to_array_2d(const ifm3d::Image& img)
  {
    // Alloc a new ifm3d::Image_<T> and tie its lifecycle to the Python object
    // via a capsule. The resulting numpy.ndarray will not own the memory, but
    // the memory will remain valid for the lifecycle of the object.
    auto mat = new ifm3d::Image_<T>(img);
    auto capsule = py::capsule(mat, [](void* m) {
      delete reinterpret_cast<ifm3d::Image_<T>*>(m);
    });

    return py::array_t<T>({mat->height(), mat->width()},
                          {sizeof(T) * mat->width(), sizeof(T)},
                          reinterpret_cast<T*>(mat->ptr(0)),
                          capsule);
  }

  template <typename T>
  py::array_t<T>
  image_to_array_nd(const ifm3d::Image& cld)
  {
    // Alloc a new ifm3d::Image_<T> and tie its lifecycle to the Python object
    // via a capsule. The resulting numpy.ndarray will not own the memory, but
    // the memory will remain valid for the lifecycle of the object.
    auto mat = new ifm3d::Image_<ifm3d::Point3D<T>>(cld);
    auto capsule = py::capsule(mat, [](void* m) {
      delete reinterpret_cast<ifm3d::Image_<ifm3d::Point3D<T>>*>(m);
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
  image_to_array_(const ifm3d::Image& img)
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
  image_to_array(const ifm3d::Image& img)
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
}

#endif // IFM3D_PY_UTIL_HPP
