/*
 * Copyright (C) 2019 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef IFM3D_PY_UTIL_HPP
#define IFM3D_PY_UTIL_HPP

#include <stdexcept>
#include <opencv2/core/core.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace ifm3d
{
  template<typename T>
  py::array_t<T> image_to_array_2d(const cv::Mat& img)
  {
    // Alloc a new cv::Mat_<T> and tie its lifecycle to the Python object
    // via a capsule. The resulting numpy.ndarray will not own the memory, but
    // the memory will remain valid for the lifecycle of the object.
    auto mat = new cv::Mat_<T>(img);
    auto capsule = py::capsule(
      mat,
      [](void *m)
      {
        delete reinterpret_cast<cv::Mat_<cv::Vec<T, 3>>*>(m);
      });

    return py::array_t<T>(
      { mat->rows, mat->cols },
      { sizeof(T) * mat->cols, sizeof(T)},
      reinterpret_cast<T*>(mat->ptr(0)),
      capsule);
  }

  template<typename T>
  py::array_t<T> image_to_array_nd(const cv::Mat& cld)
  {
    // Alloc a new cv::Mat_<T> and tie its lifecycle to the Python object
    // via a capsule. The resulting numpy.ndarray will not own the memory, but
    // the memory will remain valid for the lifecycle of the object.
    auto mat = new cv::Mat_<cv::Vec<T, 3>>(cld);
    auto capsule = py::capsule(
      mat,
      [](void *m)
      {
        delete reinterpret_cast<cv::Mat_<cv::Vec<T, 3>>*>(m);
      });

    return py::array_t<T>(
      { mat->rows, mat->cols, mat->channels() },
      { sizeof(T) * mat->channels() * mat->cols,
        sizeof(T) * mat->channels(),
        sizeof(T)},
      reinterpret_cast<T*>(mat->ptr(0)),
      capsule);
  }

  template<typename T>
  py::array_t<T> image_to_array_(const cv::Mat& img)
  {
    if (img.channels() > 1)
      {
        return image_to_array_nd<T>(img);
      }
    else
      {
        return image_to_array_2d<T>(img);
      }
  }

  py::array image_to_array(const cv::Mat& img)
  {
    switch(img.depth())
    {
      case CV_8U:
        return image_to_array_<std::uint8_t>(img);
        break;
      case CV_8S:
        return image_to_array_<std::int8_t>(img);
        break;
      case CV_16U:
        return image_to_array_<std::uint16_t>(img);
        break;
      case CV_16S:
        return image_to_array_<std::int16_t>(img);
        break;
      case CV_32S:
        return image_to_array_<std::int32_t>(img);
        break;
      case CV_32F:
        return image_to_array_<float>(img);
        break;
      case CV_64F:
        return image_to_array_<double>(img);
        break;
      default:
        throw std::runtime_error(
          "Unsupported CV type: " + img.type());
    }
  }
}

#endif // IFM3D_PY_UTIL_HPP
