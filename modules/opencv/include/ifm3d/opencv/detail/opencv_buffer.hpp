// -*- c++ -*-
/*
 * Copyright (C) 2018 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IFM3D_OPENCV_DETAIL_OPENCV_BUFFER_HPP__
#define __IFM3D_OPENCV_DETAIL_OPENCV_BUFFER_HPP__

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>
#include <opencv2/core/core.hpp>
#include <ifm3d/fg/byte_buffer.h>

namespace ifm3d
{
  //
  // This stuff is repeated from the image module, however we add the
  // "_" suffix to each in the event both the `image` and `opencv` modules are
  // installed.
  //

  static std::unordered_map<std::uint32_t, int> PIX_LUT_
  {
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U), CV_8U},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S), CV_8S},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U), CV_16U},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S), CV_16S},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S), CV_32S},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), CV_32F},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3), CV_32F},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F), CV_64F}
  };

  static std::unordered_map<std::uint32_t, int> PIX_LUT3_
  {
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U), CV_8UC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S), CV_8SC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U), CV_16UC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S), CV_16SC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S), CV_32SC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), CV_32FC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3), CV_32FC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F), CV_64FC3}
  };

} // end: namespace ifm3d

//==============================================
// ifm3d::OpenCVBuffer implementation
//==============================================

inline ifm3d::OpenCVBuffer::OpenCVBuffer()
  : ifm3d::ByteBuffer<ifm3d::OpenCVBuffer>()
{ }

inline ifm3d::OpenCVBuffer::~OpenCVBuffer() = default;

// move ctor
inline ifm3d::OpenCVBuffer::OpenCVBuffer(ifm3d::OpenCVBuffer&& src_buff)
  : ifm3d::OpenCVBuffer::OpenCVBuffer()
{
  this->SetBytes(src_buff.bytes_, false);
}

// move assignment
inline ifm3d::OpenCVBuffer&
ifm3d::OpenCVBuffer::operator= (ifm3d::OpenCVBuffer&& src_buff)
{
  this->SetBytes(src_buff.bytes_, false);
  return *this;
}

// copy ctor
inline ifm3d::OpenCVBuffer::OpenCVBuffer(const ifm3d::OpenCVBuffer& src_buff)
  : ifm3d::OpenCVBuffer::OpenCVBuffer()
{
  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
}

// copy assignment
inline ifm3d::OpenCVBuffer&
ifm3d::OpenCVBuffer::operator= (const ifm3d::OpenCVBuffer& src_buff)
{
  if (this == &src_buff)
    {
      return *this;
    }

  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
  return *this;
}

inline cv::Mat
ifm3d::OpenCVBuffer::DistanceImage()
{
  this->Organize();
  return this->dist_;
}

inline cv::Mat
ifm3d::OpenCVBuffer::UnitVectors()
{
  this->Organize();
  return this->uvec_;
}

inline cv::Mat
ifm3d::OpenCVBuffer::GrayImage()
{
  this->Organize();
  return this->gray_;
}

inline cv::Mat
ifm3d::OpenCVBuffer::AmplitudeImage()
{
  this->Organize();
  return this->amp_;
}

inline cv::Mat
ifm3d::OpenCVBuffer::RawAmplitudeImage()
{
  this->Organize();
  return this->ramp_;
}

inline cv::Mat
ifm3d::OpenCVBuffer::ConfidenceImage()
{
  this->Organize();
  return this->conf_;
}

inline cv::Mat
ifm3d::OpenCVBuffer::XYZImage()
{
  this->Organize();
  return this->xyz_;
}

template <typename T>
inline void
ifm3d::OpenCVBuffer::ImCreate(ifm3d::image_chunk im,
                              std::uint32_t fmt,
                              std::size_t idx,
                              std::uint32_t width,
                              std::uint32_t height,
                              int nchan,
                              std::uint32_t npts,
                              const std::vector<std::uint8_t>& bytes)
{
  cv::Mat *mat;
  switch (im)
    {
    case ifm3d::image_chunk::CONFIDENCE:
      mat = &this->conf_;
      break;

    case ifm3d::image_chunk::AMPLITUDE:
      mat = &this->amp_;
      break;

    case ifm3d::image_chunk::RADIAL_DISTANCE:
      mat = &this->dist_;
      break;

    case ifm3d::image_chunk::UNIT_VECTOR_ALL:
      mat = &this->uvec_;
      break;

    case ifm3d::image_chunk::RAW_AMPLITUDE:
      mat = &this->ramp_;
      break;

    case ifm3d::image_chunk::GRAY:
      mat = &this->gray_;
      break;

    default:
      return;
    }

  std::size_t incr = sizeof(T) * nchan;
  if (nchan == 3)
    {
      mat->create(height, width, ifm3d::PIX_LUT3_.at(fmt));
    }
  else
    {
      mat->create(height, width, ifm3d::PIX_LUT_.at(fmt));
    }

  int col = 0;
  int row = -1;
  int col3 = 0;

  T* ptr;

  for (std::size_t i = 0; i < npts; ++i, idx += incr)
    {
      col = i % width;
      col3 = col * 3;

      if (col == 0)
        {
          row += 1;
          ptr = mat->ptr<T>(row);
        }

      if (nchan == 3)
        {
          ptr[col3] = ifm3d::mkval<T>(bytes.data()+idx);
          ptr[col3 + 1] = ifm3d::mkval<T>(bytes.data()+idx+sizeof(T));
          ptr[col3 + 2] = ifm3d::mkval<T>(bytes.data()+idx+(sizeof(T)*2));
        }
      else
        {
          ptr[col] = ifm3d::mkval<T>(bytes.data()+idx);
        }
    }

  // constexpr T bad_pixel =
  //   std::numeric_limits<T>::has_quiet_NaN
  //   ? std::numeric_limits<T>::quiet_NaN() : 0;
  constexpr T bad_pixel = 0;

  if (im == ifm3d::image_chunk::CONFIDENCE)
    {
      cv::bitwise_and(this->conf_, 0x1, this->bad_);
    }
  else if (im != ifm3d::image_chunk::UNIT_VECTOR_ALL)
    {
      mat->setTo(bad_pixel, this->bad_);
    }
}

template <typename T>
inline void
ifm3d::OpenCVBuffer::CloudCreate(std::uint32_t fmt,
                                 std::size_t xidx,
                                 std::size_t yidx,
                                 std::size_t zidx,
                                 std::uint32_t width,
                                 std::uint32_t height,
                                 std::uint32_t npts,
                                 const std::vector<std::uint8_t>& bytes)
{
  std::size_t incr = sizeof(T);
  this->xyz_.create(height, width, ifm3d::PIX_LUT3_.at(fmt));

  int col = 0;
  int row = -1;
  int xyz_col = 0;

  T* xyz_ptr;
  T x_, y_, z_;
  std::uint8_t* bad_ptr;

  // constexpr T bad_pixel =
  //   std::numeric_limits<T>::has_quiet_NaN
  //   ? std::numeric_limits<T>::quiet_NaN() : 0;
  constexpr T bad_pixel = 0;

  for (std::size_t i = 0; i < npts;
       ++i, xidx += incr, yidx += incr, zidx += incr)
    {
      col = i % width;
      xyz_col = col * 3;
      if (col == 0)
        {
          row += 1;
          xyz_ptr = this->xyz_.ptr<T>(row);
          bad_ptr = this->bad_.ptr(row);
        }

      // convert to ifm3d coord frame
      x_ = ifm3d::mkval<T>(bytes.data()+zidx);
      y_ = -ifm3d::mkval<T>(bytes.data()+xidx);
      z_ = -ifm3d::mkval<T>(bytes.data()+yidx);

      if (bad_ptr[col] == 0)
        {
          xyz_ptr[xyz_col] = x_;
          xyz_ptr[xyz_col + 1] = y_;
          xyz_ptr[xyz_col + 2] = z_;
        }
      else
        {
          xyz_ptr[xyz_col] = bad_pixel;
          xyz_ptr[xyz_col + 1] = bad_pixel;
          xyz_ptr[xyz_col + 2] = bad_pixel;
        }
    }
}

#endif // __IFM3D_OPENCV_DETAIL_OPENCV_BUFFER_HPP__
