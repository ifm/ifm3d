// -*- c++ -*-
/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_IMAGE_IMAGE_BUFFER_IMPL_HPP
#define IFM3D_IMAGE_IMAGE_BUFFER_IMPL_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <glog/logging.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/stlimage/image.h>

namespace ifm3d
{
  //============================================================
  // Impl interface
  //============================================================
  class StlImageBuffer::Impl
  {
  public:
    Impl();
    ~Impl() = default;

    ifm3d::Image DistanceImage();
    ifm3d::Image UnitVectors();
    ifm3d::Image GrayImage();
    ifm3d::Image AmplitudeImage();
    ifm3d::Image RawAmplitudeImage();
    ifm3d::Image ConfidenceImage();
    ifm3d::Image XYZImage();

    void ImCreate(ifm3d::image_chunk im,
                  std::uint32_t fmt,
                  std::size_t idx,
                  std::uint32_t width,
                  std::uint32_t height,
                  int nchan,
                  std::uint32_t npts,
                  const std::vector<std::uint8_t>& bytes);

    void CloudCreate(std::uint32_t fmt,
                     std::size_t xidx,
                     std::size_t yidx,
                     std::size_t zidx,
                     std::uint32_t width,
                     std::uint32_t height,
                     std::uint32_t npts,
                     const std::vector<std::uint8_t>& bytes);

  protected:
    ifm3d::Image dist_;
    ifm3d::Image uvec_;
    ifm3d::Image gray_;
    ifm3d::Image amp_;
    ifm3d::Image ramp_;
    ifm3d::Image conf_;
    ifm3d::Image xyz_;
    ifm3d::Image bad_; // mask of bad pixels

    template <typename T>
    void
    _ImCreate(ifm3d::Image& im,
              ifm3d::image_chunk chunk,
              std::uint32_t fmt,
              std::size_t idx,
              std::uint32_t width,
              std::uint32_t height,
              int nchan,
              std::uint32_t npts,
              const std::vector<std::uint8_t>& bytes)
    {
      std::size_t incr = sizeof(T) * nchan;
      im.Create(width, height, nchan, static_cast<ifm3d::pixel_format>(fmt));
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
              ptr = im.ptr<T>(row);
            }

          if (nchan == 3)
            {
              ptr[col3] = ifm3d::mkval<T>(bytes.data() + idx);
              ptr[col3 + 1] = ifm3d::mkval<T>(bytes.data() + idx + sizeof(T));
              ptr[col3 + 2] =
                ifm3d::mkval<T>(bytes.data() + idx + (sizeof(T) * 2));
            }
          else
            {
              ptr[col] = ifm3d::mkval<T>(bytes.data() + idx);
            }
        }
      //
      // There are only certain image types we want to flag bad
      // pixels for. For example we do *not* want to flag the confidence image
      // or the unit vectors. To that end, we do not do this in-line above
      // with a bunch of nasty if-statements for each pixel. Instead,
      // we take another pass over the image. This vectorized approach
      // should be fast/cache-friendly enough to not have us introduce too
      // much additional latency.
      //
      // constexpr T bad_pixel =
      //   std::numeric_limits<T>::has_quiet_NaN
      //   ? std::numeric_limits<T>::quiet_NaN() : 0;
      constexpr T bad_pixel = 0;

      if ((chunk != ifm3d::image_chunk::CONFIDENCE) &&
          (chunk != ifm3d::image_chunk::UNIT_VECTOR_ALL))
        {
          im.setTo(bad_pixel, this->bad_);
        }
    }

    template <typename T>
    void
    _CloudCreate(ifm3d::Image& im,
                 std::uint32_t fmt,
                 std::size_t xidx,
                 std::size_t yidx,
                 std::size_t zidx,
                 std::uint32_t width,
                 std::uint32_t height,
                 std::uint32_t npts,
                 const std::vector<std::uint8_t>& bytes)
    {
      std::size_t incr = sizeof(T);
      im.Create(width,
                height,
                3,
                static_cast<ifm3d::pixel_format>(fmt));

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
              xyz_ptr = im.ptr<T>(row);
              bad_ptr = this->bad_.ptr<uint8_t>(row);
            }

          // convert to ifm3d coord frame
          x_ = ifm3d::mkval<T>(bytes.data() + zidx);
          y_ = -ifm3d::mkval<T>(bytes.data() + xidx);
          z_ = -ifm3d::mkval<T>(bytes.data() + yidx);
         
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
    };
} // end: namespace ifm3d

  //============================================================
  // Impl -- Implementation Details
  //============================================================

  //-------------------------------------
  // ctor/dtor
  //-------------------------------------
  ifm3d::StlImageBuffer::Impl::Impl()
  { }

  //-------------------------------------
  // Accessors
  //-------------------------------------
  ifm3d::Image
  ifm3d::StlImageBuffer::Impl::DistanceImage()
  {
    return this->dist_;
  }

  ifm3d::Image
  ifm3d::StlImageBuffer::Impl::UnitVectors()
  {
    return this->uvec_;
  }

  ifm3d::Image
  ifm3d::StlImageBuffer::Impl::GrayImage()
  {
    return this->gray_;
  }

  ifm3d::Image
  ifm3d::StlImageBuffer::Impl::AmplitudeImage()
  {
    return this->amp_;
  }

  ifm3d::Image
  ifm3d::StlImageBuffer::Impl::RawAmplitudeImage()
  {
    return this->ramp_;
  }

  ifm3d::Image
  ifm3d::StlImageBuffer::Impl::ConfidenceImage()
  {
    return this->conf_;
  }

  ifm3d::Image
  ifm3d::StlImageBuffer::Impl::XYZImage()
  {
    return this->xyz_;
  }

  void
  ifm3d::StlImageBuffer::Impl::ImCreate(ifm3d::image_chunk im,
                                        std::uint32_t fmt,
                                        std::size_t idx,
                                        std::uint32_t width,
                                        std::uint32_t height,
                                        int nchan,
                                        std::uint32_t npts,
                                        const std::vector<std::uint8_t>& bytes)
  {
    ifm3d::Image* image;
    switch (im)
      {
      case ifm3d::image_chunk::CONFIDENCE:
        image = &this->conf_;
        break;

      case ifm3d::image_chunk::AMPLITUDE:
        image = &this->amp_;
        break;

      case ifm3d::image_chunk::RADIAL_DISTANCE:
        image = &this->dist_;
        break;

      case ifm3d::image_chunk::UNIT_VECTOR_ALL:
        image = &this->uvec_;
        break;

      case ifm3d::image_chunk::RAW_AMPLITUDE:
        image = &this->ramp_;
        break;

      case ifm3d::image_chunk::GRAY:
        image = &this->gray_;
        break;

      default:
        return;
      }
    switch (fmt)
      {
      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U):
        this->_ImCreate<std::uint8_t>(*image,
                                      im,
                                      fmt,
                                      idx,
                                      width,
                                      height,
                                      1,
                                      npts,
                                      bytes);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S):
        this->_ImCreate<std::int8_t>(*image,
                                     im,
                                     fmt,
                                     idx,
                                     width,
                                     height,
                                     1,
                                     npts,
                                     bytes);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U):
        this->_ImCreate<std::uint16_t>(*image,
                                       im,
                                       fmt,
                                       idx,
                                       width,
                                       height,
                                       1,
                                       npts,
                                       bytes);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
        this->_ImCreate<std::int16_t>(*image,
                                      im,
                                      fmt,
                                      idx,
                                      width,
                                      height,
                                      1,
                                      npts,
                                      bytes);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S):
        this->_ImCreate<std::int32_t>(*image,
                                      im,
                                      fmt,
                                      idx,
                                      width,
                                      height,
                                      1,
                                      npts,
                                      bytes);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
        this->_ImCreate<float>(*image,
                               im,
                               fmt,
                               idx,
                               width,
                               height,
                               1,
                               npts,
                               bytes);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3):
        this->_ImCreate<float>(*image,
                               im,
                               fmt,
                               idx,
                               width,
                               height,
                               3,
                               npts,
                               bytes);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F):
        this->_ImCreate<double>(*image,
                                im,
                                fmt,
                                idx,
                                width,
                                height,
                                1,
                                npts,
                                bytes);
        break;

      default:
        LOG(ERROR) << "Unknown image pixel format: " << fmt;
        throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
      }

    // update the bad pixel mask if we just saw the confidence image
    if (im == ifm3d::image_chunk::CONFIDENCE)
      {
        this->bad_.Create(this->conf_.Width(),
                          this->conf_.Height(),
                          1,
                          this->conf_.DataFormat());
        int index = 0;
        auto it = this->bad_.begin<unsigned char>();
        for (unsigned char value : ifm3d::IteratorAdapter<unsigned char>(this->conf_))
          {
            *it = value & 0x1;
            it++;
          }
      }
  }

  void
  ifm3d::StlImageBuffer::Impl::CloudCreate(
    std::uint32_t fmt,
    std::size_t xidx,
    std::size_t yidx,
    std::size_t zidx,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t npts,
    const std::vector<std::uint8_t>& bytes)
  {
    switch (fmt)
      {
      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
        this->_CloudCreate<std::int16_t>(this->xyz_,
                                         fmt,
                                         xidx,
                                         yidx,
                                         zidx,
                                         width,
                                         height,
                                         npts,
                                         bytes);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
        this->_CloudCreate<float>(this->xyz_,
                                  fmt,
                                  xidx,
                                  yidx,
                                  zidx,
                                  width,
                                  height,
                                  npts,
                                  bytes);
        break;

      default:
        LOG(ERROR) << "Unknown cloud pixel format: " << fmt;
        throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
      }
  }
#endif // IFM3D_IMAGE_IMAGE_BUFFER_IMPL_HPP
