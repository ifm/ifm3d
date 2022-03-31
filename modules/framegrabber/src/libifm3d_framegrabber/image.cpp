/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/fg/image.h>
#include <ifm3d/camera/err.h>
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <limits>
#include <stdexcept>

namespace ifm3d
{
  std::unordered_map<uint32_t, std::size_t> PIX_SZ{
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U), 1},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S), 1},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U), 2},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S), 2},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S), 4},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), 4},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F), 8},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U2), 2},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3), 4}};

  //--------------------------------
  // ImageAllocator class
  //--------------------------------

  class ifm3d::Image::ImageAllocator
  {
    /* @ brief raw pointer to the data*/
    std::uint8_t* data_;
    /* @brief memory allocator */
    std::allocator<std::uint8_t> data_alloc_;
    /*@brief size of current allocation*/
    size_t size_;

  public:
    ImageAllocator() : data_(nullptr), size_(0) {}

    ~ImageAllocator()
    {
      if (data_ != nullptr)
        {
          deallocate();
        }
    }

  public:
    std::uint8_t*
    allocate(size_t size)
    {
      data_ = (uint8_t*)data_alloc_.allocate(size);
      if (data_ != nullptr)
        {
          size_ = size;
          return data_;
        }
      else
        {
          throw std::runtime_error("cannot allocate memory");
        }
    }
    void
    deallocate()
    {
      data_alloc_.deallocate(data_, size_);
      data_ = nullptr;
    }

    uint8_t*
    data()
    {
      return data_;
    }
  };
}

//--------------------------------
// Image class
//--------------------------------

ifm3d::Image::Image()
  : data_(nullptr),
    rows_(0),
    cols_(0),
    nchannel_(0),
    data_size_in_bytes_(0),
    size_(0),
    bytes_per_pixel(0),
    bytes_per_row(0)
{}

ifm3d::Image::Image(const std::uint32_t cols,
                    const std::uint32_t rows,
                    const std::uint32_t nchannel,
                    ifm3d::pixel_format format)
{
  create(cols, rows, nchannel, format);
}

void
ifm3d::Image::create(const std::uint32_t cols,
                     const std::uint32_t rows,
                     const std::uint32_t nchannel,
                     ifm3d::pixel_format format)
{
  cols_ = cols;
  rows_ = rows;
  nchannel_ = nchannel;
  data_format_ = format;
  if (PIX_SZ.find(static_cast<std::uint32_t>(format)) != PIX_SZ.end())
    {
      data_size_in_bytes_ = PIX_SZ[static_cast<std::uint32_t>(format)];
    }
  else
    {
      throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_NOT_SUPPORTED);
    }
  bytes_per_pixel = data_size_in_bytes_ * nchannel;
  bytes_per_row = bytes_per_pixel * cols_;
  size_ = cols * rows * nchannel_ * data_size_in_bytes_;
  image_allocator_ = std::make_shared<ifm3d::Image::ImageAllocator>();
  data_ = image_allocator_->allocate(size_);
}

ifm3d::Image
ifm3d::Image::clone() const
{
  Image copy;
  copy.create(cols_, rows_, nchannel_, data_format_);
  std::memcpy(copy.ptr(0), data_, size_);
  return copy;
}

std::uint32_t
ifm3d::Image::height() const
{
  return rows_;
}

std::uint32_t
ifm3d::Image::width() const
{
  return cols_;
}

std::uint32_t
ifm3d::Image::nchannels() const
{
  return nchannel_;
}

ifm3d::pixel_format
ifm3d::Image::dataFormat() const
{
  return data_format_;
}
