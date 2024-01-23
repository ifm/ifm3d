/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/fg/buffer.h>
#include <ifm3d/device/err.h>
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

  class ifm3d::Buffer::BufferAllocator
  {
    /* @ brief raw pointer to the data*/
    std::uint8_t* data_;
    /* @brief memory allocator */
    std::allocator<std::uint8_t> data_alloc_;
    /*@brief size of current allocation*/
    size_t size_;

  public:
    BufferAllocator() : data_(nullptr), size_(0) {}

    ~BufferAllocator()
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

ifm3d::Buffer::Buffer()
  : data_(nullptr),
    rows_(0),
    cols_(0),
    nchannel_(0),
    data_size_in_bytes_(0),
    size_(0),
    bytes_per_pixel(0),
    bytes_per_row(0),
    metadata_(ifm3d::json())
{}

ifm3d::Buffer::Buffer(const std::uint32_t cols,
                      const std::uint32_t rows,
                      const std::uint32_t nchannel,
                      ifm3d::pixel_format format,
                      std::optional<ifm3d::json> metadata)
  : metadata_(metadata.value_or(ifm3d::json()))
{
  create(cols, rows, nchannel, format);
}

void
ifm3d::Buffer::create(const std::uint32_t cols,
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
      throw ifm3d::Error(IFM3D_PIXEL_FORMAT_NOT_SUPPORTED);
    }
  bytes_per_pixel = data_size_in_bytes_ * nchannel;
  bytes_per_row = bytes_per_pixel * cols_;
  size_ = cols * rows * nchannel_ * data_size_in_bytes_;
  buffer_allocator_ = std::make_shared<ifm3d::Buffer::BufferAllocator>();
  data_ = buffer_allocator_->allocate(size_);
}

ifm3d::Buffer
ifm3d::Buffer::clone() const
{
  Buffer copy(cols_, rows_, nchannel_, data_format_, metadata_);
  std::memcpy(copy.ptr(0), data_, size_);
  return copy;
}

std::uint32_t
ifm3d::Buffer::height() const
{
  return rows_;
}

std::uint32_t
ifm3d::Buffer::width() const
{
  return cols_;
}

std::uint32_t
ifm3d::Buffer::nchannels() const
{
  return nchannel_;
}

ifm3d::pixel_format
ifm3d::Buffer::dataFormat() const
{
  return data_format_;
}

size_t
ifm3d::Buffer::size() const
{
  return size_;
}

ifm3d::json
ifm3d::Buffer::metadata() const
{
  return metadata_;
}