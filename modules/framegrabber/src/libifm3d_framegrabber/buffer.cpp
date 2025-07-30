/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstddef>
#include "ifm3d/common/json_impl.hpp"
#include "ifm3d/device/device.h"
#include "ifm3d/fg/buffer_id.h"
#include "ifm3d/common/err.h"
#include <cstring>
#include <ifm3d/fg/buffer.h>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <stdexcept>

namespace ifm3d
{
  static std::unordered_map<uint32_t, std::size_t> PIX_SZ{
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
    std::uint8_t* _data{nullptr};
    /* @brief memory allocator */
    std::allocator<std::uint8_t> _data_alloc;
    /*@brief size of current allocation*/
    size_t _size{0};

  public:
    BufferAllocator() = default;
    BufferAllocator(const BufferAllocator&) = default;
    BufferAllocator& operator=(const BufferAllocator&) = default;
    BufferAllocator(BufferAllocator&&) = default;
    BufferAllocator& operator=(BufferAllocator&&) = default;

    ~BufferAllocator()
    {
      if (_data != nullptr)
        {
          Deallocate();
        }
    }

  public:
    std::uint8_t*
    Allocate(size_t size)
    {
      _data = (uint8_t*)_data_alloc.allocate(size);
      if (_data != nullptr)
        {
          _size = size;
          return _data;
        }

      throw std::runtime_error("cannot allocate memory");
    }
    void
    Deallocate()
    {
      _data_alloc.deallocate(_data, _size);
      _data = nullptr;
    }

    uint8_t*
    Data()
    {
      return _data;
    }
  };
}

//--------------------------------
// Image class
//--------------------------------

ifm3d::Buffer::Buffer() = default;

ifm3d::Buffer::Buffer(const std::uint32_t cols,
                      const std::uint32_t rows,
                      const std::uint32_t nchannel,
                      ifm3d::pixel_format format,
                      const std::optional<ifm3d::json>& metadata,
                      ifm3d::buffer_id buffer_id)
  : metadata_(metadata.value_or(ifm3d::json()))
{
  create(cols, rows, nchannel, format, buffer_id);
}

void
ifm3d::Buffer::create(const std::uint32_t cols,
                      const std::uint32_t rows,
                      const std::uint32_t nchannel,
                      ifm3d::pixel_format format,
                      ifm3d::buffer_id buffer_id)
{
  cols_ = cols;
  rows_ = rows;
  nchannel_ = nchannel;
  data_format_ = format;
  bufferId_ = buffer_id;
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
  size_ =
    static_cast<std::size_t>(cols * rows * nchannel_) * data_size_in_bytes_;
  buffer_allocator_ = std::make_shared<ifm3d::Buffer::BufferAllocator>();
  data_ = buffer_allocator_->Allocate(size_);
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

ifm3d::buffer_id
ifm3d::Buffer::bufferId() const
{
  return bufferId_;
}