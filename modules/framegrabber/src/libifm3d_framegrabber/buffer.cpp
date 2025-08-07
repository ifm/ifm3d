/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ifm3d/common/err.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/buffer_id.h>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>

namespace
{
  const std::unordered_map<uint32_t, std::size_t> PIX_SZ{
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U), 1},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S), 1},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U), 2},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S), 2},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S), 4},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), 4},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F), 8},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U2), 2},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3), 4}};
}

namespace ifm3d
{

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
  : _metadata(metadata.value_or(ifm3d::json()))
{
  Create(cols, rows, nchannel, format, buffer_id);
}

void
ifm3d::Buffer::Create(const std::uint32_t cols,
                      const std::uint32_t rows,
                      const std::uint32_t nchannel,
                      ifm3d::pixel_format format,
                      ifm3d::buffer_id buffer_id)
{
  _cols = cols;
  _rows = rows;
  _nchannel = nchannel;
  _data_format = format;
  _buffer_id = buffer_id;
  if (PIX_SZ.find(static_cast<std::uint32_t>(format)) != PIX_SZ.end())
    {
      _data_size_in_bytes = PIX_SZ.at(static_cast<std::uint32_t>(format));
    }
  else
    {
      throw ifm3d::Error(IFM3D_PIXEL_FORMAT_NOT_SUPPORTED);
    }
  _bytes_per_pixel = _data_size_in_bytes * nchannel;
  _bytes_per_row = _bytes_per_pixel * _cols;
  _size =
    static_cast<std::size_t>(cols * rows * _nchannel) * _data_size_in_bytes;
  _buffer_allocator = std::make_shared<ifm3d::Buffer::BufferAllocator>();
  _data = _buffer_allocator->Allocate(_size);
}

ifm3d::Buffer
ifm3d::Buffer::Clone() const
{
  Buffer copy(_cols, _rows, _nchannel, _data_format, _metadata);
  std::memcpy(copy.Ptr(0), _data, _size);
  return copy;
}

std::uint32_t
ifm3d::Buffer::Height() const
{
  return _rows;
}

std::uint32_t
ifm3d::Buffer::Width() const
{
  return _cols;
}

std::uint32_t
ifm3d::Buffer::NumChannels() const
{
  return _nchannel;
}

ifm3d::pixel_format
ifm3d::Buffer::DataFormat() const
{
  return _data_format;
}

size_t
ifm3d::Buffer::Size() const
{
  return _size;
}

ifm3d::json
ifm3d::Buffer::Metadata() const
{
  return _metadata;
}

ifm3d::buffer_id
ifm3d::Buffer::BufferId() const
{
  return _buffer_id;
}