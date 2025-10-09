// -*- c++ -*-
/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_BUFFER_IMPL_HPP
#define IFM3D_FG_BUFFER_IMPL_HPP
#pragma once

#include <cstddef>
#include <cstdint>
#include <ifm3d/fg/buffer.h> // NOLINT(misc-header-include-cycle)
#include <ifm3d/fg/buffer_id.h>

///////////////////////////Image //////////////////

template <typename T>
T*
ifm3d::Buffer::Ptr(const std::uint32_t row)
{
  return reinterpret_cast<T*>(_data + (row * _bytes_per_row));
}

template <typename T>
T*
ifm3d::Buffer::Ptr(const std::uint32_t row, const std::uint32_t col)
{
  return reinterpret_cast<T*>(
    (_data + (row * _bytes_per_row) + (col * _bytes_per_pixel)));
}

template <typename T>
T const*
ifm3d::Buffer::Ptr(const std::uint32_t row) const
{
  return reinterpret_cast<T*>(_data + (row * _bytes_per_row));
}

template <typename T>
T const*
ifm3d::Buffer::Ptr(const std::uint32_t row, const std::uint32_t col) const
{
  return reinterpret_cast<T*>(
    (_data + (row * _bytes_per_row) + (col * _bytes_per_pixel)));
}

template <typename T>
T&
ifm3d::Buffer::At(const std::size_t index)
{
  auto idx = index * _bytes_per_pixel;
  return *(reinterpret_cast<T*>((_data + idx)));
}

template <typename T>
T&
ifm3d::Buffer::At(std::uint32_t row, std::uint32_t col)
{
  auto idx = (row * _cols) + col;
  return At<T>(idx);
}

template <typename T>
T const&
ifm3d::Buffer::At(std::size_t index) const
{
  auto idx = index * _bytes_per_pixel;
  return *(reinterpret_cast<T*>((_data + idx)));
}

template <typename T>
T const&
ifm3d::Buffer::At(std::uint32_t row, std::uint32_t col) const
{
  auto idx = (row * _cols) + col;
  return At<T>(idx);
}

template <typename T>
void
ifm3d::Buffer::SetTo(const T val, const ifm3d::Buffer& mask)
{
  for (std::uint32_t i = 0; i < _rows; i++)
    {
      for (std::uint32_t j = 0; j < _cols; j++)
        {
          std::uint32_t index = (i * _cols) + j;
          if (mask.At<uint8_t>(index) != 0)
            {
              T* ptr = reinterpret_cast<T*>(_data +
                                            (static_cast<std::size_t>(index) *
                                             _nchannel * _data_size_in_bytes));
              for (std::uint32_t k = 0; k < _nchannel; k++)
                {
                  ptr[k] = val;
                }
            }
        }
    }
}
// Iterator function

template <typename T>
ifm3d::Buffer::Iterator<T>
ifm3d::Buffer::begin()
{
  return Iterator<T>(_data);
}

template <typename T>
ifm3d::Buffer::Iterator<T>
ifm3d::Buffer::end()
{
  return Iterator<T>(_data + _size);
}

// Iterator Adapter
template <typename T>
ifm3d::IteratorAdapter<T>::IteratorAdapter(ifm3d::Buffer& it) : _it(it)
{}

template <typename T>
auto
ifm3d::IteratorAdapter<T>::begin()
{
  return _it.begin<T>();
}

template <typename T>
auto
ifm3d::IteratorAdapter<T>::end()
{
  return _it.end<T>();
}

//*** Iterators ***//
template <typename T>
ifm3d::Buffer::Iterator<T>::Iterator(const std::uint8_t* ptr) : _ptr((T*)ptr)
{}

template <typename T>
typename ifm3d::Buffer::Iterator<T>::reference
ifm3d::Buffer::Iterator<T>::operator*() const
{
  return *_ptr;
}

template <typename T>
typename ifm3d::Buffer::Iterator<T>::pointer
ifm3d::Buffer::Iterator<T>::operator->()
{
  return _ptr;
}

template <typename T>
ifm3d::Buffer::Iterator<T>&
ifm3d::Buffer::Iterator<T>::operator++()
{
  _ptr++;
  return *this;
}

template <typename T>
ifm3d::Buffer::Iterator<T>
ifm3d::Buffer::Iterator<T>::operator++(int)
{
  Iterator tmp = *this;
  ++(*this);
  return tmp;
}

////////////////////////////// FormatType<Tp>//////////

namespace ifm3d
{

  template <typename T>
  struct FormatType
  {
    using ValueType = T;
    using DataType = T;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_8U),
      NumChannels = 1
    };
  };

  template <>
  struct FormatType<uint8_t>
  {
    using ValueType = uint8_t;
    using DataType = uint8_t;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_8U),
      NumChannels = 1
    };
  };

  template <>
  struct FormatType<int8_t>
  {
    using ValueType = int8_t;
    using DataType = int8_t;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_8S),
      NumChannels = 1
    };
  };

  template <>
  struct FormatType<uint16_t>
  {
    using ValueType = uint16_t;
    using DataType = uint16_t;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_16U),
      NumChannels = 1
    };
  };

  template <>
  struct FormatType<int16_t>
  {
    using ValueType = uint16_t;
    using DataType = uint16_t;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_16S),
      NumChannels = 1
    };
  };

  template <>
  struct FormatType<uint32_t>
  {
    using ValueType = uint32_t;
    using DataType = uint32_t;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_32U),
      NumChannels = 1
    };
  };

  template <>
  struct FormatType<int32_t>
  {
    using ValueType = int32_t;
    using DataType = int32_t;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_32S),
      NumChannels = 1
    };
  };

  template <>
  struct FormatType<float>
  {
    using ValueType = float;
    using DataType = float;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_32F),
      NumChannels = 1
    };
  };

  template <>
  struct FormatType<double>
  {
    using ValueType = double;
    using DataType = double;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_64F),
      NumChannels = 1
    };
  };

  template <>
  struct FormatType<uint64_t>
  {
    using ValueType = uint64_t;
    using DataType = uint64_t;
    enum : uint8_t
    {
      Format = static_cast<int>(ifm3d::PixelFormat::FORMAT_64U),
      NumChannels = 1
    };
  };

  template <typename T, int N>
  struct FormatType<ifm3d::Point<T, N>>
  {
    using ValueType = ifm3d::Point<T, N>;
    using DataType = T;
    enum : uint8_t
    {
      Format = ifm3d::FormatType<T>::Format,
      NumChannels = N
    };
  };

}
////////////////////////////Buffer_<Tp>//////////////

template <typename TP>
ifm3d::Buffer_<TP>::Buffer_() : ifm3d::Buffer(){};

template <typename TP>
ifm3d::Buffer_<TP>::Buffer_(const std::uint32_t cols,
                            const std::uint32_t rows,
                            std::optional<ifm3d::json> metadata)
  : ifm3d::Buffer(
      cols,
      rows,
      static_cast<std::uint32_t>(ifm3d::FormatType<TP>::NumChannels),
      static_cast<ifm3d::PixelFormat>(ifm3d::FormatType<TP>::Format),
      metadata)
{}

template <typename TP>
ifm3d::Buffer_<TP>::Buffer_(const Buffer& img) : Buffer()
{
  Buffer::operator=(img);
}

template <typename TP>
ifm3d::Buffer_<TP>&
ifm3d::Buffer_<TP>::operator=(const Buffer& img)
{
  if (this->DataFormat() == img.DataFormat() &&
      this->Nchannels() == img.NumChannels())
    {
      Buffer::operator=(img);
      return *this;
    }

  throw std::runtime_error("cannot convert due to type or channel mistmatch");
}
template <typename TP>
void
ifm3d::Buffer_<TP>::Create(const std::uint32_t cols,
                           const std::uint32_t rows,
                           ifm3d::buffer_id buffer_id)
{
  ifm3d::Buffer::Create(
    cols,
    rows,
    static_cast<uint32_t>(ifm3d::FormatType<TP>::NumChannels),
    static_cast<ifm3d::PixelFormat>(ifm3d::FormatType<TP>::Format),
    buffer_id);
}

template <typename TP>
ifm3d::Buffer_<TP>
ifm3d::Buffer_<TP>::Clone() const
{
  return Buffer_<TP>(Buffer::Clone());
}

/* getters*/
template <typename TP>
std::uint32_t
ifm3d::Buffer_<TP>::Height() const
{
  return Buffer::Height();
}
template <typename TP>
std::uint32_t
ifm3d::Buffer_<TP>::Width() const
{
  return Buffer::Width();
}
template <typename TP>
std::uint32_t
ifm3d::Buffer_<TP>::Nchannels() const
{
  return Buffer::NumChannels();
}
template <typename TP>
ifm3d::PixelFormat
ifm3d::Buffer_<TP>::DataFormat() const
{
  return Buffer::DataFormat();
}

template <typename TP>
ifm3d::json
ifm3d::Buffer_<TP>::Metadata() const
{
  return Buffer::Metadata();
}

template <typename TP>
TP*
ifm3d::Buffer_<TP>::Ptr(const std::uint32_t row)
{
  return Buffer::Ptr<TP>(row);
}

template <typename TP>
TP*
ifm3d::Buffer_<TP>::Ptr(const std::uint32_t row, const std::uint32_t col)
{
  return Buffer::Ptr<TP>(row, col);
}

template <typename TP>
TP&
ifm3d::Buffer_<TP>::At(const std::size_t index)
{
  return Buffer::At<TP>(index);
}

template <typename TP>
TP&
ifm3d::Buffer_<TP>::At(const std::uint32_t row, const std::uint32_t col)
{
  return Buffer::At<TP>(row, col);
}

template <typename TP>
void
ifm3d::Buffer_<TP>::SetTo(const TP val, ifm3d::Buffer& mask)
{
  return Buffer::SetTo<TP>(val, mask);
}

template <typename TP>
ifm3d::Buffer::Iterator<TP>
ifm3d::Buffer_<TP>::begin()
{
  return Buffer::begin<TP>();
}

template <typename TP>
ifm3d::Buffer::Iterator<TP>
ifm3d::Buffer_<TP>::end()
{
  return Buffer::end<TP>();
}

////////conversion helper //////

namespace ifm3d
{

  template <typename FROM, typename TO>
  class Conversion
  {
  public:
    TO
    operator()(const FROM& val)
    {
      return (TO)(val);
    }
  };

  template <typename FROM_T, typename TO_T, int N>
  class Conversion<ifm3d::Point<FROM_T, N>, ifm3d::Point<TO_T, N>>
  {
  public:
    ifm3d::Point<TO_T, N>
    operator()(ifm3d::Point<FROM_T, N>& in)
    {
      ifm3d::Point<TO_T, N> out;
      for (int i = 0; i < N; i++)
        {
          out.val[i] = static_cast<TO_T>(in.val[i]);
        }
      return out;
    }
  };

  template <typename FROM, typename TO>
  ifm3d::Buffer_<TO>
  convert_to(ifm3d::Buffer_<FROM>& img)
  {
    if (static_cast<uint32_t>(ifm3d::FormatType<TO>::NumChannels ==
                              img.Nchannels()))
      {
        Buffer_<TO> out = ifm3d::Buffer(
          img.Width(),
          img.Height(),
          img.Nchannels(),
          (static_cast<ifm3d::PixelFormat>(ifm3d::FormatType<TO>::Format)));

        ifm3d::Buffer_<FROM> image_from = img;

        if (std::is_convertible_v<typename ifm3d::FormatType<FROM>::data_type,
                                  typename ifm3d::FormatType<TO>::data_type>)
          {
            std::transform(img.begin(),
                           img.end(),
                           out.begin(),
                           ifm3d::Conversion<FROM, TO>());
            return out;
          }
      }
    throw std::runtime_error("cannot convert due to type mistmatch");
  }
}

#endif // IFM3D_CAMERA_IMAGE_INL_H
