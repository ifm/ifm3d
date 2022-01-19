// -*- c++ -*-
/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_STLIMAGE_IMAGE_INL_HPP
#define IFM3D_STLIMAGE_IMAGE_INL_HPP

#include <ifm3d/stlimage/image.h>
#include <cstdint>
#include <memory>
#include <vector>

///////////////////////////Image //////////////////

template <typename T>
T*
ifm3d::Image::ptr(const std::uint32_t row)
{
  return reinterpret_cast<T*>(data_ + row * bytes_per_row);
}

template <typename T>
T*
ifm3d::Image::ptr(const std::uint32_t row, const std::uint32_t col)
{
  return reinterpret_cast<T*>(
    (data_ + row * bytes_per_row + col * bytes_per_pixel));
}

template <typename T>
T&
ifm3d::Image::at(const std::size_t index)
{
  auto idx = index * bytes_per_pixel;
  return *(reinterpret_cast<T*>((data_ + idx)));
}

template <typename T>
T&
ifm3d::Image::at(const std::uint32_t row, const std::uint32_t col)
{
  auto idx = row * cols_ + col;
  return at<T>(idx);
}

template <typename T>
void
ifm3d::Image::setTo(const T val, ifm3d::Image& mask)
{
  for (std::uint32_t i = 0; i < rows_; i++)
    {
      for (std::uint32_t j = 0; j < cols_; j++)
        {
          std::uint32_t index = i * cols_ + j;
          if (mask.at<uint8_t>(index) != 0)
            {
              T* ptr = reinterpret_cast<T*>(data_ + index * nchannel_ *
                                                      data_size_in_bytes_);
              for (std::uint32_t k = 0; k < nchannel_; k++)
                {
                  ptr[k] = val;
                }
            }
        }
    }
}
// Iterator function

template <typename T>
ifm3d::Image::Iterator<T>
ifm3d::Image::begin()
{
  return Iterator<T>(data_);
}

template <typename T>
ifm3d::Image::Iterator<T>
ifm3d::Image::end()
{
  return Iterator<T>(data_ + size_);
}

// Iterator Adapter
template <typename T>
ifm3d::IteratorAdapter<T>::IteratorAdapter(ifm3d::Image& it) : it(it)
{}

template <typename T>
auto
ifm3d::IteratorAdapter<T>::begin()
{
  return it.begin<T>();
}

template <typename T>
auto
ifm3d::IteratorAdapter<T>::end()
{
  return it.end<T>();
}

//*** Iterators ***//
template <typename T>
ifm3d::Image::Iterator<T>::Iterator(std::uint8_t* ptr)
{
  m_ptr = (T*)ptr;
}

template <typename T>
typename ifm3d::Image::Iterator<T>::reference
ifm3d::Image::Iterator<T>::operator*() const
{
  return *m_ptr;
}

template <typename T>
typename ifm3d::Image::Iterator<T>::pointer
ifm3d::Image::Iterator<T>::operator->()
{
  return m_ptr;
}

template <typename T>
ifm3d::Image::Iterator<T>&
ifm3d::Image::Iterator<T>::operator++()
{
  m_ptr++;
  return *this;
}

template <typename T>
ifm3d::Image::Iterator<T>
ifm3d::Image::Iterator<T>::operator++(int)
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
    using value_type = T;
    using data_type = T;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_8U),
      nchannel = 1
    };
  };

  template <>
  struct FormatType<uint8_t>
  {
    using value_type = uint8_t;
    using data_type = uint8_t;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_8U),
      nchannel = 1
    };
  };

  template <>
  struct FormatType<int8_t>
  {
    using value_type = int8_t;
    using data_type = int8_t;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_8S),
      nchannel = 1
    };
  };

  template <>
  struct FormatType<uint16_t>
  {
    using value_type = uint16_t;
    using data_type = uint16_t;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_16U),
      nchannel = 1
    };
  };

  template <>
  struct FormatType<int16_t>
  {
    using value_type = uint16_t;
    using data_type = uint16_t;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_16S),
      nchannel = 1
    };
  };

  template <>
  struct FormatType<uint32_t>
  {
    using value_type = uint32_t;
    using data_type = uint32_t;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_32U),
      nchannel = 1
    };
  };

  template <>
  struct FormatType<int32_t>
  {
    using value_type = int32_t;
    using data_type = int32_t;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_32S),
      nchannel = 1
    };
  };

  template <>
  struct FormatType<float>
  {
    using value_type = float;
    using data_type = float;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_32F),
      nchannel = 1
    };
  };

  template <>
  struct FormatType<double>
  {
    using value_type = double;
    using data_type = double;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_64F),
      nchannel = 1
    };
  };

  template <>
  struct FormatType<uint64_t>
  {
    using value_type = uint64_t;
    using data_type = uint64_t;
    enum
    {
      format = static_cast<int>(ifm3d::pixel_format::FORMAT_64U),
      nchannel = 1
    };
  };

  template <typename T, int n>
  struct FormatType<ifm3d::point<T, n>>
  {
    using value_type = ifm3d::point<T, n>;
    using data_type = T;
    enum
    {
      format = ifm3d::FormatType<T>::format,
      nchannel = n
    };
  };

}
////////////////////////////Image_<Tp>//////////////

template <typename Tp>
ifm3d::Image_<Tp>::Image_() : ifm3d::Image(){};

template <typename Tp>
ifm3d::Image_<Tp>::Image_(const std::uint32_t cols, const std::uint32_t rows)
  : ifm3d::Image(
      cols,
      rows,
      static_cast<std::uint32_t>(ifm3d::FormatType<Tp>::nchannel),
      static_cast<ifm3d::pixel_format>(ifm3d::FormatType<Tp>::format))
{}

template <typename Tp>
ifm3d::Image_<Tp>::Image_(const Image& img) : Image()
{
  *this = img;
}

template <typename Tp>
ifm3d::Image_<Tp>&
ifm3d::Image_<Tp>::operator=(const Image& img)
{
  if (static_cast<ifm3d::pixel_format>(ifm3d::FormatType<Tp>::format) ==
        img.dataFormat() &&
      static_cast<uint32_t>(ifm3d::FormatType<Tp>::nchannel) ==
        img.nchannels())
    {
      Image::operator=(img);
      return *this;
    }
  else
    {
      throw std::runtime_error(
        "cannot convert due to type or channel mistmatch");
    }
}
template <typename Tp>
void
ifm3d::Image_<Tp>::create(const std::uint32_t cols, const std::uint32_t rows)
{
  ifm3d::Image::create(
    cols,
    rows,
    static_cast<uint32_t>(ifm3d::FormatType<Tp>::nchannel),
    static_cast<ifm3d::pixel_format>(ifm3d::FormatType<Tp>::format));
}

template <typename Tp>
ifm3d::Image_<Tp>
ifm3d::Image_<Tp>::clone() const
{
  return Image_<Tp>(Image::clone());
}

/* getters*/
template <typename Tp>
std::uint32_t
ifm3d::Image_<Tp>::height() const
{
  return Image::height();
}
template <typename Tp>
std::uint32_t
ifm3d::Image_<Tp>::width() const
{
  return Image::width();
}
template <typename Tp>
std::uint32_t
ifm3d::Image_<Tp>::nchannels() const
{
  return Image::nchannels();
}
template <typename Tp>
ifm3d::pixel_format
ifm3d::Image_<Tp>::dataFormat() const
{
  return Image::dataFormat();
}

template <typename Tp>
Tp*
ifm3d::Image_<Tp>::ptr(const std::uint32_t row)
{
  return Image::ptr<Tp>(row);
}

template <typename Tp>
Tp*
ifm3d::Image_<Tp>::ptr(const std::uint32_t row, const std::uint32_t col)
{
  return Image::ptr<Tp>(row, col);
}

template <typename Tp>
Tp&
ifm3d::Image_<Tp>::at(const std::size_t index)
{
  return Image::at<Tp>(index);
}

template <typename Tp>
Tp&
ifm3d::Image_<Tp>::at(const std::uint32_t row, const std::uint32_t col)
{
  return Image::at<Tp>(row, col);
}

template <typename Tp>
void
ifm3d::Image_<Tp>::setTo(const Tp val, ifm3d::Image& mask)
{
  return Image::setTo<Tp>(val, mask);
}

template <typename Tp>
ifm3d::Image::Iterator<Tp>
ifm3d::Image_<Tp>::begin()
{
  return Image::begin<Tp>();
}

template <typename Tp>
ifm3d::Image::Iterator<Tp>
ifm3d::Image_<Tp>::end()
{
  return Image::end<Tp>();
}

////////conversion helper //////

namespace ifm3d
{

  template <typename FROM, typename TO>
  class conversion
  {
  public:
    TO
    operator()(const FROM& val)
    {
      return (TO)(val);
    }
  };

  template <typename FROM_T, typename TO_T, int n>
  class conversion<ifm3d::point<FROM_T, n>, ifm3d::point<TO_T, n>>
  {
  public:
    ifm3d::point<TO_T, n>
    operator()(ifm3d::point<FROM_T, n>& in)
    {
      ifm3d::point<TO_T, n> out;
      for (int i = 0; i < n; i++)
        {
          out.val[i] = static_cast<TO_T>(in.val[i]);
        }
      return out;
    }
  };

  template <typename FROM, typename TO>
  ifm3d::Image_<TO>
  convert_to(ifm3d::Image_<FROM>& img)
  {
    if (static_cast<uint32_t>(ifm3d::FormatType<TO>::nchannel ==
                              img.nchannels()))
      {
        Image_<TO> out = ifm3d::Image(
          img.width(),
          img.height(),
          img.nchannels(),
          (static_cast<ifm3d::pixel_format>(ifm3d::FormatType<TO>::format)));

        ifm3d::Image_<FROM> image_from = img;

        if (std::is_convertible<
              typename ifm3d::FormatType<FROM>::data_type,
              typename ifm3d::FormatType<TO>::data_type>::value)
          {
            std::transform(img.begin(),
                           img.end(),
                           out.begin(),
                           ifm3d::conversion<FROM, TO>());
            return out;
          }
      }
    throw std::runtime_error("cannot convert due to type mistmatch");
  }
}

#endif // IFM3D_STLIMAGE_IMAGE_INL_H
