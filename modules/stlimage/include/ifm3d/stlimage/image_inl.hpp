// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_STLIMAGE_IMAGE_INL_HPP__
#define IFM3D_STLIMAGE_IMAGE_INL_HPP__

#include <cstdint>
#include <memory>
#include <vector>
#include <memory_resource>
#include <ifm3d/stlimage/image.h>

template <typename T>
auto
ifm3d::Image::ptr(const int& row) -> T*
{
  return (T*)(data_ + row * cols_ * nchannel_ * data_size_in_bytes_);
}

template <typename T>
auto
ifm3d::Image::ptr(const int& row, const int& col) -> T*
{
  return &(ptr<T>(row)[col]);
}

template <typename T>
auto
ifm3d::Image::at(const int& index) -> T&
{
  int idx = index * nchannel_;
  return *((T*)data_ + idx);
}

template <typename T>
auto
ifm3d::Image::at(const int& row, const int& col) -> T&
{
  int idx = row * cols_ + col;
  return at<T>(idx);
}
template <typename T>
void
ifm3d::Image::setTo(const T val, ifm3d::Image& mask)
{
  for (int i = 0; i < rows_; i++)
    {
      for (int j = 0; j < cols_; j++)
        {
          int index = i * cols_ + j;
          if (mask.at<uint8_t>(index) != 0)
            {
              T* ptr =
                (T*)((char*)data_ + index * nchannel_ * data_size_in_bytes_);
              for (int k = 0; k < nchannel_; k++)
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

//Iterator Adapter
template <typename T>
ifm3d::IteratorAdapter<T>::IteratorAdapter(ifm3d::Image& it) : it(it)
{ }

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
ifm3d::Image::Iterator<T>::Iterator(char* ptr)
{
  m_ptr = (T*)ptr;
}

template <typename T>
typename ifm3d::Image::Iterator<T>::reference
ifm3d::Image::Iterator<T>::operator*() const
{
  return *m_ptr;
}

template<typename T>
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
#endif // IFM3D_STLIMAGE_IMAGE_INL_H__
