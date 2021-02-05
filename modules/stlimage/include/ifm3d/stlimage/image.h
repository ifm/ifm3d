// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_STLIMAGE_IMAGE_H__
#define __IFM3D_STLIMAGE_IMAGE_H__

#include <cstdint>
#include <memory>
#include <vector>
#include <memory_resource>
#include <ifm3d/fg/byte_buffer.h>

namespace ifm3d
{
  template <typename T>
  struct SpacePoint
  {
    T x;
    T y;
    T z;
  };

  template <typename T>
  using Point3D = struct SpacePoint<T>;

  using Point3D_16S = Point3D<unsigned short>;
  using Point3D_32F = Point3D<float>;

  class Image
  {
  public:
    char* data_;
    std::allocator<char> data_alloc_;

  public:
    int cols_;
    int rows_;
    int ndim_;
    ifm3d::pixel_format pixel_format_;
    int pixel_size_;
    ~Image();

    void Create(const int& cols,
                const int& rows,
                const int& ndim,
                ifm3d::pixel_format format);

    /** @brief Returns a pointer to the specified matrix row.
     */
    template <typename T = unsigned char>
    auto
    ptr(const int& row) -> T*
    {
      return (T*)(data_ + row * cols_ * ndim_ * pixel_size_);
    }

    template <typename T = unsigned char>
    auto
    ptr(const int& row, const int& col) -> T*
    {
      return &(ptr<T>(row)[col]);
    }

    template <typename T>
    auto
    at(const int& index) -> T&
    {
      int idx = index * ndim_;
      return *((T*)data_ + idx);
    }

    template <typename T>
    auto
    at(const int& row, const int& col) -> T&
    {
      int idx = row * cols_ + col;
      return at<T>(idx);
    }

    template <typename T>
    void
    setTo(const T val, ifm3d::Image& mask)
    {
      for (int i = 0; i < rows_; i++)
        {
          for (int j = 0; j < cols_; j++)
            {
              int index = i * cols_ + j;
              if (mask.at<uint8_t>(index) != 0)
                {
                  T* ptr = (T*)((char*)data_ + index * ndim_ * pixel_size_);
                  for (int k = 0; k < ndim_; k++)
                    {
                      ptr[k] = val;
                    }
                }
            }
        }
    }

    template <typename T>
    struct Iterator
    {
      using iterator_category = std::random_access_iterator_tag;
      using difference_type = std::ptrdiff_t;
      using value_type = T;
      using pointer = T*;
      using reference = T&;

      Iterator(char* ptr)
      {
        m_ptr = (T*)ptr;
      }

      reference
      operator*() const
      {
        return *m_ptr;
      }

      pointer
      operator->()
      {
        return m_ptr;
      }

      Iterator&
      operator++()
      {
        m_ptr++;
        return *this;
      }

      Iterator
      operator++(int)
      {
        Iterator tmp = *this;
        ++(*this);
        return tmp;
      }

      friend bool
      operator==(const Iterator& a, const Iterator& b)
      {
        return a.m_ptr == b.m_ptr;
      };

      friend bool
      operator!=(const Iterator& a, const Iterator& b)
      {
        return a.m_ptr != b.m_ptr;
      };

    private:
      pointer m_ptr;
    };

    template <typename T>
    Iterator<T>
    begin()
    {
      return Iterator<T>(data_);
    }

    template <typename T>
    Iterator<T>
    end()
    {
      return Iterator<T>(data_ + rows_ * cols_ * ndim_ * pixel_size_);
    }

    /*Todo : default move anc copy construtor */
  };

  template <typename T>
  struct Adapter
  {
    Image& it;
    Adapter(Image& it) : it(it) { }
    auto
    begin()
    {
      return it.begin<T>();
    }
    auto
    end()
    {
      return it.end<T>();
    }
  };
} // end: namespace ifm3d

#endif // __IFM3D_STLIMAGE_IMAGE_H__
