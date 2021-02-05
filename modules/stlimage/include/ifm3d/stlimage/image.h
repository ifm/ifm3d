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
  /**
  @brief The class Image represent a STL conatiner to store image data  from the ifm devices
  in 2 dimension and supports multiple channel. data is stores in sequnetial memory layout and 
  class provides function template to access the pixel.

  Creating an Image object :

- Use the Create(rows, cols, nchannel, ifm3d::pixel_format ) method or the similar
Image(nrows, ncols, nchannel, type) constructor. A new array of the 
specified size and type is allocated. type has the value from ifm3d::pixel_format.

For example, FORMAT_8U means a 8-bit array, FORMAT_32F floating-point array, and so on.
@code
    //an 100 x 100 Image of type 8U
    ifm3d::Image image(100,100,1,ifm3d::FORMAT_8U);
    // and now turn M to a 10 x10 3-channel 8-bit matrix.
    // The old content will be deallocated
    M.create(10,10,3,ifm3d::FORMAT_8U);
@endcode
As noted in the introduction to this chapter, create() allocates only a new array when
the memory requiremnt changes for new Image

- Accessing the pixels 
use at<T>(index) or at<T>(i,j) to access the pixel this return the reference to the pixel. 
A pixel is defined as structure of n-channel values at a given index or pixel position in 2D array
 
to access a pixel in Image I ( 100,100,1,ifm3d:: FORMAT_8U) at  50,50 position 

 @code

 auto pixel = I<uint8_t>(50,50);
 // if working as Index array then 
 
 auto index = 50*100 + 50 ;
 auto pixel = I<uint8_t>(index);

 @endcode 

 changing the pixel value can be done as follow :
 writing 100 at pixel postion 50,50

 @code 
 I<uint8_t>(50,50) = 100;
 I<uint8_t>(index) = 100;
 @endcode

 to access a pixel in n-channel Image I ( 100,100,3,ifm3d:: FORMAT_8U) at  50,50 position
 This will be the case accessing the values for 3 channel Image 

 as pixel is structure of the values of n-chanel at given position. 

 @code 

 auto pixel = I<Point3D<uint8_t>>(50,50);
 
 //now individual channel values can be access with 
 value.x, value.y , value.z
 @endcode

 -Processing the whole array 
 If you need to process a whole Image, the most efficient way is to
get the pointer to the row first, and then just use the plain C operator [] :
@code
   Image I(100,100,1,FORMAT_8U);
    for(int i = 0; i < I.rows; i++)
    {
        const uint8_t* rowi = M.ptr<uint8_t>(i);
        for(int j = 0; j < I.cols; j++)
            {
              //some operation here 
            }
    }
@endcode

One can aslo use range based for loops with adapter explained
in ifm3d::IteratorAdapter section
  */
  class Image
  {
  private:
    /* @ brief raw pointer to the data*/
    char* data_;
    /* @brief memory allocator */
    std::allocator<char> data_alloc_;
    /*@brief number of columns in Image (width)*/
    uint32_t cols_;
    /*@brief number of rows in Image (height)*/
    uint32_t rows_;
    /*@brief number of channel in Image*/
    uint32_t nchannel_;
    /* @brief data format or type*/
    ifm3d::pixel_format data_format_;
    /* @brief number of pixel to store one value of data*/
    uint32_t data_size_in_bytes_;
    /* @brief size of the memory allocated*/
    size_t size_;

    public:
    /**
      These are various constructors that form a Image.
      default constructor for forming a Image user furher 
      needs to call create Method to actually allocates the 
      Memory
    */
    Image() = default;
    /*@overload
      @param cols Number of columns in a Image.
      @param rows Number of rows in a Image.
      @param nchannel Number of channels in Image
      @param format value from ifm3d::pixel_format releates to data type 
      need to store one value. 

      @note This internally called Create Method to allocates Memory
  */ 
    Image(const int& cols,
          const int& rows,
          const int& nchannel,
          ifm3d::pixel_format format);

    ~Image();
    /*@brief allocates the memory required for storing the image data
      @param cols Number of columns in a Image.
      @param rows Number of rows in a Image.
      @param nchannel Number of channels in Image
      @param format value from ifm3d::pixel_format releates to data type

      @Note On repeated calling it will deference the old Memory
     */
    void Create(const int& cols,
                const int& rows,
                const int& nchannel,
                ifm3d::pixel_format format);

    /* getters*/
     uint32_t Height() const;
     uint32_t Width() const;
     uint32_t Nchannels() const;
     ifm3d::pixel_format DataFormat() const;

    /** @brief returns a pointer to the specified Image row.
        @param row number 
     */
    template <typename T = unsigned char>
    auto ptr(const int& row) -> T*;
   
    /**
     @brief Pointer to the Pixel at row,col
     @param row 1st dimension index
     @param col 2nd dimension index
     */
    template <typename T = unsigned char>
    auto ptr(const int& row, const int& col) -> T*;

    /*@brief access to the pixel for read  and write
      @param Index of the pixel considering image as 1D array
      @return refernce of the value at the index
    */
    template <typename T>
    auto at(const int& index) -> T&;
    /*@overload considering image as 2D 
      @param row 1st dimension index 
      @param col 2nd dimension index
    */
    template <typename T>
    auto at(const int& row, const int& col) -> T&;
    /*@brief set the value where mask value is 1
      @param val  value to be set
      @param mask  Binary mask 

      @Note mask size must be same as this
    */
    template <typename T>
    void setTo(const T val, ifm3d::Image& mask);

/*===========================*/
/*  Iterators */
/*===========================*/
    template <typename T>
    struct Iterator
    {
      using iterator_category = std::random_access_iterator_tag;
      using difference_type = std::ptrdiff_t;
      using value_type = T;
      using pointer = T*;
      using reference = T&;

      Iterator(char* ptr);
      reference operator*() const;
      pointer operator->();
      Iterator& operator++();
      Iterator operator++(int);

      friend bool operator==(const Iterator& a, const Iterator& b)
      {
        return a.m_ptr == b.m_ptr;
      }

      friend bool operator!=(const Iterator& a, const Iterator& b)
      {
        return a.m_ptr != b.m_ptr;
      }

    private:
      pointer m_ptr;
    };

    /*@brief Return the Iterator pointing to start of data*/
    template <typename T>
    Iterator<T> begin();
    /*@brief Return the Iterator pointing to end of data*/
    template <typename T>
    Iterator<T> end();

    /*Todo : Rule of 5 */
  }; // end Image

  /*@brief IteratorAdapter is adapter and can be used in range based loops

  @code
  for (auto value : ifm3d::IteratorAdapter<unsigned short>(image))
  {
      // operation on value
  @endcode
  */
  template <typename T>
  class IteratorAdapter
  {
  private:
    Image& it;

  public:
    IteratorAdapter(Image& it);
    auto begin();
    auto end();
  };

  /**
   * @brief Struct for 3D space point
   */
  template <typename T>
  struct point3d
  {
    T x;
    T y;
    T z;
  };

  template <typename T>
  using Point3D = struct point3d<T>;

  // user types
  using Point3D_16S = Point3D<unsigned short>;
  using Point3D_32F = Point3D<float>;

} // end: namespace ifm3d

#include <ifm3d/stlimage/image_inl.hpp>
#endif // __IFM3D_STLIMAGE_IMAGE_H__
