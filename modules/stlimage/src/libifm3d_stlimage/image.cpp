/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/stlimage/image.h>
#include <ifm3d/fg/byte_buffer.h>
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <limits>

namespace ifm3d
 {
 std::unordered_map<uint32_t, std::size_t> PIX_SZ{
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U), 1},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S), 1},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U), 2},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S), 2},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S), 4},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), 4},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F), 8}};

//--------------------------------
 // ImageAllocator class
 //--------------------------------

 class ifm3d::Image::ImageAllocator
 {
   /* @ brief raw pointer to the data*/
   char* data_;
   /* @brief memory allocator */
   std::allocator<char> data_alloc_;
   /*@brief size of current allocation*/
   size_t size_;

   public:
   ImageAllocator::ImageAllocator()
     : data_(NULL),
     size_(0)
   {}

   ImageAllocator::~ImageAllocator() {
     if (data_ != NULL)
       {
         deallocate();
       }
   }
   public:
   char* allocate(size_t size)
   {
     data_ = data_alloc_.allocate(size);
     if (data_ != NULL)
     {
       size_= size;
       return data_;
     }
     else
     {
         throw std::runtime_error("cannot allocate memory");
     }
   }
   void deallocate()
   {
     data_alloc_.deallocate(data_,size_);
     data_ = NULL;
   }

   char* data()
   {
     return data_;
   }
 };
 }

//--------------------------------
// Image class
//--------------------------------

 ifm3d::Image::Image()
   : data_(NULL)
   , rows_(0)
   , cols_(0)
   , nchannel_(0)
   , data_size_in_bytes_(0)
   , size_(0)
 {
 }

 ifm3d::Image::Image(const int& cols,
                     const int& rows,
                     const int& nchannel,
                     ifm3d::pixel_format format)
 {
   Create(cols, rows, nchannel, format);
 }

void ifm3d::Image::Create(const int& cols,
                          const int& rows,
                          const int& nchannel,
                          ifm3d::pixel_format format)
{
  cols_ =cols;
  rows_ = rows;
  nchannel_ = nchannel ;
  data_format_ = format;
  data_size_in_bytes_ = PIX_SZ[static_cast<std::uint32_t>(format)];
  size_ = cols * rows * nchannel_ * data_size_in_bytes_;
  image_allocator_ = std::make_shared<ifm3d::Image::ImageAllocator>();
  data_ = image_allocator_->allocate(size_);
}

 uint32_t ifm3d::Image:: Height() const
{
  return cols_;
}

uint32_t
ifm3d::Image::Width() const
{
  return rows_;
}
uint32_t ifm3d::Image::Nchannels() const
{
  return nchannel_;
}
ifm3d::pixel_format ifm3d::Image::DataFormat() const
{
  return data_format_;
}
