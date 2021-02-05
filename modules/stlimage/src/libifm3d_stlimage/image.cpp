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

 std::unordered_map<uint32_t, std::size_t> PIX_SZ{
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U), 1},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S), 1},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U), 2},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S), 2},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S), 4},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), 4},
  {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F), 8}};


//--------------------------------
// Image class
//--------------------------------

void ifm3d::Image::Create(const int& cols,
                 const int& rows,
                 const int& ndim,
                 ifm3d::pixel_format format)
{
  cols_ =cols;
  rows_ = rows;
  ndim_ = ndim;
  pixel_format_ = format;
  pixel_size_ = PIX_SZ[static_cast<std::uint32_t>(format)];
  const int buffer_size =
      cols * rows * ndim * pixel_size_;
  data_ = data_alloc_.allocate(buffer_size);
}

ifm3d::Image::~Image()
{
  //data_alloc_.deallocate(data_, cols_ * rows_ * ndim_ * pixel_size_);
}


