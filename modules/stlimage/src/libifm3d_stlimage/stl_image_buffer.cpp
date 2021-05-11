/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/stlimage/stl_image_buffer.h>
#include <ifm3d/fg/byte_buffer.h>
#include <stl_image_buffer_impl.hpp>
#include <cstdint>
#include <vector>

//--------------------------------
// StlImageBuffer class
//--------------------------------

ifm3d::StlImageBuffer::StlImageBuffer()
  : ifm3d::ByteBuffer<ifm3d::StlImageBuffer>(),
    pImpl(new ifm3d::StlImageBuffer::Impl())
{ }

ifm3d::StlImageBuffer::~StlImageBuffer() = default;

// move ctor
ifm3d::StlImageBuffer::StlImageBuffer(ifm3d::StlImageBuffer&& src_buff)
  : ifm3d::StlImageBuffer::StlImageBuffer()
{
  this->SetBytes(src_buff.bytes_, false);
}

// move assignment
ifm3d::StlImageBuffer&
ifm3d::StlImageBuffer::operator=(ifm3d::StlImageBuffer&& src_buff)
{
  this->SetBytes(src_buff.bytes_, false);
  return *this;
}

// copy ctor
ifm3d::StlImageBuffer::StlImageBuffer(const ifm3d::StlImageBuffer& src_buff)
  : ifm3d::StlImageBuffer::StlImageBuffer()
{
  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
}

// copy assignment operator
ifm3d::StlImageBuffer&
ifm3d::StlImageBuffer::operator=(const ifm3d::StlImageBuffer& src_buff)
{
  if (this == &src_buff)
    {
      return *this;
    }

  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
  return *this;
}

ifm3d::Image
ifm3d::StlImageBuffer::DistanceImage()
{
  this->Organize();
  return this->pImpl->DistanceImage();
}

ifm3d::Image
ifm3d::StlImageBuffer::UnitVectors()
{
  this->Organize();
  return this->pImpl->UnitVectors();
}

ifm3d::Image
ifm3d::StlImageBuffer::GrayImage()
{
  this->Organize();
  return this->pImpl->GrayImage();
}

ifm3d::Image
ifm3d::StlImageBuffer::AmplitudeImage()
{
  this->Organize();
  return this->pImpl->AmplitudeImage();
}

ifm3d::Image
ifm3d::StlImageBuffer::RawAmplitudeImage()
{
  this->Organize();
  return this->pImpl->RawAmplitudeImage();
}

ifm3d::Image
ifm3d::StlImageBuffer::ConfidenceImage()
{
  this->Organize();
  return this->pImpl->ConfidenceImage();
}

ifm3d::Image
ifm3d::StlImageBuffer::XYZImage()
{
  this->Organize();
  return this->pImpl->XYZImage();
}

void
ifm3d::StlImageBuffer::_ImCreate(ifm3d::image_chunk im,
                                 std::uint32_t fmt,
                                 std::size_t idx,
                                 std::uint32_t width,
                                 std::uint32_t height,
                                 int nchan,
                                 std::uint32_t npts,
                                 const std::vector<std::uint8_t>& bytes)
{
  this->pImpl->ImCreate(im, fmt, idx, width, height, nchan, npts, bytes);
}

void
ifm3d::StlImageBuffer::_CloudCreate(std::uint32_t fmt,
                                    std::size_t xidx,
                                    std::size_t yidx,
                                    std::size_t zidx,
                                    std::uint32_t width,
                                    std::uint32_t height,
                                    std::uint32_t npts,
                                    const std::vector<std::uint8_t>& bytes)
{
  this->pImpl->CloudCreate(fmt, xidx, yidx, zidx, width, height, npts, bytes);
}
