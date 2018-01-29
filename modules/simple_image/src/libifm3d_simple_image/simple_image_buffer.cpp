/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 * Copyright (C) 2018 ifm syntron gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ifm3d/simple_image/simple_image_buffer.h>
#include <cstdint>
#include <vector>
#include <ifm3d/fg/byte_buffer.h>
#include <simple_image_buffer_impl.hpp>

//--------------------------------
// ImageBuffer class
//--------------------------------

ifm3d::SimpleImageBuffer::SimpleImageBuffer()
  : ifm3d::ByteBuffer(),
    pImpl(new ifm3d::SimpleImageBuffer::Impl())
{ }

ifm3d::SimpleImageBuffer::~SimpleImageBuffer() = default;

ifm3d::SimpleImageBuffer::SimpleImageBuffer(const ifm3d::SimpleImageBuffer& src_buff)
  : ifm3d::ByteBuffer()
{
  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
}

ifm3d::SimpleImageBuffer&
ifm3d::SimpleImageBuffer::operator= (const ifm3d::SimpleImageBuffer& src_buff)
{
  if (this == &src_buff)
    {
      return *this;
    }

  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);

  return *this;
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::DistanceImage()
{
  this->Organize();
  return this->pImpl->DistanceImage();
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::UnitVectors()
{
  this->Organize();
  return this->pImpl->UnitVectors();
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::GrayImage()
{
  this->Organize();
  return this->pImpl->GrayImage();
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::AmplitudeImage()
{
  this->Organize();
  return this->pImpl->AmplitudeImage();
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::RawAmplitudeImage()
{
  this->Organize();
  return this->pImpl->RawAmplitudeImage();
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::ConfidenceImage()
{
  this->Organize();
  return this->pImpl->ConfidenceImage();
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::XYZImage()
{
  this->Organize();
  return this->pImpl->XYZImage();
}

std::vector<float>
ifm3d::SimpleImageBuffer::Extrinsics()
{
  this->Organize();
  return this->pImpl->Extrinsics();
}

std::vector<std::uint32_t>
ifm3d::SimpleImageBuffer::ExposureTimes()
{
  this->Organize();
  return this->pImpl->ExposureTimes();
}

void
ifm3d::SimpleImageBuffer::Organize()
{
  if (! this->Dirty())
    {
      return;
    }

  this->pImpl->Organize(this->bytes_);
  this->_SetDirty(false);
}
